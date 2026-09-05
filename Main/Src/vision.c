#include "vision.h"

#include "app_config.h"
#include "main.h"
#include "Uart.h"

#define FRAME_TYPE_INDEX       2U
#define FRAME_SEQUENCE_INDEX   3U
#define FRAME_PAYLOAD_INDEX    4U
#define FRAME_CRC_LOW_INDEX    12U
#define FRAME_CRC_HIGH_INDEX   13U
#define FRAME_TAIL_INDEX       14U
#define FRAME_CRC_INPUT_SIZE   10U

static volatile VisionData latest_data;
static uint8_t frame[VISION_FRAME_SIZE];
static uint8_t frame_index;
static uint8_t config_streak;
static uint8_t config_last_sequence;
static bool config_sequence_valid;
static volatile uint8_t ack_remaining;
static volatile bool status_pending;
static volatile bool odom_pending;
static uint8_t status_sequence;
static uint8_t odom_sequence;
static uint8_t tx_frame[VISION_FRAME_SIZE];
static uint8_t status_frame[VISION_FRAME_SIZE];
static uint8_t odom_frame[VISION_FRAME_SIZE];
static const uint8_t config_ack[4] = {
  VISION_FRAME_HEAD_1, VISION_FRAME_HEAD_2, 0x01U, VISION_FRAME_TAIL
};

static uint16_t vision_crc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < size; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = ((crc & 1U) != 0U) ?
          (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

static uint16_t vision_u16_be(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static bool vision_padding_zero(const uint8_t *payload, uint8_t first)
{
  for (uint8_t i = first; i < VISION_PAYLOAD_SIZE; ++i) {
    if (payload[i] != 0U) {
      return false;
    }
  }
  return true;
}

static uint8_t vision_next_sequence(uint8_t *sequence)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint8_t current = (*sequence)++;
  if (primask == 0U) {
    __enable_irq();
  }
  return current;
}

static void vision_build_frame(uint8_t *output, uint8_t type,
                               uint8_t sequence, const uint8_t *payload)
{
  output[0] = VISION_FRAME_HEAD_1;
  output[1] = VISION_FRAME_HEAD_2;
  output[FRAME_TYPE_INDEX] = type;
  output[FRAME_SEQUENCE_INDEX] = sequence;
  for (uint8_t i = 0U; i < VISION_PAYLOAD_SIZE; ++i) {
    output[FRAME_PAYLOAD_INDEX + i] = payload[i];
  }
  const uint16_t crc = vision_crc16(&output[FRAME_TYPE_INDEX],
                                    FRAME_CRC_INPUT_SIZE);
  output[FRAME_CRC_LOW_INDEX] = (uint8_t)crc;
  output[FRAME_CRC_HIGH_INDEX] = (uint8_t)(crc >> 8);
  output[FRAME_TAIL_INDEX] = VISION_FRAME_TAIL;
}

static void vision_save_config(const uint8_t *payload, uint8_t sequence)
{
  const uint8_t color = payload[0];
  const uint8_t zone = payload[1];
  const bool valid =
      ((color == VISION_COLOR_RED) || (color == VISION_COLOR_BLUE)) &&
      (zone >= VISION_ZONE_1) && (zone <= VISION_ZONE_4) &&
      vision_padding_zero(payload, 2U);

  if (latest_data.config_ready) {
    return;
  }
  if (!valid) {
    config_streak = 0U;
    config_sequence_valid = false;
    return;
  }

  if (config_sequence_valid &&
      (sequence == (uint8_t)(config_last_sequence + 1U)) &&
      (color == latest_data.color) && (zone == latest_data.start_zone)) {
    if (config_streak < APP_CONFIG_CONFIRM_FRAMES) {
      ++config_streak;
    }
  } else {
    latest_data.color = color;
    latest_data.start_zone = zone;
    config_streak = 1U;
  }
  config_last_sequence = sequence;
  latest_data.config_sequence = sequence;
  config_sequence_valid = true;
  latest_data.config_ready = config_streak >= APP_CONFIG_CONFIRM_FRAMES;
}

static void vision_save_report(const uint8_t *payload, uint8_t sequence,
                               uint32_t tick_ms)
{
  if ((latest_data.tick_ms != 0U) &&
      (sequence == latest_data.sequence)) {
    return;
  }

  const uint16_t x = vision_u16_be(&payload[0]);
  const uint16_t y = vision_u16_be(&payload[2]);
  const uint16_t distance = vision_u16_be(&payload[4]);
  const uint8_t flags = payload[7];
  const bool found = (flags & VISION_REPORT_FOUND) != 0U;
  const bool distance_valid =
      (flags & VISION_REPORT_DISTANCE_VALID) != 0U;

  if (((flags & 0xB4U) != 0U) ||
      (found && ((x > APP_VISION_MAX_X) || (y > APP_VISION_MAX_Y))) ||
      (distance_valid && (distance == 0U)) ||
      (!distance_valid && (distance != 0U)) ||
      (!found && !vision_padding_zero(payload, 0U))) {
    return;
  }

  latest_data.x = x;
  latest_data.y = y;
  latest_data.distance_mm = distance;
  latest_data.tick_ms = tick_ms;
  ++latest_data.report_generation;
  latest_data.sequence = sequence;
  latest_data.cargo_counts = payload[6];
  latest_data.found = found;
  latest_data.near = (flags & VISION_REPORT_NEAR) != 0U;
  latest_data.classification_valid =
      (flags & VISION_REPORT_CLASS_VALID) != 0U;
  latest_data.distance_valid = distance_valid;
  latest_data.valid = true;
}

static void vision_save_fused_pose(const uint8_t *payload, uint8_t sequence,
                                   uint32_t tick_ms)
{
  volatile VisionFusedPose *pose = &latest_data.fused_pose;
  if (pose->received && (pose->sequence == sequence)) {
    return;
  }
  const uint16_t heading = vision_u16_be(&payload[4]);
  if ((heading >= 36000U) || ((payload[6] & 0x80U) != 0U)) {
    return;
  }

  pose->x_mm = (int16_t)vision_u16_be(&payload[0]);
  pose->y_mm = (int16_t)vision_u16_be(&payload[2]);
  pose->heading_cdeg = heading;
  pose->status = payload[6];
  pose->tracker_confidence = payload[7] & 0x03U;
  pose->mapper_confidence = (payload[7] >> 2) & 0x03U;
  pose->position_sigma_cm = (payload[7] >> 4) & 0x0FU;
  pose->sequence = sequence;
  pose->tick_ms = tick_ms;
  pose->received = true;
}

static bool vision_mission_code_valid(uint8_t command)
{
  return (command == VISION_CMD_STOP) ||
         ((command >= VISION_CMD_GRAB_CONFIRMED) &&
          (command <= VISION_CMD_RETURN_CENTER));
}

static void vision_save_mission(const uint8_t *payload, uint8_t sequence,
                                uint32_t tick_ms)
{
  volatile VisionMissionCommand *command = &latest_data.mission;
  if (command->received && (command->sequence == sequence)) {
    return;
  }

  const uint8_t code = payload[0];
  const uint8_t flags = payload[1];
  const uint16_t heading = vision_u16_be(&payload[6]);
  if (!vision_mission_code_valid(code) ||
      ((flags & VISION_CMD_VALID) == 0U) ||
      ((flags & 0xE0U) != 0U) || (heading >= 36000U)) {
    return;
  }

  command->command = code;
  command->flags = flags;
  command->target_x_mm = (int16_t)vision_u16_be(&payload[2]);
  command->target_y_mm = (int16_t)vision_u16_be(&payload[4]);
  command->heading_cdeg = heading;
  command->sequence = sequence;
  command->tick_ms = tick_ms;
  command->received = true;
}

static void vision_save_frame(uint32_t tick_ms)
{
  const uint8_t type = frame[FRAME_TYPE_INDEX];
  const uint8_t sequence = frame[FRAME_SEQUENCE_INDEX];
  const uint8_t *payload = &frame[FRAME_PAYLOAD_INDEX];
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
    latest_data.last_frame[i] = frame[i];
  }
  latest_data.last_frame_tick_ms = tick_ms;
  latest_data.frame_received = true;

  if (type == VISION_MSG_CONFIG) {
    vision_save_config(payload, sequence);
  } else if (type == VISION_MSG_REPORT) {
    vision_save_report(payload, sequence, tick_ms);
  } else if (type == VISION_MSG_FUSED_POSE) {
    vision_save_fused_pose(payload, sequence, tick_ms);
  } else if (type == VISION_MSG_MISSION) {
    vision_save_mission(payload, sequence, tick_ms);
  }

  if (primask == 0U) {
    __enable_irq();
  }
}

static bool vision_frame_valid(void)
{
  const uint16_t received_crc =
      (uint16_t)frame[FRAME_CRC_LOW_INDEX] |
      ((uint16_t)frame[FRAME_CRC_HIGH_INDEX] << 8);
  const uint8_t type = frame[FRAME_TYPE_INDEX];
  return (frame[FRAME_TAIL_INDEX] == VISION_FRAME_TAIL) &&
         ((type == VISION_MSG_CONFIG) || (type == VISION_MSG_REPORT) ||
          (type == VISION_MSG_FUSED_POSE) ||
          (type == VISION_MSG_MISSION)) &&
         (vision_crc16(&frame[FRAME_TYPE_INDEX], FRAME_CRC_INPUT_SIZE) ==
          received_crc);
}

static void vision_resync_frame(void)
{
  uint8_t start = VISION_FRAME_SIZE;
  for (uint8_t i = 1U; i < (VISION_FRAME_SIZE - 1U); ++i) {
    if ((frame[i] == VISION_FRAME_HEAD_1) &&
        (frame[i + 1U] == VISION_FRAME_HEAD_2)) {
      start = i;
    }
  }
  if (start < VISION_FRAME_SIZE) {
    frame_index = (uint8_t)(VISION_FRAME_SIZE - start);
    for (uint8_t i = 0U; i < frame_index; ++i) {
      frame[i] = frame[start + i];
    }
  } else if (frame[FRAME_TAIL_INDEX] == VISION_FRAME_HEAD_1) {
    frame[0] = VISION_FRAME_HEAD_1;
    frame_index = 1U;
  } else {
    frame_index = 0U;
  }
}

static void vision_parse_byte(uint8_t value, uint32_t tick_ms)
{
  if (frame_index == 0U) {
    if (value == VISION_FRAME_HEAD_1) {
      frame[frame_index++] = value;
    }
    return;
  }
  if (frame_index == 1U) {
    if (value == VISION_FRAME_HEAD_2) {
      frame[frame_index++] = value;
    } else if (value != VISION_FRAME_HEAD_1) {
      frame_index = 0U;
    }
    return;
  }

  frame[frame_index++] = value;
  if (frame_index == VISION_FRAME_SIZE) {
    if (vision_frame_valid()) {
      vision_save_frame(tick_ms);
      frame_index = 0U;
    } else {
      vision_resync_frame();
    }
  }
}

void Vision_Init(void)
{
  latest_data = (VisionData){0};
  config_streak = 0U;
  config_last_sequence = 0U;
  config_sequence_valid = false;
  ack_remaining = 0U;
  status_pending = false;
  odom_pending = false;
  status_sequence = 0U;
  odom_sequence = 0U;
  Vision_ResetParser();
}

void Vision_ResetParser(void)
{
  frame_index = 0U;
}

void Vision_ParseBytes(const uint8_t *data, size_t size, uint32_t tick_ms)
{
  if (data == 0) {
    return;
  }
  for (size_t i = 0U; i < size; ++i) {
    vision_parse_byte(data[i], tick_ms);
  }
}

VisionData Vision_GetSnapshot(void)
{
  VisionData snapshot;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot = latest_data;
  if (primask == 0U) {
    __enable_irq();
  }
  return snapshot;
}

bool Vision_ReportIsFresh(const VisionData *data, uint32_t now_ms,
                          uint32_t timeout_ms)
{
  return (data != 0) && data->valid && (data->tick_ms != 0U) &&
         ((uint32_t)(now_ms - data->tick_ms) <= timeout_ms);
}

bool Vision_IsFresh(const VisionData *data, uint32_t now_ms,
                    uint32_t timeout_ms)
{
  return Vision_ReportIsFresh(data, now_ms, timeout_ms) && data->found;
}

bool Vision_FusedPoseIsFresh(const VisionFusedPose *pose, uint32_t now_ms,
                             uint32_t timeout_ms)
{
  return (pose != 0) && pose->received &&
         ((pose->status & VISION_POSE_VALID) != 0U) &&
         ((uint32_t)(now_ms - pose->tick_ms) <= timeout_ms);
}

bool Vision_MissionIsFresh(const VisionMissionCommand *command,
                           uint32_t now_ms, uint32_t timeout_ms)
{
  return (command != 0) && command->received &&
         ((command->flags & VISION_CMD_VALID) != 0U) &&
         ((uint32_t)(now_ms - command->tick_ms) <= timeout_ms);
}

void Vision_RequestConfigAck(void)
{
  ack_remaining = APP_CONFIG_CONFIRM_FRAMES;
}

void Vision_QueueStmStatus(const VisionStmStatus *status)
{
  if (status == 0) {
    return;
  }
  const uint8_t payload[VISION_PAYLOAD_SIZE] = {
    status->flags,
    status->mode,
    (uint8_t)(status->camera_pitch_cdeg >> 8),
    (uint8_t)status->camera_pitch_cdeg,
    status->acknowledged_sequence,
    status->fault_code,
    0U,
    0U
  };
  uint8_t pending[VISION_FRAME_SIZE];
  vision_build_frame(pending, VISION_MSG_STM_STATUS,
                     vision_next_sequence(&status_sequence), payload);

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
    status_frame[i] = pending[i];
  }
  status_pending = true;
  if (primask == 0U) {
    __enable_irq();
  }
}

void Vision_QueueOdom(const VisionOdom *odom)
{
  if ((odom == 0) || (odom->sample_period_ms == 0U)) {
    return;
  }
  const uint8_t payload[VISION_PAYLOAD_SIZE] = {
    (uint8_t)(odom->position[0] >> 8), (uint8_t)odom->position[0],
    (uint8_t)(odom->position[1] >> 8), (uint8_t)odom->position[1],
    (uint8_t)(odom->position[2] >> 8), (uint8_t)odom->position[2],
    odom->sample_period_ms, odom->status
  };
  uint8_t pending[VISION_FRAME_SIZE];
  vision_build_frame(pending, VISION_MSG_ODOM,
                     vision_next_sequence(&odom_sequence), payload);

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
    odom_frame[i] = pending[i];
  }
  odom_pending = true;
  if (primask == 0U) {
    __enable_irq();
  }
}

static void vision_send_pending(volatile bool *pending,
                                const uint8_t *source)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
    tx_frame[i] = source[i];
  }
  *pending = false;
  if (primask == 0U) {
    __enable_irq();
  }
  if (!Uart_Send(tx_frame, sizeof(tx_frame))) {
    *pending = true;
  }
}

void Vision_Process(void)
{
  if (ack_remaining > 0U) {
    if (Uart_Send(config_ack, sizeof(config_ack))) {
      --ack_remaining;
    }
  } else if (status_pending) {
    vision_send_pending(&status_pending, status_frame);
  } else if (odom_pending) {
    vision_send_pending(&odom_pending, odom_frame);
  }
}
