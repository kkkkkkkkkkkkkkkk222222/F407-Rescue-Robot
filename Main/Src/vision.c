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

static void vision_build_frame(uint8_t *output,
                               uint8_t type,
                               uint8_t sequence,
                               const uint8_t *payload)
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

static bool vision_config_valid(uint8_t color, uint8_t start_zone)
{
  return ((color == VISION_COLOR_RED) || (color == VISION_COLOR_BLUE)) &&
         (start_zone >= VISION_ZONE_1) && (start_zone <= VISION_ZONE_4);
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

static void vision_save_config(const uint8_t *payload, uint8_t sequence)
{
  const uint8_t color = payload[0];
  const uint8_t start_zone = payload[1];

  if (latest_data.config_ready) {
    return;
  }
  if (!vision_config_valid(color, start_zone) ||
      !vision_padding_zero(payload, 2U)) {
    config_streak = 0U;
    config_sequence_valid = false;
    return;
  }

  if (config_sequence_valid &&
      (sequence == (uint8_t)(config_last_sequence + 1U)) &&
      (color == latest_data.color) &&
      (start_zone == latest_data.start_zone)) {
    if (config_streak < APP_CONFIG_CONFIRM_FRAMES) {
      ++config_streak;
    }
  } else {
    latest_data.color = color;
    latest_data.start_zone = start_zone;
    config_streak = 1U;
  }
  config_last_sequence = sequence;
  latest_data.config_sequence = sequence;
  config_sequence_valid = true;
  latest_data.config_ready = config_streak >= APP_CONFIG_CONFIRM_FRAMES;
}

static void vision_save_report(const uint8_t *payload,
                               uint8_t sequence,
                               uint32_t tick_ms)
{
  if ((latest_data.tick_ms != 0U) &&
      (sequence == latest_data.sequence)) {
    return;
  }
  const uint16_t x = ((uint16_t)payload[0] << 8) | payload[1];
  const uint16_t y = ((uint16_t)payload[2] << 8) | payload[3];
  const uint16_t distance = ((uint16_t)payload[4] << 8) | payload[5];
  const uint8_t flags = payload[7];
  const bool found = (flags & VISION_REPORT_FOUND) != 0U;
  const bool grabbed = (flags & VISION_REPORT_GRABBED) != 0U;
  const bool near = (flags & VISION_REPORT_NEAR) != 0U;
  const bool unknown = (flags & VISION_REPORT_UNKNOWN) != 0U;
  const bool claw_view = (flags & VISION_REPORT_CLAW_VIEW) != 0U;
  const bool coordinates_valid = (x <= APP_VISION_MAX_X) &&
                                 (y <= APP_VISION_MAX_Y);

  if (((flags & 0xC0U) != 0U) ||
      (found && !coordinates_valid) ||
      (found && (distance < APP_VISION_MIN_DISTANCE_MM)) ||
      (!found && ((payload[6] != 0U) || near || grabbed || unknown))) {
    return;
  }

  latest_data.x = x;
  latest_data.y = y;
  latest_data.distance_mm = distance;
  latest_data.tick_ms = tick_ms;
  latest_data.sequence = sequence;
  latest_data.cargo_counts = payload[6];
  latest_data.found = found;
  latest_data.near = near;
  latest_data.grabbed = grabbed;
  latest_data.classification_valid =
      (flags & VISION_REPORT_CLASS_VALID) != 0U;
  latest_data.unknown = unknown;
  latest_data.claw_view = claw_view;
  latest_data.valid = true;
}

static void vision_save_event(const uint8_t *payload,
                              uint8_t sequence,
                              uint32_t tick_ms)
{
  if (!vision_padding_zero(payload, 1U)) {
    return;
  }
  if (payload[0] == VISION_EVENT_STOP) {
    latest_data.stop = true;
    latest_data.valid = false;
  } else if (payload[0] == VISION_EVENT_RESCUE) {
    latest_data.rescue_requested = true;
    latest_data.rescue_sequence = sequence;
    latest_data.rescue_tick_ms = tick_ms;
  }
}

static void vision_save_nav(const uint8_t *payload,
                            uint8_t sequence,
                            uint32_t tick_ms)
{
  if (latest_data.nav_valid &&
      (sequence == latest_data.nav_sequence)) {
    return;
  }
  const uint8_t direction = payload[0];
  const uint8_t zone_state = payload[1];
  const uint8_t destination = payload[2];
  const bool direction_valid = direction <= VISION_NAV_BACKWARD;
  const bool zone_valid = (zone_state == VISION_NAV_EN_ROUTE) ||
                          (zone_state == VISION_NAV_NEAR_SAFE);
  const bool destination_valid =
      (destination == VISION_DEST_MATERIAL) ||
      (destination == VISION_DEST_CASUALTY);

  if (!direction_valid || !zone_valid || !destination_valid ||
      !vision_padding_zero(payload, 3U) ||
      ((zone_state == VISION_NAV_NEAR_SAFE) &&
       (direction != VISION_NAV_HOLD))) {
    return;
  }

  latest_data.nav_sequence = sequence;
  latest_data.nav_direction = direction;
  latest_data.nav_zone_state = zone_state;
  latest_data.nav_destination = destination;
  latest_data.nav_tick_ms = tick_ms;
  latest_data.nav_valid = true;
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
  } else {
    if (!latest_data.config_ready) {
      config_streak = 0U;
      config_sequence_valid = false;
    }
    if (type == VISION_MSG_REPORT) {
      vision_save_report(payload, sequence, tick_ms);
    } else if (type == VISION_MSG_EVENT) {
      vision_save_event(payload, sequence, tick_ms);
    } else if (type == VISION_MSG_NAV) {
      vision_save_nav(payload, sequence, tick_ms);
    }
  }

  if (primask == 0U) {
    __enable_irq();
  }
}

static bool vision_frame_valid(void)
{
  const uint16_t expected_crc =
      (uint16_t)frame[FRAME_CRC_LOW_INDEX] |
      ((uint16_t)frame[FRAME_CRC_HIGH_INDEX] << 8);
  const uint8_t type = frame[FRAME_TYPE_INDEX];
  return (frame[FRAME_TAIL_INDEX] == VISION_FRAME_TAIL) &&
         ((type == VISION_MSG_CONFIG) ||
          (type == VISION_MSG_REPORT) ||
          (type == VISION_MSG_EVENT) ||
          (type == VISION_MSG_NAV)) &&
         (vision_crc16(&frame[FRAME_TYPE_INDEX], FRAME_CRC_INPUT_SIZE) ==
          expected_crc);
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
      frame[0] = value;
      frame_index = 1U;
    }
    return;
  }

  if (frame_index == 1U) {
    if (value == VISION_FRAME_HEAD_2) {
      frame[1] = value;
      frame_index = 2U;
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
      if (!latest_data.config_ready) {
        config_streak = 0U;
        config_sequence_valid = false;
      }
      vision_resync_frame();
    }
  }
}

void Vision_Init(void)
{
  latest_data.x = 0U;
  latest_data.y = 0U;
  latest_data.distance_mm = 0U;
  latest_data.tick_ms = 0U;
  latest_data.nav_tick_ms = 0U;
  latest_data.rescue_tick_ms = 0U;
  latest_data.last_frame_tick_ms = 0U;
  latest_data.color = 0U;
  latest_data.start_zone = 0U;
  latest_data.config_sequence = 0U;
  latest_data.sequence = 0U;
  latest_data.cargo_counts = 0U;
  latest_data.nav_sequence = 0U;
  latest_data.nav_direction = VISION_NAV_HOLD;
  latest_data.nav_zone_state = 0U;
  latest_data.nav_destination = 0U;
  latest_data.rescue_sequence = 0U;
  for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
    latest_data.last_frame[i] = 0U;
  }
  latest_data.valid = false;
  latest_data.nav_valid = false;
  latest_data.stop = false;
  latest_data.rescue_requested = false;
  latest_data.frame_received = false;
  latest_data.config_ready = false;
  latest_data.found = false;
  latest_data.grabbed = false;
  latest_data.near = false;
  latest_data.classification_valid = false;
  latest_data.unknown = false;
  latest_data.claw_view = false;
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
  if (!latest_data.config_ready) {
    config_streak = 0U;
    config_sequence_valid = false;
  }
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

bool Vision_IsFresh(const VisionData *data, uint32_t now_ms, uint32_t timeout_ms)
{
  return (data != 0) && data->valid && data->found &&
         (data->tick_ms != 0U) &&
         ((uint32_t)(now_ms - data->tick_ms) <= timeout_ms);
}

bool Vision_NavIsFresh(const VisionData *data,
                       uint32_t now_ms,
                       uint32_t timeout_ms)
{
  return (data != 0) && data->nav_valid &&
         (data->nav_tick_ms != 0U) &&
         ((uint32_t)(now_ms - data->nav_tick_ms) <= timeout_ms);
}

void Vision_RequestConfigAck(void)
{
  ack_remaining = APP_CONFIG_CONFIRM_FRAMES;
}

void Vision_QueueTaskStatus(const VisionTaskStatus *status)
{
  if (status == 0) {
    return;
  }

  const uint8_t payload[VISION_PAYLOAD_SIZE] = {
    status->state,
    status->destination,
    (uint8_t)(status->remaining_s >> 8),
    (uint8_t)status->remaining_s,
    status->flags,
    status->fault,
    status->recovery_count,
    status->cargo_counts
  };
  uint8_t pending[VISION_FRAME_SIZE];
  const uint8_t sequence = vision_next_sequence(&status_sequence);
  vision_build_frame(pending, VISION_MSG_STATUS, sequence, payload);

  const uint32_t copy_primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
    status_frame[i] = pending[i];
  }
  status_pending = true;
  if (copy_primask == 0U) {
    __enable_irq();
  }
}

void Vision_QueueOdom(const VisionOdom *odom)
{
  if ((odom == 0) || (odom->sample_period_ms == 0U)) {
    return;
  }

  const uint8_t payload[VISION_PAYLOAD_SIZE] = {
    (uint8_t)(odom->position[0] >> 8),
    (uint8_t)odom->position[0],
    (uint8_t)(odom->position[1] >> 8),
    (uint8_t)odom->position[1],
    (uint8_t)(odom->position[2] >> 8),
    (uint8_t)odom->position[2],
    odom->sample_period_ms,
    odom->status
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

void Vision_Process(void)
{
  if (ack_remaining > 0U) {
    if (Uart_Send(config_ack, sizeof(config_ack))) {
      --ack_remaining;
    }
  } else if (odom_pending) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
      tx_frame[i] = odom_frame[i];
    }
    odom_pending = false;
    if (primask == 0U) {
      __enable_irq();
    }
    if (!Uart_Send(tx_frame, sizeof(tx_frame))) {
      odom_pending = true;
    }
  } else if (status_pending) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < VISION_FRAME_SIZE; ++i) {
      tx_frame[i] = status_frame[i];
    }
    status_pending = false;
    if (primask == 0U) {
      __enable_irq();
    }
    if (!Uart_Send(tx_frame, sizeof(tx_frame))) {
      status_pending = true;
    }
  }
}
