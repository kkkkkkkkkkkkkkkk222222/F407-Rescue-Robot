#include "Debug.h"

#include <math.h>

#include "app_config.h"
#include "imu.h"
#include "Location.h"
#include "main.h"
#include "motor.h"
#include "vision.h"

#if APP_ENABLE_MOTION_DEBUG_TASK && \
    ((VISION_MSG_MOTION_COMMAND != 0x19U) || \
     (VISION_MSG_MOTION_STATUS != 0x1AU))
#error "Mapping debug must use shijue_fangan TYPE=0x19/0x1A"
#endif

#define DEBUG_DEG_RAD 0.01745329252f

static volatile DebugStatus debug_status;
static bool initialized;
static bool active;
static bool command_sequence_valid;
static uint8_t last_command_sequence;
static uint8_t active_command;
static float active_turn_angle_deg;
static float active_turn_speed_mm_s;
static int64_t active_turn_start_yaw_mdeg;
static float active_move_start_x_mm;
static float active_move_start_y_mm;
static float active_move_field_angle_rad;
static float active_move_distance_mm;
static float active_move_speed_mm_s;
static float active_move_heading_deg;
static float active_move_progress_sign;
static uint32_t active_start_ms;
static uint32_t active_timeout_ms;
static uint32_t last_status_ms;

static uint32_t debug_enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void debug_leave_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static float debug_wrap_degrees(float angle_deg)
{
  angle_deg = fmodf(angle_deg, 360.0f);
  if (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static float debug_wrap_error(float angle_deg)
{
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static uint16_t debug_saturate_u16(float value)
{
  if (!(value > 0.0f)) {
    return 0U;
  }
  if (value >= 65535.0f) {
    return UINT16_MAX;
  }
  return (uint16_t)(value + 0.5f);
}

static uint16_t debug_heading_cdeg(void)
{
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    return 0U;
  }
  return (uint16_t)((uint32_t)pose.heading_mdeg / 10U);
}

static bool debug_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static uint8_t debug_health_flags(void)
{
  const IMUData imu = IMU_GetData();
  const LocationPose pose = Location_GetPose();
  uint8_t flags = 0U;
  if (imu.ready) {
    flags |= VISION_MOTION_STATUS_IMU_READY;
  }
  if (pose.valid) {
    flags |= VISION_MOTION_STATUS_ODOM_VALID;
  }
  if (debug_motor_fault()) {
    flags |= VISION_MOTION_STATUS_MOTOR_FAULT;
  }
  return flags;
}

static void debug_publish_status(uint32_t now_ms, bool force)
{
  debug_status.flags = debug_health_flags();
  if (!force &&
      ((uint32_t)(now_ms - last_status_ms) <
       APP_MOTION_DEBUG_STATUS_PERIOD_MS)) {
    return;
  }

  const VisionMotionStatus status = {
    .command_sequence = debug_status.command_sequence,
    .state = (uint8_t)debug_status.state,
    .fault = (uint8_t)debug_status.fault,
    .command = debug_status.command,
    .progress = debug_status.progress,
    .heading_cdeg = debug_heading_cdeg()
  };
  Vision_QueueMotionStatus(&status);
  last_status_ms = now_ms;
}

static void debug_finish(DebugMotionState state, DebugMotionFault fault,
                         uint32_t now_ms)
{
  Motor_Stop();
  active = false;
  debug_status.state = state;
  debug_status.fault = fault;
  debug_publish_status(now_ms, true);
}

static uint32_t debug_move_timeout(float distance_mm, float speed_mm_s)
{
  const float travel_ms = distance_mm * 1000.0f / speed_mm_s;
  float timeout_ms = travel_ms * 4.0f +
                     (float)APP_MOTION_DEBUG_MOVE_TIMEOUT_MIN_MS;
  if (timeout_ms > (float)APP_MOTION_DEBUG_MOVE_TIMEOUT_MAX_MS) {
    timeout_ms = (float)APP_MOTION_DEBUG_MOVE_TIMEOUT_MAX_MS;
  }
  if (timeout_ms < (float)APP_MOTION_DEBUG_MOVE_TIMEOUT_MIN_MS) {
    timeout_ms = (float)APP_MOTION_DEBUG_MOVE_TIMEOUT_MIN_MS;
  }
  return (uint32_t)(timeout_ms + 0.5f);
}

static void debug_start_command(const VisionData *vision, uint32_t now_ms)
{
  active = false;
  Motor_Stop();
  debug_status.command = vision->motion_opcode;
  debug_status.command_sequence = vision->motion_sequence;
  debug_status.progress = 0U;
  debug_status.remaining = 0U;
  debug_status.fault = DEBUG_MOTION_FAULT_NONE;
  active_command = vision->motion_opcode;
  active_start_ms = now_ms;

  if (active_command == VISION_MOTION_CMD_STOP) {
    debug_status.state = DEBUG_MOTION_STOPPED;
    debug_publish_status(now_ms, true);
    return;
  }
  if (active_command == VISION_MOTION_CMD_HOLD) {
    debug_status.state = DEBUG_MOTION_DONE;
    debug_publish_status(now_ms, true);
    return;
  }
  if (active_command == VISION_MOTION_CMD_RESET_ODOM) {
    Location_ResetReference(0, 0, 0);
    debug_status.state = DEBUG_MOTION_DONE;
    debug_publish_status(now_ms, true);
    return;
  }

  const IMUData imu = IMU_GetData();
  if (!imu.ready) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_IMU, now_ms);
    return;
  }

  if (active_command == VISION_MOTION_CMD_TURN_REL) {
    active_turn_angle_deg = (float)vision->motion_arg1 * 0.1f;
    const float turn_rate_deg_s = (vision->motion_speed == 0U) ?
        90.0f : (float)vision->motion_speed * 0.1f;
    active_turn_speed_mm_s = turn_rate_deg_s * DEBUG_DEG_RAD *
                             APP_MOTION_DEBUG_TURN_RADIUS_MM;
    active_turn_start_yaw_mdeg = imu.yaw_mdeg;
    active_timeout_ms = APP_MOTOR_TURN_TIMEOUT_MS;
    debug_status.remaining = (uint16_t)((vision->motion_arg1 < 0) ?
        -vision->motion_arg1 : vision->motion_arg1);
    active = true;
    debug_status.state = DEBUG_MOTION_RUNNING;
    debug_publish_status(now_ms, true);
    return;
  }

  if ((active_command == VISION_MOTION_CMD_MOVE_BODY) ||
      (active_command == VISION_MOTION_CMD_MOVE_FIELD)) {
    const LocationPose pose = Location_GetPose();
    if (!pose.valid) {
      debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_ODOM, now_ms);
      return;
    }

    const float arg1_mm = (float)vision->motion_arg1;
    const float arg2_mm = (float)vision->motion_arg2;
    const float direction_deg = atan2f(arg2_mm, arg1_mm) *
                                57.2957795f;
    const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
    const bool field_frame = active_command == VISION_MOTION_CMD_MOVE_FIELD;
    const float field_direction_deg = field_frame ?
        debug_wrap_degrees(direction_deg) :
        debug_wrap_degrees(current_heading_deg + direction_deg);
    active_move_start_x_mm = (float)pose.x_mm;
    active_move_start_y_mm = (float)pose.y_mm;
    active_move_field_angle_rad = field_direction_deg * DEBUG_DEG_RAD;
    active_move_distance_mm = sqrtf(arg1_mm * arg1_mm + arg2_mm * arg2_mm);
    active_move_speed_mm_s = (vision->motion_speed == 0U) ?
        APP_GO_DISTANCE_SPEED_MM_S : (float)vision->motion_speed;
    active_move_heading_deg = current_heading_deg;
    active_move_progress_sign =
        ((arg1_mm < 0.0f) || ((arg1_mm == 0.0f) && (arg2_mm < 0.0f))) ?
        -1.0f : 1.0f;
    active_timeout_ms = debug_move_timeout(active_move_distance_mm,
                                            active_move_speed_mm_s);
    debug_status.remaining = debug_saturate_u16(active_move_distance_mm);
    active = true;
    debug_status.state = DEBUG_MOTION_RUNNING;
    debug_publish_status(now_ms, true);
    return;
  }

  debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_INVALID, now_ms);
}

static void debug_update_turn(uint32_t now_ms)
{
  const IMUData imu = IMU_GetData();
  if (!imu.ready) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_IMU, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_TIMEOUT, now_ms);
    return;
  }

  const int64_t raw_delta_mdeg = imu.yaw_mdeg - active_turn_start_yaw_mdeg;
  const int64_t field_delta_mdeg = (APP_LOCATION_IMU_YAW_SIGN < 0.0f) ?
      -raw_delta_mdeg : raw_delta_mdeg;
  const float directed_delta_deg = (float)field_delta_mdeg * 0.001f *
      ((active_turn_angle_deg >= 0.0f) ? 1.0f : -1.0f);
  float progress_deg = (directed_delta_deg > 0.0f) ? directed_delta_deg : 0.0f;
  const float target_deg = (active_turn_angle_deg >= 0.0f) ?
      active_turn_angle_deg : -active_turn_angle_deg;
  if (progress_deg > target_deg) {
    progress_deg = target_deg;
  }
  const float turn_sign = (active_turn_angle_deg >= 0.0f) ? 1.0f : -1.0f;
  debug_status.progress = (int16_t)(turn_sign * progress_deg * 10.0f);
  debug_status.remaining = debug_saturate_u16(
      (target_deg - progress_deg) * 10.0f);

  const float remaining_deg = target_deg - progress_deg;
  if (remaining_deg * 1000.0f <= APP_MOTOR_TURN_TOLERANCE_MDEG) {
    debug_status.progress = (int16_t)(turn_sign * target_deg * 10.0f);
    debug_status.remaining = 0U;
    debug_finish(DEBUG_MOTION_DONE, DEBUG_MOTION_FAULT_NONE, now_ms);
    return;
  }

  const float speed =
      (remaining_deg * 1000.0f <= APP_MOTOR_TURN_SLOWDOWN_MDEG) ?
      fminf(active_turn_speed_mm_s, APP_MOTOR_TURN_SLOW_MM_S) :
      active_turn_speed_mm_s;
  Motor_Move(0.0f, 0.0f,
             speed * ((active_turn_angle_deg >= 0.0f) ? 1.0f : -1.0f));
}

static void debug_update_move(uint32_t now_ms)
{
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_ODOM, now_ms);
    return;
  }
  if (debug_motor_fault()) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_MOTOR, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_TIMEOUT, now_ms);
    return;
  }

  const float cosine = cosf(active_move_field_angle_rad);
  const float sine = sinf(active_move_field_angle_rad);
  const float delta_x = (float)pose.x_mm - active_move_start_x_mm;
  const float delta_y = (float)pose.y_mm - active_move_start_y_mm;
  const float progress_mm = delta_x * cosine + delta_y * sine;
  const float cross_track_mm = -delta_x * sine + delta_y * cosine;
  float bounded_progress_mm = (progress_mm > 0.0f) ? progress_mm : 0.0f;
  if (bounded_progress_mm > active_move_distance_mm) {
    bounded_progress_mm = active_move_distance_mm;
  }
  const float remaining_mm = active_move_distance_mm - bounded_progress_mm;
  debug_status.progress = (int16_t)(active_move_progress_sign *
                                    bounded_progress_mm);
  debug_status.remaining = debug_saturate_u16(remaining_mm);

  if (remaining_mm <= APP_MOTION_DEBUG_DISTANCE_TOLERANCE_MM) {
    debug_status.progress = debug_saturate_u16(active_move_distance_mm);
    debug_status.remaining = 0U;
    debug_finish(DEBUG_MOTION_DONE, DEBUG_MOTION_FAULT_NONE, now_ms);
    return;
  }

  float speed_mm_s = active_move_speed_mm_s;
  if (remaining_mm < APP_MOTION_DEBUG_SLOWDOWN_MM) {
    float ratio = (remaining_mm - APP_MOTION_DEBUG_DISTANCE_TOLERANCE_MM) /
        (APP_MOTION_DEBUG_SLOWDOWN_MM -
         APP_MOTION_DEBUG_DISTANCE_TOLERANCE_MM);
    if (ratio < 0.0f) {
      ratio = 0.0f;
    } else if (ratio > 1.0f) {
      ratio = 1.0f;
    }
    const float minimum_speed =
        (active_move_speed_mm_s < APP_MOTION_DEBUG_MIN_SLOW_SPEED_MM_S) ?
        active_move_speed_mm_s : APP_MOTION_DEBUG_MIN_SLOW_SPEED_MM_S;
    speed_mm_s = minimum_speed +
        (active_move_speed_mm_s - minimum_speed) * ratio;
  }

  float correction_mm_s = -APP_MOTION_DEBUG_CROSS_TRACK_KP * cross_track_mm;
  if (correction_mm_s > APP_MOTION_DEBUG_CROSS_TRACK_LIMIT_MM_S) {
    correction_mm_s = APP_MOTION_DEBUG_CROSS_TRACK_LIMIT_MM_S;
  } else if (correction_mm_s < -APP_MOTION_DEBUG_CROSS_TRACK_LIMIT_MM_S) {
    correction_mm_s = -APP_MOTION_DEBUG_CROSS_TRACK_LIMIT_MM_S;
  }
  const float field_x_mm_s = speed_mm_s * cosine - correction_mm_s * sine;
  const float field_y_mm_s = speed_mm_s * sine + correction_mm_s * cosine;
  const float heading_rad = (float)pose.heading_mdeg * 0.001f * DEBUG_DEG_RAD;
  const float heading_cosine = cosf(heading_rad);
  const float heading_sine = sinf(heading_rad);
  const float body_forward_mm_s = heading_cosine * field_x_mm_s +
                                  heading_sine * field_y_mm_s;
  const float physical_left_mm_s = -heading_sine * field_x_mm_s +
                                   heading_cosine * field_y_mm_s;
  float yaw_mm_s = debug_wrap_error(
      active_move_heading_deg - (float)pose.heading_mdeg * 0.001f) *
      APP_MOTOR_HEADING_KP;
  if (yaw_mm_s > APP_MOTOR_HEADING_LIMIT_MM_S) {
    yaw_mm_s = APP_MOTOR_HEADING_LIMIT_MM_S;
  } else if (yaw_mm_s < -APP_MOTOR_HEADING_LIMIT_MM_S) {
    yaw_mm_s = -APP_MOTOR_HEADING_LIMIT_MM_S;
  }
  Motor_Move(body_forward_mm_s,
             physical_left_mm_s * APP_OMNI_LATERAL_API_SIGN,
             yaw_mm_s);
}

void Debug_Init(uint32_t now_ms)
{
  const uint32_t primask = debug_enter_critical();
  debug_status.state = DEBUG_MOTION_IDLE;
  debug_status.command = VISION_MOTION_CMD_STOP;
  debug_status.command_sequence = 0U;
  debug_status.progress = 0U;
  debug_status.remaining = 0U;
  debug_status.flags = 0U;
  debug_status.fault = DEBUG_MOTION_FAULT_NONE;
  debug_leave_critical(primask);

  initialized = true;
  active = false;
  command_sequence_valid = false;
  last_command_sequence = 0U;
  active_command = VISION_MOTION_CMD_STOP;
  active_turn_angle_deg = 0.0f;
  active_turn_speed_mm_s = 0.0f;
  active_turn_start_yaw_mdeg = 0LL;
  active_move_start_x_mm = 0.0f;
  active_move_start_y_mm = 0.0f;
  active_move_field_angle_rad = 0.0f;
  active_move_distance_mm = 0.0f;
  active_move_speed_mm_s = 0.0f;
  active_move_heading_deg = 0.0f;
  active_move_progress_sign = 1.0f;
  active_start_ms = now_ms;
  active_timeout_ms = 0U;
  last_status_ms = now_ms;

  /* Mapping starts at the actual three-wheel rotation centre. */
  Location_ResetReference(0, 0, 0);
  debug_publish_status(now_ms, true);
}

void Debug_Process(uint32_t now_ms)
{
  if (!initialized) {
    Debug_Init(now_ms);
  }

  const VisionData vision = Vision_GetSnapshot();
  if (vision.motion_valid &&
      (!command_sequence_valid ||
       (vision.motion_sequence != last_command_sequence))) {
    command_sequence_valid = true;
    last_command_sequence = vision.motion_sequence;
    debug_start_command(&vision, now_ms);
  }

  if (active) {
    if (debug_motor_fault()) {
      debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_MOTOR, now_ms);
    } else if (active_command == VISION_MOTION_CMD_TURN_REL) {
      debug_update_turn(now_ms);
    } else if ((active_command == VISION_MOTION_CMD_MOVE_BODY) ||
               (active_command == VISION_MOTION_CMD_MOVE_FIELD)) {
      debug_update_move(now_ms);
    } else {
      debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_INVALID, now_ms);
    }
  }

  debug_publish_status(now_ms, false);
}

DebugStatus Debug_GetStatus(void)
{
  const uint32_t primask = debug_enter_critical();
  const DebugStatus status = debug_status;
  debug_leave_critical(primask);
  return status;
}
