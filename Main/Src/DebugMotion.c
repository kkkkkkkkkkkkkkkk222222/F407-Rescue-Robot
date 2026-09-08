#include "DebugMotion.h"

#include <math.h>

#include "app_config.h"
#include "Location.h"
#include "motor.h"
#include "imu.h"
#include "main.h"
#include "vision.h"

#define DEBUG_MOTION_DEG_RAD 0.01745329252f

static volatile DebugMotionStatus motion_status;
static bool initialized;
static bool active;
static bool command_sequence_valid;
static uint8_t last_command_sequence;
static uint8_t active_command;
static uint8_t active_flags;
static float active_turn_angle_deg;
static float active_turn_speed_mm_s;
static int64_t active_turn_start_yaw_mdeg;
static float active_move_start_x_mm;
static float active_move_start_y_mm;
static float active_move_field_angle_rad;
static float active_move_distance_mm;
static float active_move_speed_mm_s;
static uint32_t active_start_ms;
static uint32_t active_timeout_ms;
static uint32_t last_status_ms;

static uint32_t motion_enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void motion_leave_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static float motion_wrap_degrees(float angle_deg)
{
  angle_deg = fmodf(angle_deg, 360.0f);
  if (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static uint16_t motion_saturate_u16(float value)
{
  if (!(value > 0.0f)) {
    return 0U;
  }
  if (value >= 65535.0f) {
    return UINT16_MAX;
  }
  return (uint16_t)(value + 0.5f);
}

static bool motion_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static uint8_t motion_health_flags(void)
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
  if (motion_motor_fault()) {
    flags |= VISION_MOTION_STATUS_MOTOR_FAULT;
  }
  return flags;
}

static void motion_publish_status(uint32_t now_ms, bool force)
{
  motion_status.flags = motion_health_flags();
  if (!force &&
      ((uint32_t)(now_ms - last_status_ms) <
       APP_MOTION_DEBUG_STATUS_PERIOD_MS)) {
    return;
  }

  const VisionMotionStatus status = {
    .state = (uint8_t)motion_status.state,
    .command = motion_status.command,
    .progress = motion_status.progress,
    .remaining = motion_status.remaining,
    .flags = motion_status.flags,
    .fault = (uint8_t)motion_status.fault,
    .command_sequence = motion_status.command_sequence
  };
  Vision_QueueMotionStatus(&status);
  last_status_ms = now_ms;
}

static void motion_finish(DebugMotionState state, DebugMotionFault fault,
                          uint32_t now_ms)
{
  Motor_Stop();
  active = false;
  motion_status.state = state;
  motion_status.fault = fault;
  motion_publish_status(now_ms, true);
}

static uint32_t motion_move_timeout(float distance_mm, float speed_mm_s)
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

static void motion_start_command(const VisionData *vision, uint32_t now_ms)
{
  active = false;
  Motor_Stop();
  motion_status.command = vision->motion_opcode;
  motion_status.command_sequence = vision->motion_sequence;
  motion_status.progress = 0U;
  motion_status.remaining = 0U;
  motion_status.fault = DEBUG_MOTION_FAULT_NONE;
  active_command = vision->motion_opcode;
  active_flags = vision->motion_flags;
  active_start_ms = now_ms;

  if (active_command == VISION_MOTION_CMD_STOP) {
    motion_status.state = DEBUG_MOTION_STOPPED;
    motion_publish_status(now_ms, true);
    return;
  }

  const IMUData imu = IMU_GetData();
  if (!imu.ready) {
    motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_IMU, now_ms);
    return;
  }

  if (active_command == VISION_MOTION_CMD_TURN_REL) {
    const float magnitude_deg = (float)vision->motion_param_a * 0.01f;
    active_turn_angle_deg =
        ((active_flags & VISION_MOTION_TURN_NEGATIVE) != 0U) ?
        -magnitude_deg : magnitude_deg;
    active_turn_speed_mm_s = (vision->motion_param_b == 0U) ?
        APP_MOTOR_TURN_FAST_MM_S : (float)vision->motion_param_b;
    active_turn_start_yaw_mdeg = imu.yaw_mdeg;
    active_timeout_ms = APP_MOTOR_TURN_TIMEOUT_MS;
    motion_status.remaining = vision->motion_param_a;
    active = true;
    motion_status.state = DEBUG_MOTION_RUNNING;
    motion_publish_status(now_ms, true);
    return;
  }

  if (active_command == VISION_MOTION_CMD_MOVE_DISTANCE) {
    const LocationPose pose = Location_GetPose();
    if (!pose.valid) {
      motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_ODOM, now_ms);
      return;
    }

    const float direction_deg = (float)vision->motion_param_a * 0.01f;
    const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
    const bool field_frame =
        (active_flags & VISION_MOTION_MOVE_FIELD_FRAME) != 0U;
    const float field_direction_deg = field_frame ? direction_deg :
        motion_wrap_degrees(current_heading_deg + direction_deg);
    active_move_start_x_mm = (float)pose.x_mm;
    active_move_start_y_mm = (float)pose.y_mm;
    active_move_field_angle_rad = field_direction_deg * DEBUG_MOTION_DEG_RAD;
    active_move_distance_mm = (float)vision->motion_param_b;
    active_move_speed_mm_s = (vision->motion_param_c == 0U) ?
        APP_GO_DISTANCE_SPEED_MM_S : (float)vision->motion_param_c;
    active_timeout_ms = motion_move_timeout(active_move_distance_mm,
                                             active_move_speed_mm_s);
    motion_status.remaining = vision->motion_param_b;
    active = true;
    motion_status.state = DEBUG_MOTION_RUNNING;
    motion_publish_status(now_ms, true);
    return;
  }

  motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_INVALID, now_ms);
}

static void motion_update_turn(uint32_t now_ms)
{
  const IMUData imu = IMU_GetData();
  if (!imu.ready) {
    motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_IMU, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) {
    motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_TIMEOUT, now_ms);
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
  motion_status.progress = motion_saturate_u16(progress_deg * 100.0f);
  motion_status.remaining = motion_saturate_u16(
      (target_deg - progress_deg) * 100.0f);

  const float remaining_deg = target_deg - progress_deg;
  if (remaining_deg * 1000.0f <= APP_MOTOR_TURN_TOLERANCE_MDEG) {
    motion_status.progress = motion_saturate_u16(target_deg * 100.0f);
    motion_status.remaining = 0U;
    motion_finish(DEBUG_MOTION_DONE, DEBUG_MOTION_FAULT_NONE, now_ms);
  } else {
    const float speed = remaining_deg * 1000.0f <= APP_MOTOR_TURN_SLOWDOWN_MDEG ?
        fminf(active_turn_speed_mm_s, APP_MOTOR_TURN_SLOW_MM_S) : active_turn_speed_mm_s;
    Motor_Move(0.0f, 0.0f, speed * ((active_turn_angle_deg >= 0.0f) ? 1.0f : -1.0f));
  }
}

static void motion_update_move(uint32_t now_ms)
{
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_ODOM, now_ms);
    return;
  }
  if (motion_motor_fault()) {
    motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_MOTOR, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) {
    motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_TIMEOUT, now_ms);
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
  float remaining_mm = active_move_distance_mm - bounded_progress_mm;
  motion_status.progress = motion_saturate_u16(bounded_progress_mm);
  motion_status.remaining = motion_saturate_u16(remaining_mm);

  if (remaining_mm <= APP_MOTION_DEBUG_DISTANCE_TOLERANCE_MM) {
    motion_status.progress = motion_saturate_u16(active_move_distance_mm);
    motion_status.remaining = 0U;
    motion_finish(DEBUG_MOTION_DONE, DEBUG_MOTION_FAULT_NONE, now_ms);
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
    const float minimum_speed = (active_move_speed_mm_s <
                                 APP_MOTION_DEBUG_MIN_SLOW_SPEED_MM_S) ?
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
  const float heading_rad = (float)pose.heading_mdeg * 0.001f *
                            DEBUG_MOTION_DEG_RAD;
  const float heading_cosine = cosf(heading_rad);
  const float heading_sine = sinf(heading_rad);
  const float body_forward_mm_s = heading_cosine * field_x_mm_s +
                                  heading_sine * field_y_mm_s;
  const float physical_left_mm_s = -heading_sine * field_x_mm_s +
                                   heading_cosine * field_y_mm_s;
  Motor_Move(body_forward_mm_s,
             physical_left_mm_s * APP_OMNI_LATERAL_API_SIGN,
             0.0f);
}

void DebugMotionTask_Init(uint32_t now_ms)
{
  const uint32_t primask = motion_enter_critical();
  motion_status.state = DEBUG_MOTION_IDLE;
  motion_status.command = VISION_MOTION_CMD_STOP;
  motion_status.command_sequence = 0U;
  motion_status.progress = 0U;
  motion_status.remaining = 0U;
  motion_status.flags = 0U;
  motion_status.fault = DEBUG_MOTION_FAULT_NONE;
  motion_leave_critical(primask);

  initialized = true;
  active = false;
  command_sequence_valid = false;
  last_command_sequence = 0U;
  active_command = VISION_MOTION_CMD_STOP;
  active_flags = 0U;
  active_turn_angle_deg = 0.0f;
  active_turn_speed_mm_s = 0.0f;
  active_turn_start_yaw_mdeg = 0LL;
  active_move_start_x_mm = 0.0f;
  active_move_start_y_mm = 0.0f;
  active_move_field_angle_rad = 0.0f;
  active_move_distance_mm = 0.0f;
  active_move_speed_mm_s = 0.0f;
  active_start_ms = now_ms;
  active_timeout_ms = 0U;
  last_status_ms = now_ms;

  /* Debug motion starts a fresh local field frame. The first pose is the
   * robot centre, not the T265 tracking origin. */
  Location_ResetReference(0, 0, 0);
  motion_publish_status(now_ms, true);
}

void DebugMotionTask_Process(uint32_t now_ms)
{
  if (!initialized) {
    DebugMotionTask_Init(now_ms);
  }

  const VisionData vision = Vision_GetSnapshot();
  if (vision.mission.received &&
      (vision.mission.command == VISION_CMD_ABORT)) {
    /* Latest upstream mission ABORT is also honored in debug mode. */
    if ((motion_status.state != DEBUG_MOTION_STOPPED) || active) {
      Motor_Stop();
      active = false;
      motion_status.command = VISION_MOTION_CMD_STOP;
      motion_status.progress = 0U;
      motion_status.remaining = 0U;
      motion_status.state = DEBUG_MOTION_STOPPED;
      motion_status.fault = DEBUG_MOTION_FAULT_NONE;
      motion_publish_status(now_ms, true);
    }
    motion_publish_status(now_ms, false);
    return;
  }
  if (vision.motion_valid &&
      (!command_sequence_valid ||
       (vision.motion_sequence != last_command_sequence))) {
    command_sequence_valid = true;
    last_command_sequence = vision.motion_sequence;
    motion_start_command(&vision, now_ms);
  }

  if (active) {
    if (motion_motor_fault()) {
      motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_MOTOR, now_ms);
    } else if (active_command == VISION_MOTION_CMD_TURN_REL) {
      motion_update_turn(now_ms);
    } else if (active_command == VISION_MOTION_CMD_MOVE_DISTANCE) {
      motion_update_move(now_ms);
    } else {
      motion_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_INVALID, now_ms);
    }
  }

  motion_publish_status(now_ms, false);
}

DebugMotionStatus DebugMotionTask_GetStatus(void)
{
  const uint32_t primask = motion_enter_critical();
  const DebugMotionStatus status = motion_status;
  motion_leave_critical(primask);
  return status;
}
