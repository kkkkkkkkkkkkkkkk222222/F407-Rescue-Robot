#include "Debug.h"

#include <math.h>

#include "app_config.h"
#include "imu.h"
#include "Location.h"
#include "main.h"
#include "motor.h"
#include "vision.h"
#include "WheelCheck.h"

#if APP_ENABLE_MOTION_DEBUG_TASK && \
    ((VISION_MSG_MOTION_COMMAND != 0x19U) || \
     (VISION_MSG_MOTION_STATUS != 0x1AU))
#error "Mapping debug must use shijue_fangan TYPE=0x19/0x1A"
#endif

#define DEBUG_DEG_RAD 0.01745329252f

static volatile DebugStatus debug_status;
static bool initialized;
static bool active;
static volatile bool wheel_check_active;
static bool console_owner;
static volatile bool console_pending;
static uint8_t console_sequence;
static struct {
  uint8_t opcode;
  uint8_t sequence;
  int16_t arg1;
  int16_t arg2;
  uint16_t speed;
} console_request;
static bool command_sequence_valid;
static uint8_t last_command_sequence;
static uint8_t active_command;
static float active_turn_angle_deg;
static float active_turn_speed_mm_s;
static float active_move_start_x_mm;
static float active_move_start_y_mm;
static float active_move_field_angle_rad;
static float active_move_distance_mm;
static float active_move_speed_mm_s;
static float active_move_body_angle_deg;
static float active_move_progress_sign;
static bool active_move_body_straight;
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

static void debug_start_command(const VisionData *vision, uint32_t now_ms,
                                DebugCommandSource source)
{
  active = false;
  active_move_body_straight = false;
  Motor_Stop();
  debug_status.command = vision->motion_opcode;
  debug_status.command_sequence = vision->motion_sequence;
  debug_status.progress = 0U;
  debug_status.remaining = 0U;
  debug_status.fault = DEBUG_MOTION_FAULT_NONE;
  debug_status.source = source;
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
    active_turn_speed_mm_s = (vision->motion_speed == 0U) ?
        APP_MOTOR_TURN_FAST_MM_S : (float)vision->motion_speed * 0.1f *
        DEBUG_DEG_RAD * APP_MOTION_DEBUG_TURN_RADIUS_MM;
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
    active_move_body_straight = !field_frame && (vision->motion_arg2 == 0) &&
                               (vision->motion_arg1 != 0);
    const float field_direction_deg = field_frame ?
        debug_wrap_degrees(direction_deg) :
        debug_wrap_degrees(current_heading_deg + direction_deg);
    active_move_start_x_mm = (float)pose.x_mm;
    active_move_start_y_mm = (float)pose.y_mm;
    active_move_field_angle_rad = field_direction_deg * DEBUG_DEG_RAD;
    active_move_distance_mm = sqrtf(arg1_mm * arg1_mm + arg2_mm * arg2_mm);
    active_move_speed_mm_s = (vision->motion_speed == 0U) ?
        (active_move_body_straight ? APP_MOTION_DEBUG_STRAIGHT_SPEED_MM_S :
                                    APP_GO_DISTANCE_SPEED_MM_S) :
        (float)vision->motion_speed;
    active_move_body_angle_deg = direction_deg;
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
  /* Poll the same latched controller as NormalRun. Motor_Update owns
   * completion, slowdown and braking even when Debug_Process is delayed. */
  const MotorTurnStatus result = Motor_TurnAngleSpeed(active_turn_angle_deg,
                                                     active_turn_speed_mm_s);
  float progress_deg = Motor_GetTurnTravelledDeg();
  const float target_deg = (active_turn_angle_deg >= 0.0f) ?
      active_turn_angle_deg : -active_turn_angle_deg;
  if (progress_deg > target_deg) {
    progress_deg = target_deg;
  }
  const float turn_sign = (active_turn_angle_deg >= 0.0f) ? 1.0f : -1.0f;
  debug_status.progress = (int16_t)(turn_sign * progress_deg * 10.0f);
  debug_status.remaining = debug_saturate_u16(
      (target_deg - progress_deg) * 10.0f);

  if (result == MOTOR_TURN_DONE) {
    debug_status.progress = (int16_t)(turn_sign * target_deg * 10.0f);
    debug_status.remaining = 0U;
    debug_finish(DEBUG_MOTION_DONE, DEBUG_MOTION_FAULT_NONE, now_ms);
    return;
  }

  if (result == MOTOR_TURN_FAULT) {
    const DebugMotionFault fault =
        ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) ?
        DEBUG_MOTION_FAULT_TIMEOUT : DEBUG_MOTION_FAULT_MOTOR;
    debug_finish(DEBUG_MOTION_FAULT, fault, now_ms);
  } else if (result != MOTOR_TURN_RUNNING) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_INVALID, now_ms);
  } else if ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_TIMEOUT, now_ms);
  }
}

static void debug_update_straight(uint32_t now_ms)
{
  const MotorDistanceStatus result = Motor_MoveDistance(
      active_move_progress_sign * active_move_distance_mm * 0.001f,
      active_move_speed_mm_s);
  const float progress_mm = fminf(active_move_distance_mm,
      fmaxf(0.0f, Motor_GetDistanceTravelledMm()));
  debug_status.progress = (int16_t)(active_move_progress_sign * progress_mm);
  debug_status.remaining = debug_saturate_u16(active_move_distance_mm - progress_mm);
  if (result == MOTOR_DISTANCE_DONE) {
    debug_status.progress = (int16_t)(active_move_progress_sign * active_move_distance_mm);
    debug_status.remaining = 0U;
    debug_finish(DEBUG_MOTION_DONE, DEBUG_MOTION_FAULT_NONE, now_ms);
  } else if (result == MOTOR_DISTANCE_FAULT) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_MOTOR, now_ms);
  } else if (result != MOTOR_DISTANCE_RUNNING) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_INVALID, now_ms);
  } else if ((uint32_t)(now_ms - active_start_ms) >= active_timeout_ms) {
    debug_finish(DEBUG_MOTION_FAULT, DEBUG_MOTION_FAULT_TIMEOUT, now_ms);
  }
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
  if (active_move_body_straight) {
    debug_update_straight(now_ms);
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
  float bounded_progress_mm = (progress_mm > 0.0f) ? progress_mm : 0.0f;
  if (bounded_progress_mm > active_move_distance_mm) {
    bounded_progress_mm = active_move_distance_mm;
  }
  const float remaining_mm = active_move_distance_mm - bounded_progress_mm;
  debug_status.progress = (int16_t)(active_move_progress_sign *
                                    bounded_progress_mm);
  debug_status.remaining = debug_saturate_u16(remaining_mm);

  if (remaining_mm <= APP_MOTION_DEBUG_DISTANCE_TOLERANCE_MM) {
    debug_status.progress = (int16_t)(active_move_progress_sign *
                                     active_move_distance_mm);
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

  const float body_angle_deg = active_command == VISION_MOTION_CMD_MOVE_FIELD ?
      debug_wrap_degrees(active_move_field_angle_rad / DEBUG_DEG_RAD -
                         (float)pose.heading_mdeg * 0.001f) : active_move_body_angle_deg;
  (void)Motor_MoveAngle(speed_mm_s, body_angle_deg);
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
  debug_status.source = DEBUG_SOURCE_NONE;
  debug_leave_critical(primask);

  initialized = true;
  active = false;
  wheel_check_active = false;
  console_owner = false;
  console_pending = false;
  console_sequence = 0U;
  command_sequence_valid = false;
  last_command_sequence = 0U;
  active_command = VISION_MOTION_CMD_STOP;
  active_turn_angle_deg = 0.0f;
  active_turn_speed_mm_s = 0.0f;
  active_move_start_x_mm = 0.0f;
  active_move_start_y_mm = 0.0f;
  active_move_field_angle_rad = 0.0f;
  active_move_distance_mm = 0.0f;
  active_move_speed_mm_s = 0.0f;
  active_move_body_angle_deg = 0.0f;
  active_move_progress_sign = 1.0f;
  active_move_body_straight = false;
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

  VisionData vision = Vision_GetSnapshot();
  if (vision.motion_valid &&
      (!command_sequence_valid ||
       (vision.motion_sequence != last_command_sequence))) {
    command_sequence_valid = true;
    last_command_sequence = vision.motion_sequence;
    if (vision.motion_opcode == VISION_MOTION_CMD_STOP) {
      console_pending = false;
      console_owner = false;
#if APP_ENABLE_MOTION_DEBUG_TASK
      if (wheel_check_active) {
        WheelCheck_Stop();
      }
#endif
      debug_start_command(&vision, now_ms, DEBUG_SOURCE_UPPER);
    } else if (!wheel_check_active && !console_owner && !console_pending) {
      debug_start_command(&vision, now_ms, DEBUG_SOURCE_UPPER);
    }
  }

  if (wheel_check_active) {
    debug_publish_status(now_ms, false);
    return;
  }

  if (console_pending) {
    vision.motion_opcode = console_request.opcode;
    vision.motion_sequence = console_request.sequence;
    vision.motion_arg1 = console_request.arg1;
    vision.motion_arg2 = console_request.arg2;
    vision.motion_speed = console_request.speed;
    console_pending = false;
    debug_start_command(&vision, now_ms, DEBUG_SOURCE_CONSOLE);
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

  if (!active && !console_pending && console_owner) {
    const uint32_t primask = debug_enter_critical();
    const VisionData latest = Vision_GetSnapshot();
    if (latest.motion_valid) {
      command_sequence_valid = true;
      last_command_sequence = latest.motion_sequence;
    }
    console_owner = false;
    debug_leave_critical(primask);
  }
  debug_publish_status(now_ms, false);
}

DebugStatus Debug_GetStatus(void)
{
  const uint32_t primask = debug_enter_critical();
  DebugStatus status = debug_status;
  status.console_pending = console_pending;
  debug_leave_critical(primask);
  return status;
}

void Debug_SetWheelCheckActive(bool enabled)
{
  const uint32_t primask = debug_enter_critical();
  const VisionData vision = Vision_GetSnapshot();
  if (vision.motion_valid) {
    command_sequence_valid = true;
    last_command_sequence = vision.motion_sequence;
  }
  active = false;
  console_pending = false;
  console_owner = false;
  active_move_body_straight = false;
  Motor_Stop();
  debug_status.state = DEBUG_MOTION_STOPPED;
  debug_status.fault = DEBUG_MOTION_FAULT_NONE;
  debug_status.progress = 0;
  debug_status.remaining = 0U;
  if (enabled) {
    debug_status.source = DEBUG_SOURCE_WHEELS;
  }
  wheel_check_active = enabled;
  debug_leave_critical(primask);
}

DebugConsoleResult Debug_SubmitConsoleCommand(uint8_t opcode, int16_t arg1,
    int16_t arg2, uint16_t speed, uint8_t *sequence)
{
  const int64_t distance_sq = (int64_t)arg1 * arg1 + (int64_t)arg2 * arg2;
  bool valid = false;
  if ((opcode == VISION_MOTION_CMD_STOP) || (opcode == VISION_MOTION_CMD_HOLD) ||
      (opcode == VISION_MOTION_CMD_RESET_ODOM)) {
    valid = (arg1 == 0) && (arg2 == 0) && (speed == 0U);
  } else if (opcode == VISION_MOTION_CMD_TURN_REL) {
    valid = (arg1 != 0) && (arg1 >= -3600) && (arg1 <= 3600) && (arg2 == 0) &&
            (speed <= APP_MOTION_DEBUG_MAX_TURN_RATE_DDEG_S);
  } else if ((opcode == VISION_MOTION_CMD_MOVE_BODY) ||
             (opcode == VISION_MOTION_CMD_MOVE_FIELD)) {
    valid = (distance_sq > 0LL) &&
        (distance_sq <= (int64_t)APP_MOTION_DEBUG_MAX_DISTANCE_MM *
                            APP_MOTION_DEBUG_MAX_DISTANCE_MM) &&
        ((speed == 0U) || ((speed >= APP_MOTION_DEBUG_MIN_SPEED_MM_S) &&
                          (speed <= APP_MOTION_DEBUG_MAX_SPEED_MM_S)));
  }
  if (!valid) {
    return DEBUG_CONSOLE_INVALID;
  }
  const uint32_t primask = debug_enter_critical();
  const bool stopping = (opcode == VISION_MOTION_CMD_STOP) ||
                        (opcode == VISION_MOTION_CMD_HOLD);
  if (!stopping &&
      (wheel_check_active || active || console_pending)) {
    debug_leave_critical(primask);
    return DEBUG_CONSOLE_BUSY;
  }
  const VisionData latest = Vision_GetSnapshot();
  if (latest.motion_valid) {
    command_sequence_valid = true;
    last_command_sequence = latest.motion_sequence;
  }
  ++console_sequence;
  if (sequence != NULL) {
    *sequence = console_sequence;
  }
  console_request.opcode = opcode;
  console_request.sequence = console_sequence;
  console_request.arg1 = arg1;
  console_request.arg2 = arg2;
  console_request.speed = speed;
  console_owner = true;
  console_pending = true;
  debug_status.source = DEBUG_SOURCE_CONSOLE;
  debug_status.command = opcode;
  debug_status.command_sequence = console_sequence;
  debug_status.progress = 0;
  debug_status.remaining = 0U;
  debug_status.fault = DEBUG_MOTION_FAULT_NONE;
  debug_status.state = DEBUG_MOTION_IDLE;
  if (stopping) {
#if APP_ENABLE_MOTION_DEBUG_TASK
    if (wheel_check_active) {
      WheelCheck_Stop();
    }
#endif
    active = false;
    Motor_Stop();
    console_pending = false;
    console_owner = false;
    debug_status.state = (opcode == VISION_MOTION_CMD_STOP) ?
        DEBUG_MOTION_STOPPED : DEBUG_MOTION_DONE;
  }
  debug_leave_critical(primask);
  return DEBUG_CONSOLE_ACCEPTED;
}
