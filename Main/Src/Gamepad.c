#include "Gamepad.h"

#include <math.h>

#include "Location.h"
#include "app_config.h"
#include "imu.h"
#include "main.h"
#include "mechanism.h"
#include "motor.h"
#include "servo.h"
#include "vision.h"

typedef struct {
  int8_t forward;
  int8_t left;
  int8_t yaw;
  int8_t camera;
  uint8_t buttons;
  uint8_t speed_percent;
} GamepadCommand;

static volatile GamepadStatus gamepad_status;
static GamepadCommand command;
static float camera_angle;
static float lift_angle;
static float left_claw_angle;
static float right_claw_angle;
static uint32_t last_command_ms;
static uint32_t last_process_ms;
static uint32_t last_status_ms;
static uint8_t last_sequence;
static bool sequence_valid;

static uint32_t gamepad_enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void gamepad_leave_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static float gamepad_clamp(float value, float minimum, float maximum)
{
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static float gamepad_axis(int8_t value)
{
  const int16_t magnitude = (value < 0) ? -(int16_t)value : (int16_t)value;
  if (magnitude <= APP_GAMEPAD_AXIS_DEAD_ZONE) {
    return 0.0f;
  }
  const float scaled = (float)(magnitude - APP_GAMEPAD_AXIS_DEAD_ZONE) /
      (float)(100 - APP_GAMEPAD_AXIS_DEAD_ZONE);
  return (value < 0) ? -scaled : scaled;
}

static int16_t gamepad_round_speed(float value)
{
  if (value >= 32767.0f) {
    return INT16_MAX;
  }
  if (value <= -32768.0f) {
    return INT16_MIN;
  }
  return (int16_t)((value >= 0.0f) ? value + 0.5f : value - 0.5f);
}

static bool gamepad_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static uint16_t gamepad_heading_cdeg(void)
{
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    return 0U;
  }
  return (uint16_t)((uint32_t)pose.heading_mdeg / 10U);
}

static void gamepad_publish_status(uint32_t now_ms, bool force)
{
  if (!force &&
      ((uint32_t)(now_ms - last_status_ms) < APP_GAMEPAD_STATUS_PERIOD_MS)) {
    return;
  }
  uint8_t state = 0U;
  uint8_t fault = 0U;
  if (gamepad_status.state == GAMEPAD_RUNNING) {
    state = 1U;
  } else if (gamepad_status.state == GAMEPAD_FAULT) {
    state = 3U;
    fault = 5U;
  } else if ((gamepad_status.state == GAMEPAD_TIMEOUT) ||
             (gamepad_status.state == GAMEPAD_STOPPED)) {
    state = 4U;
  }
  const VisionMotionStatus status = {
    .command_sequence = gamepad_status.command_sequence,
    .state = state,
    .fault = fault,
    .command = VISION_MOTION_CMD_TELEOP,
    .progress = 0,
    .heading_cdeg = gamepad_heading_cdeg()
  };
  Vision_QueueMotionStatus(&status);
  last_status_ms = now_ms;
}

static void gamepad_set_servo(uint8_t id, float angle)
{
  const uint8_t rounded = (uint8_t)(angle + 0.5f);
  if (Servo_GetAngle(id) != rounded) {
    Servo_SetAngle(id, rounded);
  }
}

static void gamepad_update_servos(float dt_s)
{
  camera_angle = gamepad_clamp(
      camera_angle + gamepad_axis(command.camera) *
          APP_GAMEPAD_CAMERA_RATE_DEG_S * dt_s,
      APP_GAMEPAD_CAMERA_MIN_ANGLE, APP_GAMEPAD_CAMERA_MAX_ANGLE);

  const bool lift_up =
      (command.buttons & VISION_TELEOP_LIFT_UP) != 0U;
  const bool lift_down =
      (command.buttons & VISION_TELEOP_LIFT_DOWN) != 0U;
  if (lift_up != lift_down) {
    lift_angle += (lift_up ? 1.0f : -1.0f) *
        APP_GAMEPAD_LIFT_RATE_DEG_S * dt_s;
  }
  lift_angle = gamepad_clamp(lift_angle,
                             APP_GAMEPAD_LIFT_MIN_ANGLE,
                             APP_GAMEPAD_LIFT_MAX_ANGLE);

  const bool claw_open =
      (command.buttons & VISION_TELEOP_CLAW_OPEN) != 0U;
  const bool claw_close =
      (command.buttons & VISION_TELEOP_CLAW_CLOSE) != 0U;
  if (claw_open != claw_close) {
    const float direction = claw_open ? 1.0f : -1.0f;
    left_claw_angle += direction * APP_GAMEPAD_CLAW_RATE_DEG_S * dt_s;
    right_claw_angle -= direction * APP_GAMEPAD_CLAW_RATE_DEG_S * dt_s;
  }
  left_claw_angle = gamepad_clamp(
      left_claw_angle, APP_GAMEPAD_CLAW_LEFT_CLOSED_ANGLE,
      APP_GAMEPAD_CLAW_LEFT_OPEN_ANGLE);
  right_claw_angle = gamepad_clamp(
      right_claw_angle, APP_GAMEPAD_CLAW_RIGHT_OPEN_ANGLE,
      APP_GAMEPAD_CLAW_RIGHT_CLOSED_ANGLE);

  Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
  gamepad_set_servo(1U, lift_angle);
  gamepad_set_servo(4U, left_claw_angle);
  gamepad_set_servo(2U, right_claw_angle);
}

static void gamepad_stop(GamepadState state)
{
  Motor_Stop();
  gamepad_status.state = state;
  gamepad_status.armed = false;
  gamepad_status.forward_mm_s = 0;
  gamepad_status.left_mm_s = 0;
  gamepad_status.yaw_mm_s = 0;
}

static void gamepad_accept_command(const VisionData *vision,
                                   uint32_t now_ms)
{
  gamepad_status.received = true;
  gamepad_status.command_sequence = vision->motion_sequence;
  last_command_ms = vision->motion_tick_ms;

  if (vision->motion_opcode == VISION_MOTION_CMD_STOP) {
    gamepad_stop(GAMEPAD_STOPPED);
    return;
  }
  if (vision->motion_opcode != VISION_MOTION_CMD_TELEOP) {
    gamepad_stop(GAMEPAD_STOPPED);
    return;
  }

  command.forward = vision->teleop_forward;
  command.left = vision->teleop_left;
  command.yaw = vision->teleop_yaw;
  command.camera = vision->teleop_camera;
  command.buttons = vision->teleop_buttons;
  command.speed_percent = vision->teleop_speed_percent;
  gamepad_status.buttons = command.buttons;
  gamepad_status.armed =
      (vision->motion_flags & VISION_MOTION_TELEOP_ENABLE) != 0U;
  gamepad_status.state = gamepad_status.armed ?
      GAMEPAD_RUNNING : GAMEPAD_IDLE;
  (void)now_ms;
}

void Gamepad_Init(uint32_t now_ms)
{
  Mechanism_Init();
  Lift_SetStartPosition();
  Servo_SetAngle(4U, (uint8_t)APP_GAMEPAD_CLAW_LEFT_CLOSED_ANGLE);
  Servo_SetAngle(2U, (uint8_t)APP_GAMEPAD_CLAW_RIGHT_CLOSED_ANGLE);
  Location_ResetReference(0, 0, 0);

  command = (GamepadCommand){0};
  gamepad_status = (GamepadStatus){0};
  gamepad_status.state = GAMEPAD_IDLE;
  camera_angle = (float)Camera_GetAngle();
  lift_angle = (float)Servo_GetAngle(1U);
  left_claw_angle = (float)Servo_GetAngle(4U);
  right_claw_angle = (float)Servo_GetAngle(2U);
  gamepad_status.camera_angle = (uint8_t)camera_angle;
  gamepad_status.lift_angle = (uint8_t)lift_angle;
  gamepad_status.left_claw_angle = (uint8_t)left_claw_angle;
  gamepad_status.right_claw_angle = (uint8_t)right_claw_angle;
  last_command_ms = now_ms;
  last_process_ms = now_ms;
  last_status_ms = now_ms - APP_GAMEPAD_STATUS_PERIOD_MS;
  last_sequence = 0U;
  sequence_valid = false;
  gamepad_publish_status(now_ms, true);
}

void Gamepad_Process(uint32_t now_ms)
{
  const VisionData vision = Vision_GetSnapshot();
  if (vision.motion_valid &&
      (!sequence_valid || (vision.motion_sequence != last_sequence))) {
    sequence_valid = true;
    last_sequence = vision.motion_sequence;
    gamepad_accept_command(&vision, now_ms);
  }

  uint32_t elapsed_ms = now_ms - last_process_ms;
  last_process_ms = now_ms;
  if (elapsed_ms > 100U) {
    elapsed_ms = 100U;
  }

  if (!gamepad_status.received ||
      ((uint32_t)(now_ms - last_command_ms) >
       APP_GAMEPAD_COMMAND_TIMEOUT_MS)) {
    gamepad_stop(gamepad_status.received ? GAMEPAD_TIMEOUT : GAMEPAD_IDLE);
  } else if (gamepad_motor_fault() || !IMU_GetData().ready) {
    gamepad_stop(GAMEPAD_FAULT);
  } else if (!gamepad_status.armed) {
    gamepad_stop(GAMEPAD_IDLE);
  } else {
    const float scale = (float)command.speed_percent * 0.01f;
    float forward = gamepad_axis(command.forward);
    float left = gamepad_axis(command.left);
    const float vector_magnitude = sqrtf(forward * forward + left * left);
    if (vector_magnitude > 1.0f) {
      forward /= vector_magnitude;
      left /= vector_magnitude;
    }
    forward *= APP_GAMEPAD_MAX_LINEAR_MM_S * scale;
    left *= APP_GAMEPAD_MAX_LINEAR_MM_S * scale;
    const float yaw = gamepad_axis(command.yaw) *
                      APP_GAMEPAD_MAX_YAW_MM_S * scale;

    gamepad_update_servos((float)elapsed_ms * 0.001f);
    Motor_Move(forward, left * APP_OMNI_LATERAL_API_SIGN, yaw);
    gamepad_status.state = GAMEPAD_RUNNING;
    gamepad_status.forward_mm_s = gamepad_round_speed(forward);
    gamepad_status.left_mm_s = gamepad_round_speed(left);
    gamepad_status.yaw_mm_s = gamepad_round_speed(yaw);
  }

  gamepad_status.camera_angle = Camera_GetAngle();
  gamepad_status.lift_angle = Servo_GetAngle(1U);
  gamepad_status.left_claw_angle = Servo_GetAngle(4U);
  gamepad_status.right_claw_angle = Servo_GetAngle(2U);
  gamepad_publish_status(now_ms, false);
}

GamepadStatus Gamepad_GetStatus(void)
{
  const uint32_t primask = gamepad_enter_critical();
  const GamepadStatus status = gamepad_status;
  gamepad_leave_critical(primask);
  return status;
}
