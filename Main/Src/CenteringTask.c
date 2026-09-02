#include "CenteringTask.h"

#include <stdbool.h>

#include "app_config.h"
#include "main.h"
#include "mechanism.h"
#include "motor.h"
#include "pid.h"
#include "vision.h"

static volatile CenteringTaskStatus centering_status;
static Pid_t rotation_pid;
static Pid_t camera_pid;
static float camera_angle;
static float rotation_command_mm_s;
static uint32_t previous_report_ms;
static uint8_t previous_sequence;
static bool sequence_valid;
static bool rotation_active;
static bool initialized;

static float centering_report_dt(uint32_t report_ms)
{
  if (!sequence_valid) {
    return APP_VISION_PID_DEFAULT_DT_S;
  }

  float dt_s = (float)(uint32_t)(report_ms - previous_report_ms) * 0.001f;
  if (dt_s < APP_VISION_PID_MIN_DT_S) {
    dt_s = APP_VISION_PID_MIN_DT_S;
  } else if (dt_s > APP_VISION_PID_MAX_DT_S) {
    dt_s = APP_VISION_PID_MAX_DT_S;
  }
  return dt_s;
}

static bool centering_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static void centering_stop(CenteringState state)
{
  Motor_Stop();
  Pid_Reset(&rotation_pid);
  Pid_Reset(&camera_pid);
  rotation_command_mm_s = 0.0f;
  rotation_active = false;
  sequence_valid = false;
  centering_status.state = state;
  centering_status.rotation_mm_s = 0;
}

void CenteringTask_Init(uint32_t now_ms)
{
  Pid_Init(&rotation_pid,
           APP_STEERING_KP_MM_S, 0.0f, APP_STEERING_KD_MM,
           -APP_STEERING_LIMIT_MM_S, APP_STEERING_LIMIT_MM_S,
           -40.0f, 40.0f);
  Pid_Init(&camera_pid,
           APP_CAMERA_KP_DEG_PER_PX,
           APP_CAMERA_KI_DEG_PER_PX_S,
           APP_CAMERA_KD_DEG_S_PER_PX,
           -2.0f, 2.0f,
           -APP_CAMERA_INTEGRAL_LIMIT_PX_S,
           APP_CAMERA_INTEGRAL_LIMIT_PX_S);

  Mechanism_Init();
  Camera_SetAngle(APP_CENTERING_CAMERA_START_ANGLE);
  Motor_Stop();
  camera_angle = (float)Camera_GetAngle();
  rotation_command_mm_s = 0.0f;
  previous_report_ms = now_ms;
  previous_sequence = 0U;
  sequence_valid = false;
  rotation_active = false;
  centering_status.state = CENTERING_WAIT_TARGET;
  centering_status.camera_angle = Camera_GetAngle();
  centering_status.rotation_mm_s = 0;
  initialized = true;
}

void CenteringTask_Process(uint32_t now_ms)
{
  if (!initialized) {
    CenteringTask_Init(now_ms);
  }

  if (centering_motor_fault()) {
    centering_stop(CENTERING_MOTOR_FAULT);
    return;
  }

  const VisionData vision = Vision_GetSnapshot();
  if (!Vision_IsFresh(&vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    centering_stop(CENTERING_WAIT_TARGET);
    return;
  }

  if (sequence_valid && (vision.sequence == previous_sequence)) {
    return;
  }

  const float dt_s = centering_report_dt(vision.tick_ms);
  previous_report_ms = vision.tick_ms;
  previous_sequence = vision.sequence;
  sequence_valid = true;

  const int32_t y_error = (int32_t)APP_VISION_TARGET_Y - vision.y;
  if ((y_error >= -APP_CAMERA_DEAD_ZONE) &&
      (y_error <= APP_CAMERA_DEAD_ZONE)) {
    Pid_Reset(&camera_pid);
  } else {
    camera_angle -= Pid_UpdateDt(&camera_pid,
                                 (float)APP_VISION_TARGET_Y,
                                 (float)vision.y,
                                 dt_s);
    if (camera_angle < (float)APP_CENTERING_CAMERA_MIN_ANGLE) {
      camera_angle = (float)APP_CENTERING_CAMERA_MIN_ANGLE;
    } else if (camera_angle > (float)APP_CENTERING_CAMERA_MAX_ANGLE) {
      camera_angle = (float)APP_CENTERING_CAMERA_MAX_ANGLE;
    }
    Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
  }

  const int32_t x_error = (int32_t)APP_VISION_TARGET_X - vision.x;
  const int32_t x_magnitude = (x_error < 0) ? -x_error : x_error;
  if ((rotation_active &&
       (x_magnitude <= APP_STEERING_EXIT_DEAD_ZONE)) ||
      (!rotation_active &&
       (x_magnitude <= APP_STEERING_ENTER_DEAD_ZONE))) {
    rotation_active = false;
    rotation_command_mm_s = 0.0f;
    Pid_Reset(&rotation_pid);
  } else {
    rotation_active = true;
    rotation_command_mm_s =
        Pid_UpdateDt(&rotation_pid,
                     (float)APP_VISION_TARGET_X,
                     (float)vision.x,
                     dt_s) * APP_STEERING_DIRECTION;
    if ((rotation_command_mm_s > 0.0f) &&
        (rotation_command_mm_s < APP_STEERING_MIN_MM_S)) {
      rotation_command_mm_s = APP_STEERING_MIN_MM_S;
    } else if ((rotation_command_mm_s < 0.0f) &&
               (rotation_command_mm_s > -APP_STEERING_MIN_MM_S)) {
      rotation_command_mm_s = -APP_STEERING_MIN_MM_S;
    }
  }

  if (rotation_active) {
    Motor_Move(0.0f, 0.0f, rotation_command_mm_s);
  } else {
    Motor_Stop();
  }

  const bool y_centered =
      (y_error >= -APP_CAMERA_DEAD_ZONE) &&
      (y_error <= APP_CAMERA_DEAD_ZONE);
  centering_status.state =
      (!rotation_active && y_centered) ? CENTERING_CENTERED :
                                        CENTERING_TRACKING;
  centering_status.camera_angle = Camera_GetAngle();
  centering_status.rotation_mm_s =
      (int16_t)(rotation_command_mm_s +
                ((rotation_command_mm_s >= 0.0f) ? 0.5f : -0.5f));
}

CenteringTaskStatus CenteringTask_GetStatus(void)
{
  CenteringTaskStatus status;
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  status = centering_status;
  if (primask == 0U) {
    __enable_irq();
  }
  return status;
}
