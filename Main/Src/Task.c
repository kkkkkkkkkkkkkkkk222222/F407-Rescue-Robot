#include "Task.h"

#include "app_config.h"
#include "mechanism.h"
#include "motor.h"
#include "pid.h"
#include "vision.h"

static TaskState_t task_state;
static uint32_t state_tick;
static uint32_t update_tick;
static Pid_t steering_pid;
static Pid_t camera_pid;
static float camera_angle;

static void task_set_state(TaskState_t state, uint32_t now_ms)
{
  task_state = state;
  state_tick = now_ms;
  if (state == TASK_APPROACH_TARGET) {
    Pid_Reset(&steering_pid);
    Pid_Reset(&camera_pid);
  }
  if (state == TASK_STOPPED) {
    Motor_Stop();
  }
}

void Task_Init(uint32_t now_ms)
{
  Pid_Init(&steering_pid, 3.0f, 0.0f, 2.0f,
           -320.0f, 320.0f, -1000.0f, 1000.0f);
  Pid_Init(&camera_pid, 0.5f, 0.0f, 0.3f,
           -3.0f, 3.0f, -1000.0f, 1000.0f);
  Mechanism_Init();
  camera_angle = (float)Camera_GetAngle();
  Motor_Stop();
  update_tick = now_ms;
  task_set_state(TASK_CAMERA_WIDE, now_ms);
}

void Task_Update(uint32_t now_ms)
{
  VisionData vision;

  if ((uint32_t)(now_ms - update_tick) < APP_TASK_PERIOD_MS) {
    return;
  }
  update_tick = now_ms;
  vision = Vision_GetSnapshot();

  if (vision.stop) {
    task_set_state(TASK_STOPPED, now_ms);
    return;
  }
  if (task_state == TASK_STOPPED) {
    Motor_Stop();
    return;
  }

  switch (task_state) {
    case TASK_CAMERA_WIDE:
      Motor_Stop();
      if ((uint32_t)(now_ms - state_tick) >= APP_CAMERA_SETTLE_MS) {
        task_set_state(TASK_WAIT_TARGET, now_ms);
      }
      break;

    case TASK_WAIT_TARGET:
      Motor_Stop();
      if (Vision_IsFresh(&vision, now_ms, APP_VISION_TIMEOUT_MS)) {
        task_set_state(TASK_APPROACH_TARGET, now_ms);
      } else if ((uint32_t)(now_ms - state_tick) >= APP_INITIAL_SEARCH_MS) {
        task_set_state(TASK_CLEAR_BUMP, now_ms);
      }
      break;

    case TASK_CLEAR_BUMP:
      if (Vision_IsFresh(&vision, now_ms, APP_VISION_TIMEOUT_MS)) {
        Motor_Stop();
        task_set_state(TASK_APPROACH_TARGET, now_ms);
      } else if ((uint32_t)(now_ms - state_tick) < APP_CLEAR_BUMP_MS) {
        Motor_Forward(APP_CLEAR_BUMP_SPEED);
      } else {
        Motor_Stop();
        task_set_state(TASK_SEARCH_TARGET, now_ms);
      }
      break;

    case TASK_SEARCH_TARGET:
      if (Vision_IsFresh(&vision, now_ms, APP_VISION_TIMEOUT_MS)) {
        Motor_Stop();
        task_set_state(TASK_APPROACH_TARGET, now_ms);
      } else {
        Motor_RotateLeft(APP_SEARCH_ROTATE_SPEED);
      }
      break;

    case TASK_APPROACH_TARGET:
      if (Vision_IsFresh(&vision, now_ms, APP_VISION_TIMEOUT_MS)) {
        float turn;
        float camera_change;
        const int32_t x_error = (int32_t)APP_VISION_TARGET_X - vision.x;

        if ((x_error >= -APP_STEERING_DEAD_ZONE) &&
            (x_error <= APP_STEERING_DEAD_ZONE)) {
          turn = 0.0f;
          Pid_Reset(&steering_pid);
        } else {
          turn = Pid_Update(&steering_pid,
                            (float)APP_VISION_TARGET_X,
                            (float)vision.x);
        }
        Motor_Follow(APP_APPROACH_SPEED,
                     (int16_t)(turn * APP_STEERING_DIRECTION));

        camera_change = Pid_Update(&camera_pid,
                                   (float)APP_VISION_TARGET_Y,
                                   (float)vision.y) * APP_CAMERA_DIRECTION;
        camera_angle += camera_change;
        if (camera_angle < (float)APP_CAMERA_MIN_ANGLE) {
          camera_angle = (float)APP_CAMERA_MIN_ANGLE;
        } else if (camera_angle > (float)APP_CAMERA_MAX_ANGLE) {
          camera_angle = (float)APP_CAMERA_MAX_ANGLE;
        }
        Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
      } else {
        Motor_Stop();
        task_set_state(TASK_SEARCH_TARGET, now_ms);
      }
      break;

    default:
      Motor_Stop();
      break;
  }

  Motor_Update();
}

TaskState_t Task_GetState(void)
{
  return task_state;
}
