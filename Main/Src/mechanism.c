#include "mechanism.h"

#include "app_config.h"
#include "servo.h"

static uint8_t camera_angle = 90U;

typedef enum {
  CLAW_ACTION_NONE = 0,
  CLAW_ACTION_OPEN,
  CLAW_ACTION_OPEN_LEFT,
  CLAW_ACTION_OPEN_RIGHT,
  CLAW_ACTION_CLUSTER_OPEN_LEFT,
  CLAW_ACTION_CLUSTER_OPEN_RIGHT,
  CLAW_ACTION_RETRACT,
  CLAW_ACTION_TOUCH
} ClawAction;

static ClawAction claw_action;
static uint8_t claw_step;
static uint32_t claw_step_ms;
static bool claw_done;
static uint8_t separation_relax_deg;
static bool separation_cluster_base;

void Claw_SetSeparationRelax(uint8_t degrees, bool cluster_base)
{
  if (degrees > 20U) degrees = 20U;
  if (degrees != separation_relax_deg || cluster_base != separation_cluster_base) {
    separation_relax_deg = degrees;
    separation_cluster_base = cluster_base;
    claw_action = CLAW_ACTION_NONE;
  }
}

static uint8_t claw_relaxed_left(void)
{
  const uint8_t angle = separation_cluster_base ? APP_CLAW_LEFT_CLUSTER_HOLD_ANGLE :
                                                 APP_CLAW_LEFT_SEPARATE_HOLD_ANGLE;
  const uint16_t relaxed = (uint16_t)angle + separation_relax_deg;
  return relaxed > 108U ? 108U : (uint8_t)relaxed;
}

static uint8_t claw_relaxed_right(void)
{
  const uint8_t angle = separation_cluster_base ? APP_CLAW_RIGHT_CLUSTER_HOLD_ANGLE :
                                                 APP_CLAW_RIGHT_SEPARATE_HOLD_ANGLE;
  return angle < 72U + separation_relax_deg ? 72U : angle - separation_relax_deg;
}

static bool claw_move_sequential(ClawAction action, uint32_t now_ms,
                                 uint8_t first_servo, uint8_t first_angle,
                                 uint8_t second_servo, uint8_t second_angle)
{
  if (claw_action != action) {
    claw_action = action;
    claw_step = 1U;
    claw_step_ms = now_ms;
    claw_done = false;
    Servo_SetAngle(first_servo, first_angle);
    return false;
  }

  if (claw_done) {
    return true;
  }
  if ((uint32_t)(now_ms - claw_step_ms) < 1000U) {
    return false;
  }

  if (claw_step == 1U) {
    Servo_SetAngle(second_servo, second_angle);
    claw_step = 2U;
    claw_step_ms = now_ms;
    return false;
  }

  claw_done = true;
  return true;
}

static bool claw_move_together(ClawAction action, uint32_t now_ms,
                               uint8_t left_angle, uint8_t right_angle,
                               uint32_t wait_ms)
{
  if (claw_action != action) {
    claw_action = action;
    claw_step_ms = now_ms;
    claw_done = false;
    Servo_SetAngle(4U, left_angle);
    Servo_SetAngle(2U, right_angle);
    return false;
  }

  if (!claw_done && ((uint32_t)(now_ms - claw_step_ms) >= wait_ms)) {
    claw_done = true;
  }
  return claw_done;
}

void Mechanism_Init(void)
{
  separation_relax_deg = 0U;
  separation_cluster_base = false;
  claw_action = CLAW_ACTION_NONE;
  claw_step = 0U;
  claw_step_ms = 0U;
  claw_done = false;
  Camera_SetAngle(90U);
}

void Camera_SetAngle(uint8_t angle)
{
#if APP_CAMERA_MIN_ANGLE > 0U
  if (angle < APP_CAMERA_MIN_ANGLE) {
    angle = APP_CAMERA_MIN_ANGLE;
  }
#endif
  if (angle > APP_CAMERA_MAX_ANGLE) {
    angle = APP_CAMERA_MAX_ANGLE;
  }
  camera_angle = angle;
  Servo_SetAngle(3U, camera_angle);
}

uint8_t Camera_GetAngle(void)
{
  return camera_angle;
}

void Lift_SetStartPosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_START_ANGLE);
}

void Lift_SetTravelPosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_TRAVEL_ANGLE);
}

bool Claw_Open(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_OPEN, now_ms, 108U, 72U, 1000U);
}

bool Claw_OpenLeft(uint32_t now_ms)
{
  /* Release left and use the task's latched baseline/attempt relaxation. */
  return claw_move_together(CLAW_ACTION_OPEN_LEFT, now_ms,
                            108U, claw_relaxed_right(), 600U);
}

bool Claw_OpenRight(uint32_t now_ms)
{
  /* Mirror of Claw_OpenLeft: a smaller left-servo angle closes it farther. */
  return claw_move_together(CLAW_ACTION_OPEN_RIGHT, now_ms,
                            claw_relaxed_left(), 72U, 600U);
}

bool Claw_ClusterOpenLeft(uint32_t now_ms)
{
  /* Curve separation uses the same retained-side profile across retries. */
  return claw_move_together(CLAW_ACTION_CLUSTER_OPEN_LEFT, now_ms,
                            108U, claw_relaxed_right(), 600U);
}

bool Claw_ClusterOpenRight(uint32_t now_ms)
{
  /* Mirror of Claw_ClusterOpenLeft for a retained left-side target. */
  return claw_move_together(CLAW_ACTION_CLUSTER_OPEN_RIGHT, now_ms,
                            claw_relaxed_left(), 72U, 600U);
}

bool Claw_Retract(uint32_t now_ms)
{
  /* Fold the left claw first, then place the right claw on the outside. */
  return claw_move_sequential(CLAW_ACTION_RETRACT, now_ms,
                              4U, 23U, 2U, 147U);
}

bool Claw_Touch(uint32_t now_ms)
{
  /* Both claws move together, but keep the existing two-second confirmation
   * window before GRIPPER_CLOSED is reported to the RDK. */
  return claw_move_together(CLAW_ACTION_TOUCH, now_ms,
                            APP_CLAW_LEFT_TOUCH_ANGLE,
                            APP_CLAW_RIGHT_TOUCH_ANGLE, 2000U);
}
