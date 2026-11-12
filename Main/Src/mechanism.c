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
  CLAW_ACTION_BUMP_TOUCH,
  CLAW_ACTION_BUMP_OPEN,
  CLAW_ACTION_RETRACT,
  CLAW_ACTION_TOUCH,
  CLAW_ACTION_HALF_TOUCH
} ClawAction;

static ClawAction claw_action;
static uint8_t claw_step;
static uint32_t claw_step_ms;
static uint32_t claw_wait_ms;
static bool claw_done;
static uint8_t separation_extra_close_deg;

void Claw_RestartAction(void)
{
  claw_action = CLAW_ACTION_NONE;
}

void Claw_SetSeparationHold(uint8_t extra_close_deg)
{
  if (extra_close_deg > APP_CLAW_SEPARATE_ONE_OR_TWO_CLOSE_DEG) {
    extra_close_deg = APP_CLAW_SEPARATE_ONE_OR_TWO_CLOSE_DEG;
  }
  if (extra_close_deg != separation_extra_close_deg) {
    separation_extra_close_deg = extra_close_deg;
    claw_action = CLAW_ACTION_NONE;
  }
}

static uint8_t claw_relaxed_left(void)
{
  return APP_CLAW_LEFT_TOUCH_ANGLE - separation_extra_close_deg;
}

static uint8_t claw_relaxed_right(void)
{
  return APP_CLAW_RIGHT_FROM_LEFT(claw_relaxed_left());
}

static uint32_t servo_pulse_wait_ms(uint16_t from_us, uint16_t to_us)
{
  const uint32_t travel_us = from_us > to_us ? from_us - to_us : to_us - from_us;
  /* 130ms/60deg *1.5, with0.09deg/us; ceil(delta_us *117/400). */
  return (travel_us * 117U + 399U) / 400U;
}

static uint32_t claw_limit_wait_ms(uint8_t id, uint8_t angle, uint8_t min_deg, uint8_t max_deg)
{
  const uint16_t target_us = Servo_OutputPulseUs(id, angle);
  const uint16_t min_us = 500U + (uint32_t)APP_CLAW_MOUNT_ANGLE(min_deg) * 2000U / 180U;
  const uint16_t max_us = 500U + (uint32_t)APP_CLAW_MOUNT_ANGLE(max_deg) * 2000U / 180U;
  const uint32_t from_min_ms = servo_pulse_wait_ms(min_us, target_us);
  const uint32_t from_max_ms = servo_pulse_wait_ms(max_us, target_us);
  const uint32_t from_limits_ms = from_min_ms > from_max_ms ? from_min_ms : from_max_ms;
  /* A single-claw release can leave the last target outside the ordinary range. */
  const uint16_t current_us = Servo_OutputPulseUs(id, Servo_GetAngle(id));
  const uint32_t from_current_ms = servo_pulse_wait_ms(current_us, target_us);
  return from_current_ms > from_limits_ms ? from_current_ms : from_limits_ms;
}

static bool claw_move_together(ClawAction action, uint32_t now_ms,
                               uint8_t left_angle, uint8_t right_angle)
{
  if (claw_action != action) {
    if (action == CLAW_ACTION_HALF_TOUCH) {
      claw_wait_ms = APP_INJURY_CLAW_SETTLE_MS;
    } else {
      /* Contact can move a claw away from its last commanded angle. */
      const uint32_t left_wait_ms = claw_limit_wait_ms(
          4U, left_angle, APP_CLAW_LEFT_LIMIT_MIN_DEG, APP_CLAW_LEFT_LIMIT_MAX_DEG);
      const uint32_t right_wait_ms = claw_limit_wait_ms(
          2U, right_angle, APP_CLAW_RIGHT_FROM_LEFT(APP_CLAW_LEFT_LIMIT_MAX_DEG),
          APP_CLAW_RIGHT_FROM_LEFT(APP_CLAW_LEFT_LIMIT_MIN_DEG));
      claw_wait_ms = left_wait_ms > right_wait_ms ? left_wait_ms : right_wait_ms;
    }
    claw_action = action;
    claw_step_ms = now_ms;
    claw_done = false;
    Servo_SetAngle(4U, left_angle);
    Servo_SetAngle(2U, right_angle);
    return false;
  }

  if (!claw_done && ((uint32_t)(now_ms - claw_step_ms) >= claw_wait_ms)) {
    claw_done = true;
  }
  return claw_done;
}

void Mechanism_Init(void)
{
  separation_extra_close_deg = 0U;
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

uint32_t Lift_PositionWaitMs(uint8_t angle)
{
  return servo_pulse_wait_ms(Servo_OutputPulseUs(1U, Servo_GetAngle(1U)),
                            Servo_OutputPulseUs(1U, angle));
}

void Lift_SetTravelPosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_TRAVEL_ANGLE);
}

void Lift_SetFirstGreenFilterPosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_FIRST_GREEN_FILTER_ANGLE);
}

bool Claw_Open(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_OPEN, now_ms,
                            APP_CLAW_LEFT_OPEN_ANGLE, APP_CLAW_RIGHT_OPEN_ANGLE);
}

static bool claw_step_to(uint8_t id, uint8_t target)
{
  const int current = Servo_GetAngle(id);
  if (current == target) { return true; }
  const int step = APP_CLAW_SEPARATE_STEP_DEG;
  int delta = (int)target - current;
  if (delta > step) { delta = step; }
  if (delta < -step) { delta = -step; }
  const uint8_t next = (uint8_t)(current + delta);
  Servo_SetAngle(id, next);
  return next == target;
}

static bool claw_move_separation(ClawAction action, uint32_t now_ms,
                                 uint8_t left_angle, uint8_t right_angle,
                                 uint32_t settle_ms)
{
  if (claw_action != action) {
    claw_action = action;
    claw_step = 1U;
    claw_step_ms = now_ms;
    claw_done = false;
  }
  if (!claw_done && claw_step == 1U &&
      (uint32_t)(now_ms - claw_step_ms) >= APP_TASK_PERIOD_MS) {
    /* At most one step after a pause; never catch up with a large angle jump. */
    const bool left_at_target = claw_step_to(4U, left_angle);
    const bool right_at_target = claw_step_to(2U, right_angle);
    claw_step_ms = now_ms;
    if (left_at_target && right_at_target) { claw_step = 2U; }
  }
  if (claw_step == 2U && (uint32_t)(now_ms - claw_step_ms) >= settle_ms) {
    claw_done = true;
  }
  return claw_done;
}

bool Claw_SetBumpPosition(bool closed, uint32_t now_ms)
{
  if (closed) {
    return claw_move_separation(CLAW_ACTION_BUMP_TOUCH, now_ms,
                                APP_CLAW_LEFT_TOUCH_ANGLE, APP_CLAW_RIGHT_TOUCH_ANGLE, 2000U);
  }
  return claw_move_separation(CLAW_ACTION_BUMP_OPEN, now_ms,
                              APP_CLAW_LEFT_OPEN_ANGLE, APP_CLAW_RIGHT_OPEN_ANGLE, 1000U);
}

bool Claw_OpenLeft(uint32_t now_ms)
{
  /* Release left and use the task's latched baseline/attempt relaxation. */
  return claw_move_separation(CLAW_ACTION_OPEN_LEFT, now_ms,
                              APP_CLAW_LEFT_RELEASE_ANGLE, claw_relaxed_right(),
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_OpenRight(uint32_t now_ms)
{
  /* Mirror of Claw_OpenLeft: a smaller left-servo angle closes it farther. */
  return claw_move_separation(CLAW_ACTION_OPEN_RIGHT, now_ms,
                              claw_relaxed_left(), APP_CLAW_RIGHT_RELEASE_ANGLE,
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_ClusterOpenLeft(uint32_t now_ms)
{
  /* Selective separation uses the same retained-side profile across retries. */
  return claw_move_separation(CLAW_ACTION_CLUSTER_OPEN_LEFT, now_ms,
                              APP_CLAW_LEFT_RELEASE_ANGLE, claw_relaxed_right(),
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_ClusterOpenRight(uint32_t now_ms)
{
  /* Mirror of Claw_ClusterOpenLeft for a retained left-side target. */
  return claw_move_separation(CLAW_ACTION_CLUSTER_OPEN_RIGHT, now_ms,
                              claw_relaxed_left(), APP_CLAW_RIGHT_RELEASE_ANGLE,
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_Retract(uint32_t now_ms)
{
  /* Fold the left claw first, then place the right claw on the outside. */
  if (claw_action != CLAW_ACTION_RETRACT) {
    claw_action = CLAW_ACTION_RETRACT;
    claw_step = 1U;
    claw_step_ms = now_ms;
    claw_done = false;
    claw_wait_ms = claw_limit_wait_ms(4U, APP_CLAW_LEFT_RETRACT_ANGLE,
        APP_CLAW_LEFT_LIMIT_MIN_DEG, APP_CLAW_LEFT_LIMIT_MAX_DEG);
    Servo_SetAngle(4U, APP_CLAW_LEFT_RETRACT_ANGLE);
    return false;
  }
  if (claw_done) { return true; }
  if ((uint32_t)(now_ms - claw_step_ms) < claw_wait_ms) { return false; }
  if (claw_step == 1U) {
    claw_wait_ms = claw_limit_wait_ms(2U, APP_CLAW_RIGHT_RETRACT_ANGLE,
        APP_CLAW_RIGHT_FROM_LEFT(APP_CLAW_LEFT_LIMIT_MAX_DEG),
        APP_CLAW_RIGHT_FROM_LEFT(APP_CLAW_LEFT_LIMIT_MIN_DEG));
    Servo_SetAngle(2U, APP_CLAW_RIGHT_RETRACT_ANGLE);
    claw_step = 2U;
    claw_step_ms = now_ms;
    return false;
  }
  claw_done = true;
  return true;
}

bool Claw_Touch(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_TOUCH, now_ms,
                            APP_CLAW_LEFT_TOUCH_ANGLE, APP_CLAW_RIGHT_TOUCH_ANGLE);
}

bool Claw_HalfTouch(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_HALF_TOUCH, now_ms,
                            APP_CLAW_LEFT_TOUCH_ANGLE + APP_INJURY_CLAW_RELAX_DEG,
                            APP_CLAW_RIGHT_TOUCH_ANGLE - APP_INJURY_CLAW_RELAX_DEG);
}
