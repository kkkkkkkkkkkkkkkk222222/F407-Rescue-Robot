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

static uint32_t servo_travel_wait_ms(uint8_t from_deg, uint8_t to_deg)
{
  const uint32_t travel_deg = from_deg > to_deg ? from_deg - to_deg : to_deg - from_deg;
  /* 130 ms per 60 degrees, times 1.5, rounded up to a whole millisecond. */
  return (travel_deg * 195U + 59U) / 60U;
}

static uint32_t claw_limit_wait_ms(uint8_t id, uint8_t angle, uint8_t min_deg, uint8_t max_deg)
{
  const uint8_t target_deg = Servo_OutputAngle(id, angle);
  const uint32_t from_min_ms = servo_travel_wait_ms(min_deg, target_deg);
  const uint32_t from_max_ms = servo_travel_wait_ms(max_deg, target_deg);
  return from_min_ms > from_max_ms ? from_min_ms : from_max_ms;
}

static bool claw_move_together(ClawAction action, uint32_t now_ms, uint8_t left_angle)
{
  if (claw_action != action) {
    const uint8_t right_angle = APP_CLAW_RIGHT_FROM_LEFT(left_angle);
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

uint32_t Lift_StartPositionWaitMs(void)
{
  return servo_travel_wait_ms(Servo_OutputAngle(1U, Servo_GetAngle(1U)),
                              Servo_OutputAngle(1U, APP_LIFT_START_ANGLE));
}

void Lift_SetTravelPosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_TRAVEL_ANGLE);
}

bool Claw_Open(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_OPEN, now_ms, APP_CLAW_LEFT_OPEN_ANGLE);
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
                              APP_CLAW_LEFT_OPEN_ANGLE, claw_relaxed_right(),
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_OpenRight(uint32_t now_ms)
{
  /* Mirror of Claw_OpenLeft: a smaller left-servo angle closes it farther. */
  return claw_move_separation(CLAW_ACTION_OPEN_RIGHT, now_ms,
                              claw_relaxed_left(), APP_CLAW_RIGHT_OPEN_ANGLE,
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_ClusterOpenLeft(uint32_t now_ms)
{
  /* Selective separation uses the same retained-side profile across retries. */
  return claw_move_separation(CLAW_ACTION_CLUSTER_OPEN_LEFT, now_ms,
                              APP_CLAW_LEFT_OPEN_ANGLE, claw_relaxed_right(),
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_ClusterOpenRight(uint32_t now_ms)
{
  /* Mirror of Claw_ClusterOpenLeft for a retained left-side target. */
  return claw_move_separation(CLAW_ACTION_CLUSTER_OPEN_RIGHT, now_ms,
                              claw_relaxed_left(), APP_CLAW_RIGHT_OPEN_ANGLE,
                              APP_CLAW_SEPARATE_SETTLE_MS);
}

bool Claw_Retract(uint32_t now_ms)
{
  /* Fold the left claw first, then place the right claw on the outside. */
  return claw_move_sequential(CLAW_ACTION_RETRACT, now_ms,
                              4U, APP_CLAW_LEFT_RETRACT_ANGLE,
                              2U, APP_CLAW_RIGHT_RETRACT_ANGLE);
}

bool Claw_Touch(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_TOUCH, now_ms, APP_CLAW_LEFT_TOUCH_ANGLE);
}

bool Claw_HalfTouch(uint32_t now_ms)
{
  return claw_move_together(CLAW_ACTION_HALF_TOUCH, now_ms,
                            APP_CLAW_LEFT_TOUCH_ANGLE + APP_INJURY_CLAW_RELAX_DEG);
}
