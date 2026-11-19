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
  CLAW_ACTION_HALF_TOUCH,
  CLAW_ACTION_FULL_OPEN,
  CLAW_ACTION_BENCH_SEPARATION
} ClawAction;

static ClawAction claw_action;
static uint8_t claw_step;
static uint32_t claw_step_ms;
static uint32_t claw_wait_ms;
static bool claw_done;
static uint8_t separation_left_hold_angle = APP_CLAW_LEFT_TOUCH_ANGLE;

void Claw_RestartAction(void)
{
  claw_action = CLAW_ACTION_NONE;
}

static void claw_set_separation_angle(uint8_t left_angle)
{
  if (left_angle != separation_left_hold_angle) {
    separation_left_hold_angle = left_angle;
    claw_action = CLAW_ACTION_NONE;
  }
}

static uint8_t claw_hold_angle(uint8_t close_deg)
{
  if (close_deg > APP_CLAW_SEPARATE_ONE_OR_TWO_CLOSE_DEG) {
    close_deg = APP_CLAW_SEPARATE_ONE_OR_TWO_CLOSE_DEG;
  }
  return APP_CLAW_LEFT_TOUCH_ANGLE - close_deg;
}

void Claw_SetSeparationHold(uint8_t extra_close_deg)
{
  claw_set_separation_angle(claw_hold_angle(extra_close_deg));
}

void Claw_SetOrangeSeparationHold(uint8_t base_close_deg)
{
  claw_set_separation_angle(claw_hold_angle(base_close_deg) + APP_CLAW_SEPARATE_ORANGE_OPEN_DEG);
}

static uint8_t claw_relaxed_right(void)
{
  return APP_CLAW_RIGHT_FROM_LEFT(separation_left_hold_angle);
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
  separation_left_hold_angle = APP_CLAW_LEFT_TOUCH_ANGLE;
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

uint32_t Camera_PositionWaitMs(uint8_t angle)
{
  return servo_pulse_wait_ms(Servo_OutputPulseUs(3U, Servo_GetAngle(3U)),
                            Servo_OutputPulseUs(3U, angle));
}

void Lift_SetStartPosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_START_ANGLE);
}

void Lift_SetReleaseRaisePosition(void)
{
  Servo_SetAngle(1U, APP_LIFT_RELEASE_RAISE_ANGLE);
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

uint8_t Lift_LoadedTravelAngle(LiftLoad load)
{
  return load == LIFT_LOAD_MATERIAL ? APP_LIFT_MATERIAL_TRAVEL_ANGLE :
      load == LIFT_LOAD_ORANGE ? APP_LIFT_ORANGE_TRAVEL_ANGLE : APP_LIFT_TRAVEL_ANGLE;
}

void Lift_SetLoadedTravelPosition(LiftLoad load)
{
  Servo_SetAngle(1U, Lift_LoadedTravelAngle(load));
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

bool Claw_FullOpen(uint32_t now_ms)
{
  if (claw_action != CLAW_ACTION_FULL_OPEN && claw_done &&
      Servo_GetAngle(4U) == APP_CLAW_LEFT_RELEASE_ANGLE &&
      Servo_GetAngle(2U) == APP_CLAW_RIGHT_RELEASE_ANGLE) {
    /* A completed pose need not start another actuator wait. */
    return true;
  }
  return claw_move_together(CLAW_ACTION_FULL_OPEN, now_ms,
                            APP_CLAW_LEFT_RELEASE_ANGLE, APP_CLAW_RIGHT_RELEASE_ANGLE);
}

bool Claw_OpenIfNeeded(uint32_t now_ms)
{
  if (claw_done && Servo_GetAngle(4U) == APP_CLAW_LEFT_OPEN_ANGLE &&
      Servo_GetAngle(2U) == APP_CLAW_RIGHT_OPEN_ANGLE) {
    return true;
  }
  return Claw_Open(now_ms);
}

static int claw_next_position(int current, int target, int step)
{
  int delta = target - current;
  if (delta > step) { delta = step; }
  if (delta < -step) { delta = -step; }
  return current + delta;
}

static bool claw_step_to(uint8_t id, uint16_t target)
{
  const int current = Servo_GetAngle(id);
  if (current == target) { return true; }
  const uint8_t next = (uint8_t)claw_next_position(current, target, APP_CLAW_SEPARATE_STEP_DEG);
  Servo_SetAngle(id, next);
  return next == target;
}

static bool claw_move_steps(ClawAction action, uint32_t now_ms, uint16_t left_target,
                            uint16_t right_target, uint32_t settle_ms,
                            bool (*step_to)(uint8_t, uint16_t))
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
    const bool left_at_target = step_to(4U, left_target);
    const bool right_at_target = step_to(2U, right_target);
    claw_step_ms = now_ms;
    if (left_at_target && right_at_target) { claw_step = 2U; }
  }
  if (claw_step == 2U && (uint32_t)(now_ms - claw_step_ms) >= settle_ms) {
    claw_done = true;
  }
  return claw_done;
}

static bool claw_move_separation(ClawAction action, uint32_t now_ms,
                                 uint8_t left_angle, uint8_t right_angle, uint32_t settle_ms)
{
  return claw_move_steps(action, now_ms, left_angle, right_angle, settle_ms, claw_step_to);
}

#if APP_BOARD_F405_ROBO
typedef struct {
  int opening_cdeg, target_cdeg, trim_us;
} BenchClaw;

static BenchClaw bench_left, bench_right;
static uint16_t bench_close_step_cdeg;

static int bench_clamp(int value, int low, int high)
{
  return value < low ? low : (value > high ? high : value);
}

/* The S4 and S2 logical axes are mirrored. The mounted-PWM and opening
 * axes are also mirrors of each other on exactly one side. */
static int bench_mount(uint8_t id, int angle_cdeg)
{
  return ((id == 4U) == (APP_CLAW_SERVOS_INVERTED != 0)) ? 18000 - angle_cdeg : angle_cdeg;
}

static int bench_deg_trim(uint8_t id)
{
  return (id == 4U ? APP_SERVO_S4_OFFSET_DEG : APP_SERVO_S2_OFFSET_DEG) * 100;
}

static int bench_pulse(uint8_t id, int opening, int trim_us)
{
  const int mounted = bench_mount(id, opening);
  if (opening == 18000) { return 500 + mounted / 9; }
  return bench_clamp(500 + bench_clamp(mounted + bench_deg_trim(id), 0, 18000) / 9 + trim_us, 500, 2500);
}

static int bench_start_opening(uint8_t id, int pulse, int trim_us)
{
  const int logical = Servo_GetAngle(id);
  const int cached = (id == 4U ? logical : 180 - logical) * 100;
  if (bench_pulse(id, cached, trim_us) == pulse) { return cached; }
  if (bench_pulse(id, 18000, trim_us) == pulse) { return 18000; }
  /* SWD fine tuning can have changed the DMA pulse since the last S command. */
  const int opening = bench_clamp(bench_mount(id, (pulse - 500 - trim_us) * 9 - bench_deg_trim(id)), 0, 18000);
  const int whole = ((opening + 50) / 100) * 100;
  return bench_pulse(id, whole, trim_us) == pulse ? whole : opening;
}

bool Claw_BeginBenchSeparation(bool keep_left, uint16_t retained_cdeg,
                               int16_t left_trim_us, int16_t right_trim_us, uint16_t close_dps)
{
  const int left = Servo_GetEnabledPulseUs(4U), right = Servo_GetEnabledPulseUs(2U);
  if (left < 500 || left > 2500 || right < 500 || right > 2500) { return false; }
  bench_left = (BenchClaw){bench_start_opening(4U, left, left_trim_us),
      keep_left ? retained_cdeg : 18000, left_trim_us};
  bench_right = (BenchClaw){bench_start_opening(2U, right, right_trim_us),
      keep_left ? 18000 : retained_cdeg, right_trim_us};
  bench_close_step_cdeg = (uint16_t)(close_dps * APP_TASK_PERIOD_MS / 10U);
  Claw_RestartAction();
  return true;
}

static bool bench_step_to(uint8_t id, uint16_t target)
{
  BenchClaw *claw = id == 4U ? &bench_left : &bench_right;
  const int step = target < claw->opening_cdeg ? bench_close_step_cdeg : APP_CLAW_SEPARATE_STEP_DEG * 100;
  const int opening = claw_next_position(claw->opening_cdeg, target, step);
  const uint8_t deg = (uint8_t)((opening + 50) / 100);
  Servo_SetCalibratedPulse(id, id == 4U ? deg : 180U - deg, bench_pulse(id, opening, claw->trim_us));
  claw->opening_cdeg = opening;
  return opening == target;
}

bool Claw_RunBenchSeparation(uint32_t now_ms)
{
  return claw_move_steps(CLAW_ACTION_BENCH_SEPARATION, now_ms, bench_left.target_cdeg,
                         bench_right.target_cdeg, APP_CLAW_SEPARATE_SETTLE_MS, bench_step_to);
}
#endif

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
                              separation_left_hold_angle, APP_CLAW_RIGHT_RELEASE_ANGLE,
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
                              separation_left_hold_angle, APP_CLAW_RIGHT_RELEASE_ANGLE,
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
