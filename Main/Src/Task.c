#include "Task.h"

#include "app_config.h"
#include "encoder.h"
#include "main.h"
#include "mechanism.h"
#include "motor.h"
#include "pid.h"
#include "vision.h"

#define CRAB_APPROACH 0U
#define CRAB_CLOSE    1U
#define CRAB_BACK     2U

#define INSPECTION_NONE   0U
#define INSPECTION_EMPTY  1U
#define INSPECTION_LOADED 2U

static volatile TaskStatus task_status;
static bool task_initialized;
static bool match_started;
static bool key_sample;
static bool key_stable;
static uint8_t crab_step;
static uint8_t cargo_confirm_streak;
static uint8_t cargo_confirm_sequence;
static uint8_t nav_confirm_streak;
static uint8_t nav_confirm_sequence;
static uint8_t inspection_confirm_streak;
static uint8_t inspection_confirm_sequence;
static uint8_t inspection_confirm_result;
static uint16_t cargo_confirm_signature;
static bool cargo_confirm_sequence_valid;
static bool nav_confirm_sequence_valid;
static bool inspection_confirm_sequence_valid;
static uint32_t match_tick;
static uint32_t state_tick;
static uint32_t key_tick;
static uint32_t inspection_tick;
static int64_t start_m1;
static int64_t start_m2;
static Pid_t steering_pid;
static Pid_t camera_pid;
static float camera_angle;

static uint8_t task_total_count(uint8_t counts)
{
  return (uint8_t)(VISION_COUNT_NORMAL(counts) +
                   VISION_COUNT_CORE(counts) +
                   VISION_COUNT_CASUALTY(counts) +
                   VISION_COUNT_DANGER(counts));
}

static void task_reset_cargo_confirmation(void)
{
  cargo_confirm_streak = 0U;
  cargo_confirm_sequence = 0U;
  cargo_confirm_signature = 0U;
  cargo_confirm_sequence_valid = false;
}

static void task_reset_nav_confirmation(void)
{
  nav_confirm_streak = 0U;
  nav_confirm_sequence = 0U;
  nav_confirm_sequence_valid = false;
}

static void task_reset_inspection_confirmation(void)
{
  inspection_confirm_streak = 0U;
  inspection_confirm_sequence = 0U;
  inspection_confirm_result = INSPECTION_NONE;
  inspection_confirm_sequence_valid = false;
}

static void task_set_state(TaskState state, uint32_t now_ms)
{
  task_status.state = state;
  state_tick = now_ms;
  if (state == TASK_FIND_OBJECT) {
    task_status.destination = TASK_DEST_NONE;
    task_status.cargo_counts = 0U;
    task_status.object_count = 0U;
    task_status.found = false;
    task_status.grabbed = false;
    task_status.cargo_valid = false;
    task_status.nav_fresh = false;
    task_status.near_safe = false;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    crab_step = CRAB_APPROACH;
    task_reset_cargo_confirmation();
    Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
  } else if (state == TASK_CRAB_OBJECT) {
    Pid_Reset(&steering_pid);
    Pid_Reset(&camera_pid);
    crab_step = CRAB_APPROACH;
    task_reset_cargo_confirmation();
  } else if (state == TASK_RETURN_SAFE) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
    task_status.nav_fresh = false;
    task_status.near_safe = false;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    task_reset_nav_confirmation();
  } else if (state == TASK_DROP_OBJECT) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
    task_status.drop_phase = TASK_DROP_ENTER;
    task_status.nav_fresh = false;
    task_status.near_safe = true;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    task_reset_inspection_confirmation();
  } else if (state == TASK_STOPPED) {
    Motor_Stop();
  }
}

static void task_initialize(uint32_t now_ms)
{
  Pid_Init(&steering_pid, 3.0f, 0.0f, 2.0f,
           -320.0f, 320.0f, -1000.0f, 1000.0f);
  Pid_Init(&camera_pid, 0.5f, 0.0f, 0.3f,
           -3.0f, 3.0f, -1000.0f, 1000.0f);
  Mechanism_Init();
  Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
  camera_angle = (float)Camera_GetAngle();
  Motor_Stop();

  task_status.state = TASK_WAIT_CONFIG;
  task_status.destination = TASK_DEST_NONE;
  task_status.remaining_s = APP_MATCH_TIME_S;
  task_status.distance_mm = 0U;
  task_status.color = 0U;
  task_status.start_zone = 0U;
  task_status.cargo_counts = 0U;
  task_status.object_count = 0U;
  task_status.nav_direction = VISION_NAV_HOLD;
  task_status.drop_phase = TASK_DROP_ENTER;
  task_status.found = false;
  task_status.grabbed = false;
  task_status.cargo_valid = false;
  task_status.normal_delivered = false;
  task_status.nav_fresh = false;
  task_status.near_safe = false;
  task_status.claw_empty = false;

  match_started = false;
  key_sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;
  key_stable = key_sample;
  key_tick = now_ms;
  crab_step = CRAB_APPROACH;
  task_reset_cargo_confirmation();
  task_reset_nav_confirmation();
  task_reset_inspection_confirmation();
  match_tick = now_ms;
  state_tick = now_ms;
  inspection_tick = now_ms;
  start_m1 = 0;
  start_m2 = 0;
  task_initialized = true;
}

static bool task_key_pressed(uint32_t now_ms)
{
  const bool sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;

  if (sample != key_sample) {
    key_sample = sample;
    key_tick = now_ms;
  }
  if ((sample == key_stable) ||
      ((uint32_t)(now_ms - key_tick) < APP_MOTOR_KEY_DEBOUNCE_MS)) {
    return false;
  }
  key_stable = sample;
  return key_stable;
}

static void task_update_time(uint32_t now_ms)
{
  if (!match_started) {
    task_status.remaining_s = APP_MATCH_TIME_S;
    return;
  }

  const uint32_t elapsed = (uint32_t)(now_ms - match_tick);
  if (elapsed >= APP_MATCH_TIME_MS) {
    task_status.remaining_s = 0U;
    task_set_state(TASK_STOPPED, now_ms);
  } else {
    task_status.remaining_s =
        (uint16_t)((APP_MATCH_TIME_MS - elapsed + 999U) / 1000U);
  }
}

static void task_update_distance(void)
{
  const EncoderStatus m1 = Encoder_GetStatus(1U);
  const EncoderStatus m2 = Encoder_GetStatus(2U);
  const float millimetres_per_count =
      3.14159265358979323846f * (float)APP_WHEEL_DIAMETER_MM /
      (float)APP_ENCODER_COUNTS_PER_WHEEL_REV;
  const float m1_mm = (float)(m1.position - start_m1) *
                      (float)APP_OMNI_M1_ENCODER_SIGN * millimetres_per_count;
  const float m2_mm = (float)(m2.position - start_m2) *
                      (float)APP_OMNI_M2_ENCODER_SIGN * millimetres_per_count;
  float distance = (m2_mm - m1_mm) / 1.7320508f;

  if (distance < 0.0f) {
    distance = 0.0f;
  } else if (distance > 65535.0f) {
    distance = 65535.0f;
  }
  task_status.distance_mm = (uint16_t)(distance + 0.5f);
}

static void task_track_camera(const VisionData *vision)
{
  const float change = Pid_Update(&camera_pid,
                                  (float)APP_VISION_TARGET_Y,
                                  (float)vision->y) * APP_CAMERA_DIRECTION;
  camera_angle += change;
  if (camera_angle < (float)APP_CAMERA_MIN_ANGLE) {
    camera_angle = (float)APP_CAMERA_MIN_ANGLE;
  } else if (camera_angle > (float)APP_CAMERA_MAX_ANGLE) {
    camera_angle = (float)APP_CAMERA_MAX_ANGLE;
  }
  Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
}

static int16_t task_turn(const VisionData *vision)
{
  const int32_t error = (int32_t)APP_VISION_TARGET_X - vision->x;
  if ((error >= -APP_STEERING_DEAD_ZONE) &&
      (error <= APP_STEERING_DEAD_ZONE)) {
    Pid_Reset(&steering_pid);
    return 0;
  }
  return (int16_t)(Pid_Update(&steering_pid,
                              (float)APP_VISION_TARGET_X,
                              (float)vision->x) * APP_STEERING_DIRECTION);
}

static bool task_report_fresh(const VisionData *vision, uint32_t now_ms)
{
  return (vision->tick_ms != 0U) &&
         ((uint32_t)(now_ms - vision->tick_ms) <= APP_VISION_TIMEOUT_MS);
}

static bool task_candidate_allowed(const VisionData *vision)
{
  const uint8_t counts = vision->cargo_counts;
  if (!vision->classification_valid || vision->unknown ||
      (task_total_count(counts) != 1U) ||
      (VISION_COUNT_DANGER(counts) != 0U)) {
    return false;
  }

  if (!task_status.normal_delivered) {
    return VISION_COUNT_NORMAL(counts) == 1U;
  }
  return true;
}

static bool task_cargo_allowed(uint8_t counts, TaskDestination *destination)
{
  const uint8_t normal = VISION_COUNT_NORMAL(counts);
  const uint8_t core = VISION_COUNT_CORE(counts);
  const uint8_t casualty = VISION_COUNT_CASUALTY(counts);
  const uint8_t danger = VISION_COUNT_DANGER(counts);
  const uint8_t total = task_total_count(counts);

  *destination = TASK_DEST_NONE;
  if ((danger != 0U) || (total == 0U) || (total > 3U)) {
    return false;
  }

  if (!task_status.normal_delivered) {
    if ((normal == total) && (core == 0U) && (casualty == 0U)) {
      *destination = TASK_DEST_MATERIAL;
      return true;
    }
    return false;
  }

  if (casualty != 0U) {
    if ((casualty == 1U) && (normal == 0U) && (core == 0U) &&
        (total == 1U)) {
      *destination = TASK_DEST_CASUALTY;
      return true;
    }
    return false;
  }

  if ((uint8_t)(normal + core) == total) {
    *destination = TASK_DEST_MATERIAL;
    return true;
  }
  return false;
}

static bool task_cargo_confirmed(const VisionData *vision)
{
  const uint16_t signature = (uint16_t)vision->cargo_counts |
      (vision->classification_valid ? 0x0100U : 0U) |
      (vision->unknown ? 0x0200U : 0U);

  if (!cargo_confirm_sequence_valid) {
    cargo_confirm_sequence = vision->sequence;
    cargo_confirm_signature = signature;
    cargo_confirm_streak = 1U;
    cargo_confirm_sequence_valid = true;
    return false;
  }
  if (vision->sequence == cargo_confirm_sequence) {
    return false;
  }
  if (vision->sequence != (uint8_t)(cargo_confirm_sequence + 1U)) {
    cargo_confirm_sequence = vision->sequence;
    cargo_confirm_signature = signature;
    cargo_confirm_streak = 1U;
    return false;
  }
  cargo_confirm_sequence = vision->sequence;

  if (signature == cargo_confirm_signature) {
    if (cargo_confirm_streak < APP_CARGO_CONFIRM_FRAMES) {
      ++cargo_confirm_streak;
    }
  } else {
    cargo_confirm_signature = signature;
    cargo_confirm_streak = 1U;
  }
  return cargo_confirm_streak >= APP_CARGO_CONFIRM_FRAMES;
}

static bool task_nav_near_confirmed(const VisionData *vision)
{
  if (!nav_confirm_sequence_valid) {
    nav_confirm_sequence = vision->nav_sequence;
    nav_confirm_streak = 1U;
    nav_confirm_sequence_valid = true;
    return APP_NAV_CONFIRM_FRAMES <= 1U;
  }
  if (vision->nav_sequence == nav_confirm_sequence) {
    return false;
  }
  if (vision->nav_sequence != (uint8_t)(nav_confirm_sequence + 1U)) {
    nav_confirm_sequence = vision->nav_sequence;
    nav_confirm_streak = 1U;
    return false;
  }

  nav_confirm_sequence = vision->nav_sequence;
  if (nav_confirm_streak < APP_NAV_CONFIRM_FRAMES) {
    ++nav_confirm_streak;
  }
  return nav_confirm_streak >= APP_NAV_CONFIRM_FRAMES;
}

static bool task_inspection_confirmed(uint8_t result, uint8_t sequence)
{
  if (!inspection_confirm_sequence_valid) {
    inspection_confirm_result = result;
    inspection_confirm_sequence = sequence;
    inspection_confirm_streak = 1U;
    inspection_confirm_sequence_valid = true;
    return APP_DROP_CONFIRM_FRAMES <= 1U;
  }
  if (sequence == inspection_confirm_sequence) {
    return false;
  }
  if ((sequence != (uint8_t)(inspection_confirm_sequence + 1U)) ||
      (result != inspection_confirm_result)) {
    inspection_confirm_result = result;
    inspection_confirm_sequence = sequence;
    inspection_confirm_streak = 1U;
    return false;
  }

  inspection_confirm_sequence = sequence;
  if (inspection_confirm_streak < APP_DROP_CONFIRM_FRAMES) {
    ++inspection_confirm_streak;
  }
  return inspection_confirm_streak >= APP_DROP_CONFIRM_FRAMES;
}

static bool task_motor_has_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static void task_apply_nav_direction(uint8_t direction)
{
  switch (direction) {
    case VISION_NAV_FORWARD:
      Motor_Move(APP_RETURN_FORWARD_SPEED, 0, 0);
      break;
    case VISION_NAV_TURN_LEFT:
      Motor_Move(0, 0, APP_RETURN_TURN_SPEED);
      break;
    case VISION_NAV_TURN_RIGHT:
      Motor_Move(0, 0, -APP_RETURN_TURN_SPEED);
      break;
    case VISION_NAV_BACKWARD:
      Motor_Move(-APP_RETURN_BACKWARD_SPEED, 0, 0);
      break;
    default:
      Motor_Stop();
      break;
  }
}

static uint8_t task_inspection_result(const VisionData *vision,
                                      uint32_t now_ms)
{
  const bool received_after_camera =
      (int32_t)(vision->tick_ms - inspection_tick) > 0;
  if (!task_report_fresh(vision, now_ms) || !vision->valid ||
      !vision->claw_view || !received_after_camera ||
      !vision->classification_valid || vision->unknown) {
    return INSPECTION_NONE;
  }
  if (!vision->found && !vision->grabbed &&
      (vision->cargo_counts == 0U)) {
    return INSPECTION_EMPTY;
  }
  if (vision->found && (vision->cargo_counts != 0U)) {
    return INSPECTION_LOADED;
  }
  return INSPECTION_NONE;
}

static void Wait_Config(const VisionData *vision, uint32_t now_ms)
{
  Motor_Stop();
  if (!vision->config_ready) {
    return;
  }

  task_status.color = vision->color;
  task_status.start_zone = vision->start_zone;
  Vision_RequestConfigAck();
  task_set_state(TASK_START, now_ms);
}

static void Start(uint32_t now_ms)
{
  if (!match_started) {
    Motor_Stop();
    if (!task_key_pressed(now_ms)) {
      return;
    }

    const EncoderStatus m1 = Encoder_GetStatus(1U);
    const EncoderStatus m2 = Encoder_GetStatus(2U);
    start_m1 = m1.position;
    start_m2 = m2.position;
    match_tick = now_ms;
    match_started = true;
  }

  task_update_distance();
  const MotorDistanceStatus result = Go_distance(APP_INITIAL_FORWARD_DISTANCE_M);
  if (result == MOTOR_DISTANCE_DONE) {
    Motor_Stop();
    Mechanism_Init();
    camera_angle = (float)Camera_GetAngle();
    task_set_state(TASK_FIND_OBJECT, now_ms);
  } else if ((result == MOTOR_DISTANCE_FAULT) ||
             (result == MOTOR_DISTANCE_INVALID)) {
    task_set_state(TASK_STOPPED, now_ms);
  }
}

static void Find_Object(const VisionData *vision, uint32_t now_ms)
{
  const bool found = Vision_IsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS);
  const bool allowed = found && task_candidate_allowed(vision);
  task_status.found = found;
  task_status.grabbed = false;
  task_status.cargo_valid = false;
  task_status.cargo_counts = found ? vision->cargo_counts : 0U;
  task_status.object_count = task_total_count(task_status.cargo_counts);

  if (allowed) {
    Motor_Stop();
    task_set_state(TASK_CRAB_OBJECT, now_ms);
  } else if ((uint32_t)(now_ms - state_tick) < APP_TARGET_WAIT_MS) {
    Motor_Stop();
  } else {
    Motor_Move(0, 0, APP_SEARCH_ROTATE_SPEED);
  }
}

static void Crab_Object(const VisionData *vision, uint32_t now_ms)
{
  const bool report_fresh = task_report_fresh(vision, now_ms);
  const bool found = Vision_IsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS);
  const bool grabbed = report_fresh && vision->grabbed;
  task_status.found = found;
  task_status.grabbed = grabbed;
  task_status.cargo_counts = report_fresh ? vision->cargo_counts : 0U;
  task_status.object_count = task_total_count(task_status.cargo_counts);

  if (crab_step == CRAB_BACK) {
    if ((uint32_t)(now_ms - state_tick) < APP_CRAB_BACK_MS) {
      Motor_Move(-APP_CRAB_ADJUST_SPEED, 0, 0);
    } else {
      Motor_Stop();
      task_set_state(TASK_FIND_OBJECT, now_ms);
    }
    return;
  }

  if (grabbed) {
    TaskDestination destination = TASK_DEST_NONE;
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    if (!task_cargo_confirmed(vision)) {
      return;
    }

    const bool allowed = vision->classification_valid && !vision->unknown &&
                         task_cargo_allowed(vision->cargo_counts, &destination);
    if (allowed) {
      task_status.destination = destination;
      task_status.cargo_valid = true;
      task_set_state(TASK_RETURN_SAFE, now_ms);
    } else {
      task_status.cargo_valid = false;
      task_status.grabbed = false;
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      task_reset_cargo_confirmation();
      crab_step = CRAB_BACK;
      state_tick = now_ms;
    }
    return;
  }
  task_reset_cargo_confirmation();

  if (crab_step == CRAB_CLOSE) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    if ((uint32_t)(now_ms - state_tick) >= APP_CRAB_CLOSE_WAIT_MS) {
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      crab_step = CRAB_BACK;
      state_tick = now_ms;
    }
    return;
  }

  if (!found) {
    Motor_Stop();
    task_set_state(TASK_FIND_OBJECT, now_ms);
    return;
  }
  if (!task_candidate_allowed(vision)) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
    crab_step = CRAB_BACK;
    state_tick = now_ms;
    return;
  }

  task_track_camera(vision);
  const bool distance_valid =
      vision->distance_mm >= APP_VISION_MIN_DISTANCE_MM;
  if (distance_valid &&
      (vision->near ||
       (vision->distance_mm <= APP_CRAB_STOP_DISTANCE_MM))) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    crab_step = CRAB_CLOSE;
    state_tick = now_ms;
    return;
  }

  int16_t speed = APP_APPROACH_SPEED;
  if (vision->distance_mm <= APP_CRAB_SLOW_DISTANCE_MM) {
    speed = APP_CRAB_SLOW_SPEED;
  } else if (vision->distance_mm <= APP_CRAB_MID_DISTANCE_MM) {
    speed = APP_CRAB_MID_SPEED;
  }
  Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
  Motor_Move(speed, 0, task_turn(vision));
}

static void Return_Safe(const VisionData *vision, uint32_t now_ms)
{
  Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
  const bool nav_fresh = Vision_NavIsFresh(vision, now_ms,
                                            APP_NAV_TIMEOUT_MS) &&
      (vision->nav_destination == (uint8_t)task_status.destination) &&
      ((int32_t)(vision->nav_tick_ms - state_tick) > 0);
  task_status.nav_fresh = nav_fresh;
  task_status.nav_direction = nav_fresh ? vision->nav_direction :
                                                VISION_NAV_HOLD;
  task_status.near_safe = nav_fresh &&
      (vision->nav_zone_state == VISION_NAV_NEAR_SAFE);

  if (!nav_fresh) {
    Motor_Stop();
    task_reset_nav_confirmation();
    return;
  }

  if (task_status.near_safe) {
    Motor_Stop();
    if (task_nav_near_confirmed(vision)) {
      task_set_state(TASK_DROP_OBJECT, now_ms);
    }
    return;
  }

  task_reset_nav_confirmation();
  task_apply_nav_direction(vision->nav_direction);
}

static bool task_distance_failed(MotorDistanceStatus result)
{
  return (result == MOTOR_DISTANCE_FAULT) ||
         (result == MOTOR_DISTANCE_INVALID);
}

static void Drop_Object(const VisionData *vision, uint32_t now_ms)
{
  MotorDistanceStatus result;

  switch (task_status.drop_phase) {
    case TASK_DROP_ENTER:
      Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
      result = Go_distance(APP_DROP_FORWARD_DISTANCE_M);
      if (result == MOTOR_DISTANCE_DONE) {
        Motor_Stop();
        Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
        task_status.drop_phase = TASK_DROP_RELEASE;
        state_tick = now_ms;
      } else if (task_distance_failed(result)) {
        task_set_state(TASK_STOPPED, now_ms);
      }
      break;

    case TASK_DROP_RELEASE:
      Motor_Stop();
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      if ((uint32_t)(now_ms - state_tick) >= APP_DROP_RELEASE_WAIT_MS) {
        Camera_SetAngle(APP_CAMERA_CHECK_ANGLE);
        task_status.drop_phase = TASK_DROP_CAMERA;
        state_tick = now_ms;
      }
      break;

    case TASK_DROP_CAMERA:
      Motor_Stop();
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      if ((uint32_t)(now_ms - state_tick) >= APP_CAMERA_SETTLE_MS) {
        inspection_tick = now_ms;
        task_reset_inspection_confirmation();
        task_status.drop_phase = TASK_DROP_VERIFY;
      }
      break;

    case TASK_DROP_VERIFY: {
      Motor_Stop();
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      const uint8_t inspection = task_inspection_result(vision, now_ms);
      if (inspection == INSPECTION_NONE) {
        task_reset_inspection_confirmation();
        break;
      }
      if (!task_inspection_confirmed(inspection, vision->sequence)) {
        break;
      }

      Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
      Motor_Stop();
      if (inspection == INSPECTION_EMPTY) {
        task_status.claw_empty = true;
        if (VISION_COUNT_NORMAL(task_status.cargo_counts) > 0U) {
          task_status.normal_delivered = true;
        }
        task_status.drop_phase = TASK_DROP_BACK;
      } else {
        TaskDestination destination = TASK_DEST_NONE;
        task_status.claw_empty = false;
        task_status.cargo_counts = vision->cargo_counts;
        task_status.object_count = task_total_count(vision->cargo_counts);
        if (!task_cargo_allowed(vision->cargo_counts, &destination)) {
          task_set_state(TASK_STOPPED, now_ms);
          break;
        }
        task_status.destination = destination;
        Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
        task_status.drop_phase = TASK_DROP_RETRY_BACK;
      }
      state_tick = now_ms;
      break;
    }

    case TASK_DROP_BACK:
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      result = Go_distance(-APP_DROP_BACK_DISTANCE_M);
      if (result == MOTOR_DISTANCE_DONE) {
        Motor_Stop();
        task_set_state(TASK_FIND_OBJECT, now_ms);
      } else if (task_distance_failed(result)) {
        task_set_state(TASK_STOPPED, now_ms);
      }
      break;

    case TASK_DROP_RETRY_BACK:
      Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
      result = Go_distance(-APP_DROP_BACK_DISTANCE_M);
      if (result == MOTOR_DISTANCE_DONE) {
        Motor_Stop();
        task_set_state(TASK_RETURN_SAFE, now_ms);
      } else if (task_distance_failed(result)) {
        task_set_state(TASK_STOPPED, now_ms);
      }
      break;

    default:
      task_set_state(TASK_STOPPED, now_ms);
      break;
  }
}

void Task_FindObject(uint32_t now_ms)
{
  if (!task_initialized) {
    task_initialize(now_ms);
  }

  const VisionData vision = Vision_GetSnapshot();
  task_update_time(now_ms);
  if (match_started) {
    task_update_distance();
  }

  if (vision.stop) {
    task_set_state(TASK_STOPPED, now_ms);
  }
  if (task_motor_has_fault()) {
    task_set_state(TASK_STOPPED, now_ms);
  }
  if (task_status.state == TASK_STOPPED) {
    Motor_Stop();
    return;
  }

  switch (task_status.state) {
    case TASK_WAIT_CONFIG:
      Wait_Config(&vision, now_ms);
      break;
    case TASK_START:
      Start(now_ms);
      break;
    case TASK_FIND_OBJECT:
      Find_Object(&vision, now_ms);
      break;
    case TASK_CRAB_OBJECT:
      Crab_Object(&vision, now_ms);
      break;
    case TASK_RETURN_SAFE:
      Return_Safe(&vision, now_ms);
      break;
    case TASK_DROP_OBJECT:
      Drop_Object(&vision, now_ms);
      break;
    default:
      task_set_state(TASK_STOPPED, now_ms);
      break;
  }
}

TaskStatus Task_GetStatus(void)
{
  TaskStatus status;
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  status = task_status;
  if (primask == 0U) {
    __enable_irq();
  }
  return status;
}
