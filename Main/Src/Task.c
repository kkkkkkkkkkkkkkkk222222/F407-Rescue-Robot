#include "Task.h"

#include <math.h>

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

typedef struct {
  uint16_t signature;
  uint8_t sequence;
  uint8_t streak;
  bool valid;
} FrameConfirmation;

static volatile TaskStatus task_status;
static bool task_initialized;
static bool match_started;
static bool key_sample;
static bool key_stable;
static uint8_t crab_step;
static uint8_t inspection_retry_count;
static FrameConfirmation cargo_confirmation;
static FrameConfirmation nav_confirmation;
static FrameConfirmation inspection_confirmation;
static uint32_t match_tick;
static uint32_t state_tick;
static uint32_t key_tick;
static uint32_t inspection_tick;
static int64_t distance_last_count[3];
static float travel_distance_mm;
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

static void task_confirmation_reset(FrameConfirmation *confirmation)
{
  confirmation->signature = 0U;
  confirmation->sequence = 0U;
  confirmation->streak = 0U;
  confirmation->valid = false;
}

static bool task_confirmation_update(FrameConfirmation *confirmation,
                                     uint8_t sequence,
                                     uint16_t signature,
                                     uint8_t required_frames)
{
  if (!confirmation->valid) {
    confirmation->signature = signature;
    confirmation->sequence = sequence;
    confirmation->streak = 1U;
    confirmation->valid = true;
  } else if (sequence == confirmation->sequence) {
    return false;
  } else if ((sequence != (uint8_t)(confirmation->sequence + 1U)) ||
             (signature != confirmation->signature)) {
    confirmation->signature = signature;
    confirmation->sequence = sequence;
    confirmation->streak = 1U;
  } else {
    confirmation->sequence = sequence;
    if (confirmation->streak < required_frames) {
      ++confirmation->streak;
    }
  }
  return confirmation->streak >= required_frames;
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
    task_confirmation_reset(&cargo_confirmation);
    Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
  } else if (state == TASK_CRAB_OBJECT) {
    Pid_Reset(&steering_pid);
    Pid_Reset(&camera_pid);
    crab_step = CRAB_APPROACH;
    task_confirmation_reset(&cargo_confirmation);
  } else if (state == TASK_RETURN_SAFE) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
    task_status.nav_fresh = false;
    task_status.near_safe = false;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    task_confirmation_reset(&nav_confirmation);
  } else if (state == TASK_DROP_OBJECT) {
    Motor_Stop();
    Claw_SetAngle(APP_CLAW_CLOSE_ANGLE);
    Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
    task_status.drop_phase = TASK_DROP_ENTER;
    task_status.nav_fresh = false;
    task_status.near_safe = true;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    inspection_retry_count = 0U;
    task_confirmation_reset(&inspection_confirmation);
  } else if (state == TASK_STOPPED) {
    Motor_Stop();
  }
}

static void task_initialize(uint32_t now_ms)
{
  Pid_Init(&steering_pid,
           APP_STEERING_KP_MM_S, 0.0f, APP_STEERING_KD_MM_S,
           -APP_STEERING_LIMIT_MM_S, APP_STEERING_LIMIT_MM_S,
           -1000.0f, 1000.0f);
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
  task_confirmation_reset(&cargo_confirmation);
  task_confirmation_reset(&nav_confirmation);
  task_confirmation_reset(&inspection_confirmation);
  match_tick = now_ms;
  state_tick = now_ms;
  inspection_tick = now_ms;
  for (uint8_t i = 0U; i < 3U; ++i) {
    distance_last_count[i] = 0;
  }
  travel_distance_mm = 0.0f;
  inspection_retry_count = 0U;
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
  EncoderStatus encoder[3];
  Encoder_GetAll(encoder);
  const float millimetres_per_count =
      3.14159265358979323846f * (float)APP_WHEEL_DIAMETER_MM /
      (float)APP_ENCODER_COUNTS_PER_WHEEL_REV;
  float wheel_mm[3];
  const int8_t signs[3] = {
    APP_OMNI_M1_ENCODER_SIGN,
    APP_OMNI_M2_ENCODER_SIGN,
    APP_OMNI_M3_ENCODER_SIGN
  };
  for (uint8_t i = 0U; i < 3U; ++i) {
    wheel_mm[i] = (float)(encoder[i].position - distance_last_count[i]) *
                  (float)signs[i] * millimetres_per_count;
    distance_last_count[i] = encoder[i].position;
  }
  const float forward_mm = (wheel_mm[1] - wheel_mm[2]) / 1.7320508f;
  const float lateral_mm =
      (-2.0f * wheel_mm[0] + wheel_mm[1] + wheel_mm[2]) / 3.0f;
  travel_distance_mm += sqrtf(forward_mm * forward_mm +
                              lateral_mm * lateral_mm);
  task_status.distance_mm =
      (travel_distance_mm >= (float)UINT32_MAX) ? UINT32_MAX :
      (uint32_t)(travel_distance_mm + 0.5f);
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

static float task_turn(const VisionData *vision)
{
  const int32_t error = (int32_t)APP_VISION_TARGET_X - vision->x;
  if ((error >= -APP_STEERING_DEAD_ZONE) &&
      (error <= APP_STEERING_DEAD_ZONE)) {
    Pid_Reset(&steering_pid);
    return 0;
  }
  return Pid_Update(&steering_pid,
                    (float)APP_VISION_TARGET_X,
                    (float)vision->x) * APP_STEERING_DIRECTION;
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
      vision->claw_view || vision->grabbed ||
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
  return task_confirmation_update(&cargo_confirmation,
                                  vision->sequence,
                                  signature,
                                  APP_CARGO_CONFIRM_FRAMES);
}

static bool task_nav_near_confirmed(const VisionData *vision)
{
  return task_confirmation_update(&nav_confirmation,
                                  vision->nav_sequence,
                                  0U,
                                  APP_NAV_CONFIRM_FRAMES);
}

static bool task_inspection_confirmed(uint8_t result, uint8_t sequence)
{
  return task_confirmation_update(&inspection_confirmation,
                                  sequence,
                                  result,
                                  APP_DROP_CONFIRM_FRAMES);
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
      Motor_Move(APP_RETURN_FORWARD_SPEED_MM_S, 0.0f, 0.0f);
      break;
    case VISION_NAV_TURN_LEFT:
      Motor_Move(0.0f, 0.0f, APP_RETURN_TURN_SPEED_MM_S);
      break;
    case VISION_NAV_TURN_RIGHT:
      Motor_Move(0.0f, 0.0f, -APP_RETURN_TURN_SPEED_MM_S);
      break;
    case VISION_NAV_BACKWARD:
      Motor_Move(-APP_RETURN_BACKWARD_SPEED_MM_S, 0.0f, 0.0f);
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

    EncoderStatus encoder[3];
    Encoder_GetAll(encoder);
    for (uint8_t i = 0U; i < 3U; ++i) {
      distance_last_count[i] = encoder[i].position;
    }
    travel_distance_mm = 0.0f;
    task_status.distance_mm = 0U;
    match_tick = now_ms;
    match_started = true;
  }

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
    Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
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
      Motor_Move(-APP_CRAB_ADJUST_SPEED_MM_S, 0.0f, 0.0f);
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
      task_confirmation_reset(&cargo_confirmation);
      crab_step = CRAB_BACK;
      state_tick = now_ms;
    }
    return;
  }
  task_confirmation_reset(&cargo_confirmation);

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

  float speed_mm_s = APP_APPROACH_SPEED_MM_S;
  if (vision->distance_mm <= APP_CRAB_SLOW_DISTANCE_MM) {
    speed_mm_s = APP_CRAB_SLOW_SPEED_MM_S;
  } else if (vision->distance_mm <= APP_CRAB_MID_DISTANCE_MM) {
    speed_mm_s = APP_CRAB_MID_SPEED_MM_S;
  }
  Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
  Motor_Move(speed_mm_s, 0.0f, task_turn(vision));
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
    task_confirmation_reset(&nav_confirmation);
    return;
  }

  if (task_status.near_safe) {
    Motor_Stop();
    if (task_nav_near_confirmed(vision)) {
      task_set_state(TASK_DROP_OBJECT, now_ms);
    }
    return;
  }

  task_confirmation_reset(&nav_confirmation);
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
        task_confirmation_reset(&inspection_confirmation);
        task_status.drop_phase = TASK_DROP_VERIFY;
      }
      break;

    case TASK_DROP_VERIFY: {
      Motor_Stop();
      Claw_SetAngle(APP_CLAW_OPEN_ANGLE);
      if ((uint32_t)(now_ms - inspection_tick) >=
          APP_DROP_VERIFY_TIMEOUT_MS) {
        task_confirmation_reset(&inspection_confirmation);
        if (inspection_retry_count < APP_DROP_VERIFY_RETRIES) {
          ++inspection_retry_count;
          Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
          task_status.drop_phase = TASK_DROP_RELEASE;
          state_tick = now_ms;
        } else {
          task_set_state(TASK_STOPPED, now_ms);
        }
        break;
      }

      const uint8_t inspection = task_inspection_result(vision, now_ms);
      if (inspection == INSPECTION_NONE) {
        task_confirmation_reset(&inspection_confirmation);
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
