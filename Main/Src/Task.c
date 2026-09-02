#include "Task.h"

#include <math.h>

#include "app_config.h"
#include "encoder.h"
#include "Location.h"
#include "main.h"
#include "mechanism.h"
#include "motor.h"
#include "pid.h"
#include "Rescue.h"
#include "servo.h"
#include "vision.h"

typedef enum {
  GRAB_PHASE_APPROACH = 0,
  GRAB_PHASE_OPEN,
  GRAB_PHASE_CLOSE,
  GRAB_PHASE_CONFIRM,
  GRAB_PHASE_RECOVER_CLOSE,
  GRAB_PHASE_RECOVER_BACK
} GrabPhase;

typedef enum {
  START_EXIT = 0,
  START_BRAKE,
  START_CLAW_TOUCH,
  START_TURN,
  START_SCAN_WAIT
} StartStep;

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
static GrabPhase grab_phase;
static StartStep start_step;
static uint8_t inspection_retry_count;
static uint8_t rescue_retry_count;
static bool rescue_active;
static bool task_claw_retracted;
static bool drop_release_complete;
static bool drop_claw_closed;
static bool steering_active;
static FrameConfirmation cargo_confirmation;
static FrameConfirmation candidate_confirmation;
static FrameConfirmation nav_confirmation;
static FrameConfirmation inspection_confirmation;
static uint32_t match_started_ms;
static uint32_t phase_started_ms;
static uint32_t step_started_ms;
static uint32_t inspection_started_ms;
static uint32_t grab_last_valid_ms;
static uint32_t status_sent_ms;
static TaskState status_state;
static TaskFault status_fault;
static int8_t search_direction;
static int32_t search_last_heading_mdeg;
static uint32_t search_rotated_mdeg;
static uint32_t nav_accept_after_ms;
static uint8_t rescue_sequence;
static bool search_heading_valid;
static bool search_advancing;
static bool rescue_sequence_valid;
static int64_t distance_last_count[3];
static float travel_distance_mm;
static Pid_t steering_pid;
static Pid_t camera_pid;
static float camera_angle;
static uint8_t tracking_sequence;
static bool tracking_sequence_valid;
static uint32_t tracking_tick_ms;
static float steering_output_mm_s;

static void task_enter_state(TaskState state, uint32_t now_ms);

static void task_stop(TaskFault fault, uint32_t now_ms)
{
  if (task_status.state == TASK_STOPPED) {
    return;
  }
  task_status.fault = fault;
  task_enter_state(TASK_STOPPED, now_ms);
}

static uint8_t task_total_count(uint8_t counts)
{
  return (uint8_t)(VISION_COUNT_NORMAL(counts) +
                   VISION_COUNT_CORE(counts) +
                   VISION_COUNT_CASUALTY(counts) +
                   VISION_COUNT_DANGER(counts));
}

static void task_publish_status_if_due(uint32_t now_ms)
{
  const bool changed = (task_status.state != status_state) ||
                       (task_status.fault != status_fault);
  if (!changed &&
      ((uint32_t)(now_ms - status_sent_ms) < APP_TASK_STATUS_PERIOD_MS)) {
    return;
  }

  uint8_t flags = 0U;
  flags |= match_started ? VISION_STATUS_MATCH_STARTED : 0U;
  flags |= task_status.found ? VISION_STATUS_FOUND : 0U;
  flags |= task_status.grabbed ? VISION_STATUS_GRABBED : 0U;
  flags |= task_status.cargo_valid ? VISION_STATUS_CARGO_VALID : 0U;
  flags |= task_status.normal_delivered ?
      VISION_STATUS_NORMAL_DELIVERED : 0U;
  flags |= task_status.nav_fresh ? VISION_STATUS_NAV_FRESH : 0U;
  flags |= task_status.near_safe ? VISION_STATUS_NEAR_SAFE : 0U;
  flags |= task_status.claw_empty ? VISION_STATUS_CLAW_EMPTY : 0U;
  const VisionTaskStatus status = {
    .remaining_s = task_status.remaining_s,
    .state = (uint8_t)task_status.state,
    .destination = (uint8_t)task_status.destination,
    .flags = flags,
    .fault = (uint8_t)task_status.fault,
    .recovery_count = task_status.recovery_count,
    .cargo_counts = task_status.cargo_counts
  };
  Vision_QueueTaskStatus(&status);
  status_sent_ms = now_ms;
  status_state = task_status.state;
  status_fault = task_status.fault;
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

static void task_enter_state(TaskState state, uint32_t now_ms)
{
  task_status.state = state;
  phase_started_ms = now_ms;
  step_started_ms = now_ms;
  if (state == TASK_FIND_OBJECT) {
    Servo_SetAngle(1U, 85U);
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
    grab_phase = GRAB_PHASE_APPROACH;
    rescue_retry_count = 0U;
    rescue_active = false;
    rescue_sequence_valid = false;
    search_heading_valid = false;
    search_advancing = false;
    search_rotated_mdeg = 0U;
    task_confirmation_reset(&cargo_confirmation);
    task_confirmation_reset(&candidate_confirmation);
    Camera_SetAngle(90U);
    camera_angle = 90.0f;
    tracking_sequence_valid = false;
    steering_active = false;
    steering_output_mm_s = 0.0f;
    Pid_Reset(&steering_pid);
    Pid_Reset(&camera_pid);
    search_direction = -search_direction;
  } else if (state == TASK_START) {
    Camera_SetAngle(90U);
    camera_angle = 90.0f;
    tracking_sequence_valid = false;
    Pid_Reset(&camera_pid);
    start_step = START_EXIT;
  } else if (state == TASK_GRAB_OBJECT) {
    Pid_Reset(&steering_pid);
    Pid_Reset(&camera_pid);
    grab_phase = GRAB_PHASE_APPROACH;
    grab_last_valid_ms = now_ms;
    tracking_sequence_valid = false;
    steering_active = false;
    steering_output_mm_s = 0.0f;
    task_confirmation_reset(&cargo_confirmation);
    task_confirmation_reset(&candidate_confirmation);
  } else if (state == TASK_RETURN_SAFE) {
    Motor_Stop();
    Camera_SetAngle(90U);
    task_status.nav_fresh = false;
    task_status.near_safe = false;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    nav_accept_after_ms = now_ms;
    task_confirmation_reset(&nav_confirmation);
  } else if (state == TASK_DROP_OBJECT) {
    Motor_Stop();
    Servo_SetAngle(1U, 65U);
    Camera_SetAngle(90U);
    task_status.drop_phase = TASK_DROP_ENTER;
    task_status.nav_fresh = false;
    task_status.near_safe = true;
    task_status.claw_empty = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    inspection_retry_count = 0U;
    drop_release_complete = false;
    drop_claw_closed = false;
    task_confirmation_reset(&inspection_confirmation);
  } else if (state == TASK_STOPPED) {
    Motor_Stop();
  }
}

static void task_initialize(uint32_t now_ms)
{
  Pid_Init(&steering_pid,
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
  Camera_SetAngle(90U);
  Servo_SetAngle(1U, 55U);
  Rescue_Init();
  camera_angle = (float)Camera_GetAngle();
  tracking_sequence = 0U;
  tracking_sequence_valid = false;
  tracking_tick_ms = 0U;
  steering_output_mm_s = 0.0f;
  steering_active = false;
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
  task_status.recovery_count = 0U;
  task_status.drop_phase = TASK_DROP_ENTER;
  task_status.fault = TASK_FAULT_NONE;
  task_status.found = false;
  task_status.grabbed = false;
  task_status.cargo_valid = false;
  task_status.normal_delivered = false;
  task_status.nav_fresh = false;
  task_status.near_safe = false;
  task_status.claw_empty = false;

  match_started = false;
  grab_phase = GRAB_PHASE_APPROACH;
  start_step = START_EXIT;
  task_confirmation_reset(&cargo_confirmation);
  task_confirmation_reset(&candidate_confirmation);
  task_confirmation_reset(&nav_confirmation);
  task_confirmation_reset(&inspection_confirmation);
  match_started_ms = now_ms;
  phase_started_ms = now_ms;
  step_started_ms = now_ms;
  inspection_started_ms = now_ms;
  grab_last_valid_ms = now_ms;
  status_sent_ms = now_ms - APP_TASK_STATUS_PERIOD_MS;
  status_state = TASK_STOPPED;
  status_fault = TASK_FAULT_INVALID_STATE;
  search_direction = 1;
  search_last_heading_mdeg = 0;
  search_rotated_mdeg = 0U;
  nav_accept_after_ms = now_ms;
  rescue_sequence = 0U;
  search_heading_valid = false;
  search_advancing = false;
  rescue_sequence_valid = false;
  for (uint8_t i = 0U; i < 3U; ++i) {
    distance_last_count[i] = 0;
  }
  travel_distance_mm = 0.0f;
  inspection_retry_count = 0U;
  rescue_retry_count = 0U;
  rescue_active = false;
  task_claw_retracted = false;
  drop_release_complete = false;
  drop_claw_closed = false;
  task_initialized = true;
}

static void task_update_match_time(uint32_t now_ms)
{
  if (!match_started) {
    task_status.remaining_s = APP_MATCH_TIME_S;
    return;
  }

  const uint32_t elapsed = (uint32_t)(now_ms - match_started_ms);
  if (elapsed >= APP_MATCH_TIME_MS) {
    task_status.remaining_s = 0U;
    task_stop(TASK_FAULT_MATCH_TIMEOUT, now_ms);
  } else {
    task_status.remaining_s =
        (uint16_t)((APP_MATCH_TIME_MS - elapsed + 999U) / 1000U);
  }
}

static void task_update_travel_distance(void)
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
  const float forward_mm = (wheel_mm[0] - wheel_mm[1]) / 1.7320508f;
  const float lateral_mm =
      (-wheel_mm[0] - wheel_mm[1] + 2.0f * wheel_mm[2]) / 3.0f;
  travel_distance_mm += sqrtf(forward_mm * forward_mm +
                              lateral_mm * lateral_mm);
  task_status.distance_mm =
      (travel_distance_mm >= (float)UINT32_MAX) ? UINT32_MAX :
      (uint32_t)(travel_distance_mm + 0.5f);
}

static float task_vision_dt(uint32_t tick_ms, uint32_t previous_tick_ms,
                            bool previous_valid)
{
  if (!previous_valid) {
    return APP_VISION_PID_DEFAULT_DT_S;
  }

  float dt_s = (float)(uint32_t)(tick_ms - previous_tick_ms) * 0.001f;
  if (dt_s < APP_VISION_PID_MIN_DT_S) {
    dt_s = APP_VISION_PID_MIN_DT_S;
  } else if (dt_s > APP_VISION_PID_MAX_DT_S) {
    dt_s = APP_VISION_PID_MAX_DT_S;
  }
  return dt_s;
}

static float task_update_visual_tracking(const VisionData *vision)
{
  if (tracking_sequence_valid &&
      (vision->sequence == tracking_sequence)) {
    return steering_output_mm_s;
  }
  const float dt_s = task_vision_dt(vision->tick_ms, tracking_tick_ms,
                                    tracking_sequence_valid);
  tracking_sequence = vision->sequence;
  tracking_sequence_valid = true;
  tracking_tick_ms = vision->tick_ms;

  const int32_t camera_error = (int32_t)APP_VISION_TARGET_Y - vision->y;
  if ((camera_error >= -APP_CAMERA_DEAD_ZONE) &&
      (camera_error <= APP_CAMERA_DEAD_ZONE)) {
    Pid_Reset(&camera_pid);
  } else {
    /* Larger servo-3 angles look farther down. A target below image centre
     * therefore needs the opposite sign of the pixel-space PID output. */
    camera_angle -= Pid_UpdateDt(&camera_pid,
                                 (float)APP_VISION_TARGET_Y,
                                 (float)vision->y,
                                 dt_s);
    if (camera_angle < 90.0f) {
      camera_angle = 90.0f;
    } else if (camera_angle > 165.0f) {
      camera_angle = 165.0f;
    }
    Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
  }

  const int32_t steering_error = (int32_t)APP_VISION_TARGET_X - vision->x;
  const int32_t magnitude =
      (steering_error < 0) ? -steering_error : steering_error;
  if ((steering_active &&
       (magnitude <= APP_STEERING_EXIT_DEAD_ZONE)) ||
      (!steering_active &&
       (magnitude <= APP_STEERING_ENTER_DEAD_ZONE))) {
    steering_active = false;
    steering_output_mm_s = 0.0f;
    Pid_Reset(&steering_pid);
    return steering_output_mm_s;
  }
  steering_active = true;
  steering_output_mm_s = Pid_UpdateDt(&steering_pid,
                                      (float)APP_VISION_TARGET_X,
                                      (float)vision->x,
                                      dt_s) * APP_STEERING_DIRECTION;
  if ((steering_output_mm_s > 0.0f) &&
      (steering_output_mm_s < APP_STEERING_MIN_MM_S)) {
    steering_output_mm_s = APP_STEERING_MIN_MM_S;
  } else if ((steering_output_mm_s < 0.0f) &&
             (steering_output_mm_s > -APP_STEERING_MIN_MM_S)) {
    steering_output_mm_s = -APP_STEERING_MIN_MM_S;
  }
  return steering_output_mm_s;
}

static bool task_is_report_fresh(const VisionData *vision, uint32_t now_ms)
{
  return (vision->tick_ms != 0U) &&
         ((uint32_t)(now_ms - vision->tick_ms) <= APP_VISION_TIMEOUT_MS);
}

static bool task_is_candidate_allowed(const VisionData *vision)
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

static bool task_classify_cargo(uint8_t counts, TaskDestination *destination)
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

static bool task_candidate_confirmed(const VisionData *vision)
{
  return task_confirmation_update(&candidate_confirmation,
                                  vision->sequence,
                                  vision->cargo_counts,
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

static bool task_has_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static void task_apply_navigation(uint8_t direction)
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
      (int32_t)(vision->tick_ms - inspection_started_ms) > 0;
  if (!task_is_report_fresh(vision, now_ms) || !vision->valid ||
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

static void task_process_wait_config(const VisionData *vision, uint32_t now_ms)
{
  Motor_Stop();
  if (!vision->config_ready) {
    return;
  }

  task_status.color = vision->color;
  task_status.start_zone = vision->start_zone;
  Location_Reset((LocationStart)vision->start_zone);
  Vision_RequestConfigAck();
  task_enter_state(TASK_START, now_ms);
}

static void task_process_start(uint32_t now_ms)
{
  if (!match_started) {
    Motor_Stop();
    EncoderStatus encoder[3];
    Encoder_GetAll(encoder);
    for (uint8_t i = 0U; i < 3U; ++i) {
      distance_last_count[i] = encoder[i].position;
    }
    travel_distance_mm = 0.0f;
    task_status.distance_mm = 0U;
    match_started_ms = now_ms;
    match_started = true;
    phase_started_ms = now_ms;
    step_started_ms = now_ms;
    start_step = START_EXIT;
  }

  if ((uint32_t)(now_ms - phase_started_ms) >= APP_START_TIMEOUT_MS) {
    task_stop(TASK_FAULT_START_TIMEOUT, now_ms);
    return;
  }

  if (start_step == START_EXIT) {
    if ((uint32_t)(now_ms - step_started_ms) <
        APP_START_REVERSE_TIME_MS) {
      Motor_Move(-APP_START_REVERSE_SPEED_MM_S, 0.0f, 0.0f);
      return;
    }
    Motor_Stop();
    start_step = START_BRAKE;
    step_started_ms = now_ms;
    return;
  }

  if (start_step == START_BRAKE) {
    Motor_Stop();
    if ((uint32_t)(now_ms - step_started_ms) >=
        APP_START_BRAKE_WAIT_MS) {
      Servo_SetAngle(1U, 85U);
      start_step = START_CLAW_TOUCH;
      step_started_ms = now_ms;
    }
    return;
  }

  if (start_step == START_CLAW_TOUCH) {
    Motor_Stop();
    if (Claw_Touch(now_ms)) {
      start_step = START_TURN;
      step_started_ms = now_ms;
    }
    return;
  }

  if (start_step == START_TURN) {
    const MotorTurnStatus result = Motor_TurnAngle(APP_START_TURN_DEG);
    if (result == MOTOR_TURN_DONE) {
      Motor_Stop();
      start_step = START_SCAN_WAIT;
      step_started_ms = now_ms;
    } else if ((result == MOTOR_TURN_FAULT) ||
               (result == MOTOR_TURN_INVALID)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }

  Motor_Stop();
  if ((uint32_t)(now_ms - step_started_ms) >= APP_START_SCAN_WAIT_MS) {
    Mechanism_Init();
    camera_angle = (float)Camera_GetAngle();
    task_enter_state(TASK_FIND_OBJECT, now_ms);
  }
}

static void task_process_search(const VisionData *vision, uint32_t now_ms)
{
  const bool found = Vision_IsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS);
  const bool allowed = found && task_is_candidate_allowed(vision);
  task_status.found = found;
  task_status.grabbed = false;
  task_status.cargo_valid = false;
  task_status.cargo_counts = found ? vision->cargo_counts : 0U;
  task_status.object_count = task_total_count(task_status.cargo_counts);

  if (allowed) {
    Motor_Stop();
    if (task_candidate_confirmed(vision)) {
      task_enter_state(TASK_GRAB_OBJECT, now_ms);
    }
    return;
  }
  task_confirmation_reset(&candidate_confirmation);

  if (search_advancing) {
    const MotorDistanceStatus result =
        Go_distance(APP_SEARCH_ADVANCE_DISTANCE_M,
                    APP_SEARCH_ADVANCE_SPEED_MM_S);
    if (result == MOTOR_DISTANCE_DONE) {
      Motor_Stop();
      search_advancing = false;
      search_heading_valid = false;
      search_rotated_mdeg = 0U;
      step_started_ms = now_ms;
    } else if ((result == MOTOR_DISTANCE_FAULT) ||
               (result == MOTOR_DISTANCE_INVALID)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }

  if ((uint32_t)(now_ms - phase_started_ms) < APP_TARGET_WAIT_MS) {
    Motor_Stop();
    return;
  }

  const LocationPose pose = Location_GetPose();
  if (pose.valid) {
    if (search_heading_valid) {
      int32_t delta = pose.heading_mdeg - search_last_heading_mdeg;
      if (delta > 180000) {
        delta -= 360000;
      } else if (delta < -180000) {
        delta += 360000;
      }
      const uint32_t turn = (uint32_t)((delta < 0) ? -delta : delta);
      if (search_rotated_mdeg <= UINT32_MAX - turn) {
        search_rotated_mdeg += turn;
      }
    } else {
      search_heading_valid = true;
    }
    search_last_heading_mdeg = pose.heading_mdeg;
  }

  if (search_rotated_mdeg >= APP_SEARCH_FULL_TURN_MDEG) {
    Motor_Stop();
    search_advancing = true;
    step_started_ms = now_ms;
    return;
  }

  Motor_Move(0.0f, 0.0f,
             (float)search_direction * APP_SEARCH_ROTATE_SPEED_MM_S);
}

static void task_process_grab(const VisionData *vision, uint32_t now_ms)
{
  const bool report_fresh = task_is_report_fresh(vision, now_ms);
  const bool found = Vision_IsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS);
  const bool grabbed = report_fresh && vision->grabbed;
  task_status.found = found;
  task_status.grabbed = grabbed;
  task_status.cargo_counts = report_fresh ? vision->cargo_counts : 0U;
  task_status.object_count = task_total_count(task_status.cargo_counts);

  if ((grab_phase == GRAB_PHASE_APPROACH) &&
      ((uint32_t)(now_ms - phase_started_ms) >=
       APP_GRAB_APPROACH_TIMEOUT_MS)) {
    Motor_Stop();
    if (task_status.recovery_count < UINT8_MAX) {
      ++task_status.recovery_count;
    }
    grab_phase = GRAB_PHASE_RECOVER_CLOSE;
  }

  if (grab_phase == GRAB_PHASE_RECOVER_CLOSE) {
    Motor_Stop();
    if (Claw_Touch(now_ms)) {
      grab_phase = GRAB_PHASE_RECOVER_BACK;
      step_started_ms = now_ms;
    }
    return;
  }

  if (grab_phase == GRAB_PHASE_RECOVER_BACK) {
    if ((uint32_t)(now_ms - step_started_ms) <
        APP_GRAB_RECOVERY_TIME_MS) {
      Motor_Move(-APP_GRAB_RECOVERY_SPEED_MM_S, 0.0f, 0.0f);
    } else {
      Motor_Stop();
      task_enter_state(TASK_FIND_OBJECT, now_ms);
    }
    return;
  }

  if (grab_phase == GRAB_PHASE_OPEN) {
    Motor_Stop();
    Camera_SetAngle(150U);
    camera_angle = 150.0f;
    if ((uint32_t)(now_ms - step_started_ms) >=
        APP_GRAB_MECHANISM_TIMEOUT_MS) {
      grab_phase = GRAB_PHASE_RECOVER_CLOSE;
      return;
    }
    if (Claw_Open(now_ms)) {
      grab_phase = GRAB_PHASE_CLOSE;
      step_started_ms = now_ms;
    }
    return;
  }

  if (grab_phase == GRAB_PHASE_CLOSE) {
    Motor_Stop();
    Camera_SetAngle(150U);
    camera_angle = 150.0f;
    if ((uint32_t)(now_ms - step_started_ms) >=
        APP_GRAB_MECHANISM_TIMEOUT_MS) {
      grab_phase = GRAB_PHASE_RECOVER_CLOSE;
      return;
    }
    if (Claw_Touch(now_ms)) {
      grab_phase = GRAB_PHASE_CONFIRM;
      step_started_ms = now_ms;
    }
    return;
  }

  if (grab_phase == GRAB_PHASE_CONFIRM) {
    Motor_Stop();
    Camera_SetAngle(150U);
    camera_angle = 150.0f;
    if (grabbed) {
      TaskDestination destination = TASK_DEST_NONE;
      if (!task_cargo_confirmed(vision)) {
        return;
      }

      const bool allowed = vision->classification_valid && !vision->unknown &&
          task_classify_cargo(vision->cargo_counts, &destination);
      if (allowed) {
        task_status.destination = destination;
        task_status.cargo_valid = true;
        task_enter_state(TASK_RETURN_SAFE, now_ms);
      } else {
        task_status.cargo_valid = false;
        task_status.grabbed = false;
        task_confirmation_reset(&cargo_confirmation);
        grab_phase = GRAB_PHASE_RECOVER_CLOSE;
      }
      return;
    }

    if ((uint32_t)(now_ms - step_started_ms) >=
        APP_GRAB_CONFIRM_WAIT_MS) {
      grab_phase = GRAB_PHASE_RECOVER_CLOSE;
    }
    return;
  }
  task_confirmation_reset(&cargo_confirmation);

  if (!found || !task_is_candidate_allowed(vision)) {
    Motor_Stop();
    if ((uint32_t)(now_ms - grab_last_valid_ms) >=
        APP_GRAB_TARGET_LOSS_GRACE_MS) {
      grab_phase = GRAB_PHASE_RECOVER_CLOSE;
    }
    return;
  }
  grab_last_valid_ms = vision->tick_ms;

  const float steering_mm_s = task_update_visual_tracking(vision);
  if (camera_angle >= 150.0f) {
    Motor_Stop();
    Camera_SetAngle(150U);
    camera_angle = 150.0f;
    grab_phase = GRAB_PHASE_OPEN;
    step_started_ms = now_ms;
    return;
  }

  float speed_mm_s = APP_APPROACH_SPEED_MM_S;
  if (vision->distance_mm <= APP_GRAB_SLOW_DISTANCE_MM) {
    speed_mm_s = APP_GRAB_SLOW_SPEED_MM_S;
  } else if (vision->distance_mm <= APP_GRAB_MID_DISTANCE_MM) {
    speed_mm_s = APP_GRAB_MID_SPEED_MM_S;
  }
  Motor_Move(speed_mm_s, 0.0f, steering_mm_s);
}

static void task_process_return(const VisionData *vision, uint32_t now_ms)
{
  if (rescue_active) {
    const RescueStatus rescue = Rescue_Process(now_ms);
    if (rescue == RESCUE_RUNNING) {
      return;
    }
    rescue_active = false;
    Motor_Stop();
    nav_accept_after_ms = now_ms;
    task_confirmation_reset(&nav_confirmation);
    if (rescue != RESCUE_DONE) {
      task_stop(TASK_FAULT_RESCUE, now_ms);
    }
    return;
  }

  const bool new_rescue_command = vision->rescue_requested &&
      ((uint32_t)(now_ms - vision->rescue_tick_ms) <=
       APP_RESCUE_COMMAND_TIMEOUT_MS) &&
      (!rescue_sequence_valid ||
       (vision->rescue_sequence != rescue_sequence));
  if (new_rescue_command) {
    rescue_sequence = vision->rescue_sequence;
    rescue_sequence_valid = true;
    if (rescue_retry_count >= APP_TASK_RESCUE_MAX_RETRIES) {
      task_stop(TASK_FAULT_RESCUE, now_ms);
      return;
    }
    ++rescue_retry_count;
    if (task_status.recovery_count < UINT8_MAX) {
      ++task_status.recovery_count;
    }
    Motor_Stop();
    task_status.nav_fresh = false;
    task_status.nav_direction = VISION_NAV_HOLD;
    task_confirmation_reset(&nav_confirmation);
    Rescue_Start(now_ms);
    rescue_active = true;
    return;
  }

  if ((uint32_t)(now_ms - phase_started_ms) >= APP_RETURN_TIMEOUT_MS) {
    task_stop(TASK_FAULT_RETURN_TIMEOUT, now_ms);
    return;
  }

  const bool nav_matches = vision->nav_valid &&
      (vision->nav_destination == (uint8_t)task_status.destination) &&
      ((int32_t)(vision->nav_tick_ms - nav_accept_after_ms) > 0);
  const bool nav_fresh = Vision_NavIsFresh(vision, now_ms,
                                           APP_NAV_TIMEOUT_MS) &&
      nav_matches;
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
      task_enter_state(TASK_DROP_OBJECT, now_ms);
    }
    return;
  }

  task_confirmation_reset(&nav_confirmation);
  /* Keep executing the last validated command during a short camera or
   * transport dropout instead of repeatedly braking and restarting. */
  task_apply_navigation(task_status.nav_direction);
}

static bool task_distance_failed(MotorDistanceStatus result)
{
  return (result == MOTOR_DISTANCE_FAULT) ||
         (result == MOTOR_DISTANCE_INVALID);
}

static void task_process_drop(const VisionData *vision, uint32_t now_ms)
{
  MotorDistanceStatus result;

  if ((uint32_t)(now_ms - phase_started_ms) >=
      APP_DROP_TOTAL_TIMEOUT_MS) {
    task_stop(TASK_FAULT_DROP_TIMEOUT, now_ms);
    return;
  }

  switch (task_status.drop_phase) {
    case TASK_DROP_ENTER:
      result = Go_distance(APP_DROP_FORWARD_DISTANCE_M,
                           APP_GO_DISTANCE_SPEED_MM_S);
      if (result == MOTOR_DISTANCE_DONE) {
        Motor_Stop();
        task_status.drop_phase = TASK_DROP_RELEASE;
        drop_release_complete = false;
      } else if (task_distance_failed(result)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      break;

    case TASK_DROP_RELEASE:
      Motor_Stop();
      if (!drop_release_complete) {
        if (Claw_Open(now_ms)) {
          drop_release_complete = true;
          step_started_ms = now_ms;
        }
        break;
      }
      if ((uint32_t)(now_ms - step_started_ms) >= APP_DROP_RELEASE_WAIT_MS) {
        Camera_SetAngle(0U);
        task_status.drop_phase = TASK_DROP_CAMERA;
        step_started_ms = now_ms;
      }
      break;

    case TASK_DROP_CAMERA:
      Motor_Stop();
      if ((uint32_t)(now_ms - step_started_ms) >= APP_CAMERA_SETTLE_MS) {
        inspection_started_ms = now_ms;
        task_confirmation_reset(&inspection_confirmation);
        task_status.drop_phase = TASK_DROP_VERIFY;
      }
      break;

    case TASK_DROP_VERIFY: {
      Motor_Stop();
      if ((uint32_t)(now_ms - inspection_started_ms) >=
          APP_DROP_VERIFY_TIMEOUT_MS) {
        task_confirmation_reset(&inspection_confirmation);
        if (inspection_retry_count < APP_DROP_VERIFY_RETRIES) {
          ++inspection_retry_count;
          Camera_SetAngle(90U);
          task_status.drop_phase = TASK_DROP_RELEASE;
          drop_release_complete = false;
        } else {
          task_stop(TASK_FAULT_DROP_VERIFY, now_ms);
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

      Camera_SetAngle(90U);
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
        if (!task_classify_cargo(vision->cargo_counts, &destination)) {
          task_stop(TASK_FAULT_CARGO, now_ms);
          break;
        }
        task_status.destination = destination;
        task_status.drop_phase = TASK_DROP_RETRY_BACK;
        drop_claw_closed = false;
      }
      step_started_ms = now_ms;
      break;
    }

    case TASK_DROP_BACK:
      if (!drop_claw_closed) {
        Motor_Stop();
        if (Claw_Touch(now_ms)) {
          drop_claw_closed = true;
        }
        break;
      }
      result = Go_distance(-APP_DROP_BACK_DISTANCE_M,
                           APP_GO_DISTANCE_SPEED_MM_S);
      if (result == MOTOR_DISTANCE_DONE) {
        Motor_Stop();
        task_enter_state(TASK_FIND_OBJECT, now_ms);
      } else if (task_distance_failed(result)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      break;

    case TASK_DROP_RETRY_BACK:
      if (!drop_claw_closed) {
        Motor_Stop();
        if (Claw_Touch(now_ms)) {
          drop_claw_closed = true;
        }
        break;
      }
      result = Go_distance(-APP_DROP_BACK_DISTANCE_M,
                           APP_GO_DISTANCE_SPEED_MM_S);
      if (result == MOTOR_DISTANCE_DONE) {
        Motor_Stop();
        task_enter_state(TASK_RETURN_SAFE, now_ms);
      } else if (task_distance_failed(result)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      break;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      break;
  }
}

void Task_Process(uint32_t now_ms)
{
  if (!task_initialized) {
    task_initialize(now_ms);
  }

  /* The vehicle must remain stationary until both claws have reached the
   * collision-safe retracted pose (servo 2 first, then servo 4). */
  if (!task_claw_retracted) {
    Motor_Stop();
    if (Claw_Retract(now_ms)) {
      task_claw_retracted = true;
    }
    task_publish_status_if_due(now_ms);
    return;
  }

  const VisionData vision = Vision_GetSnapshot();
  task_update_match_time(now_ms);
  if (match_started) {
    task_update_travel_distance();
  }

  if (vision.stop) {
    task_stop(TASK_FAULT_REMOTE_STOP, now_ms);
  }
  if (task_has_motor_fault()) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  if (task_status.state == TASK_STOPPED) {
    Motor_Stop();
    task_publish_status_if_due(now_ms);
    return;
  }

  switch (task_status.state) {
    case TASK_WAIT_CONFIG:
      task_process_wait_config(&vision, now_ms);
      break;
    case TASK_START:
      task_process_start(now_ms);
      break;
    case TASK_FIND_OBJECT:
      task_process_search(&vision, now_ms);
      break;
    case TASK_GRAB_OBJECT:
      task_process_grab(&vision, now_ms);
      break;
    case TASK_RETURN_SAFE:
      task_process_return(&vision, now_ms);
      break;
    case TASK_DROP_OBJECT:
      task_process_drop(&vision, now_ms);
      break;
    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      break;
  }
  task_publish_status_if_due(now_ms);
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
