#include "Task.h"

#include <math.h>

#include "app_config.h"
#include "encoder.h"
#include "Location.h"
#include "main.h"
#include "mechanism.h"
#include "motor.h"
#include "pid.h"
#include "vision.h"

typedef enum {
  START_EXIT = 0,
  START_BRAKE,
  START_TOUCH_CLAW,
  START_TURN,
  START_SETTLE
} StartStep;

typedef struct {
  int32_t last_heading_mdeg;
  uint32_t accumulated_mdeg;
  bool valid;
} TurnTracker;

typedef struct {
  float x_mm;
  float y_mm;
  float heading_deg;
  bool inside_field;
} TaskPose;

typedef struct {
  float distance_mm;
  uint32_t tick_ms;
  int16_t target_x_mm;
  int16_t target_y_mm;
  bool valid;
} NavProgress;

static volatile TaskStatus task_status;
static Pid_t steering_pid;
static Pid_t camera_pid;
static TaskState state;
static StartStep start_step;
static TurnTracker turn_tracker;
static NavProgress nav_progress;
static uint32_t state_started_ms;
static uint32_t step_started_ms;
static uint32_t match_started_ms;
static uint32_t status_sent_ms;
static uint32_t command_hold_until_ms;
static float camera_angle;
static float steering_mm_s;
static uint32_t tracking_tick_ms;
static uint8_t tracking_sequence;
static uint8_t mission_sequence;
static uint32_t start_reverse_path_mm;
static uint32_t search_entry_report_generation;
static bool initialized;
static bool initial_claw_ready;
static bool match_started;
static bool tracking_valid;
static bool mission_sequence_valid;
static bool search_advancing;
static bool steering_active;
static bool nav_ready;
static bool search_report_gate_open;
static bool approach_target_lost;
static int8_t recover_dir;

static void task_enter(TaskState next, uint32_t now_ms);
static bool distance_failed(MotorDistanceStatus result);
static bool fused_pose_ready(const VisionFusedPose *pose, uint32_t now_ms);

static float task_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float task_wrap_angle(float angle_deg)
{
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static bool task_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static void task_stop(TaskFault fault, uint32_t now_ms)
{
  task_status.fault = fault;
  task_enter(TASK_STOPPED, now_ms);
}

static void task_reset_tracking(void)
{
  tracking_valid = false;
  steering_active = false;
  steering_mm_s = 0.0f;
  Pid_Reset(&steering_pid);
  Pid_Reset(&camera_pid);
}

static void task_reset_turn_tracker(void)
{
  turn_tracker.last_heading_mdeg = 0;
  turn_tracker.accumulated_mdeg = 0U;
  turn_tracker.valid = false;
}

static bool task_full_turn_reached(void)
{
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    return false;
  }
  if (turn_tracker.valid) {
    int32_t delta = pose.heading_mdeg - turn_tracker.last_heading_mdeg;
    if (delta > 180000) {
      delta -= 360000;
    } else if (delta < -180000) {
      delta += 360000;
    }
    const uint32_t magnitude = (uint32_t)((delta < 0) ? -delta : delta);
    if (turn_tracker.accumulated_mdeg <= UINT32_MAX - magnitude) {
      turn_tracker.accumulated_mdeg += magnitude;
    }
  } else {
    turn_tracker.valid = true;
  }
  turn_tracker.last_heading_mdeg = pose.heading_mdeg;
  return turn_tracker.accumulated_mdeg >= APP_SEARCH_FULL_TURN_MDEG;
}

static void task_publish_status(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - status_sent_ms) < APP_TASK_STATUS_PERIOD_MS) {
    return;
  }

  uint8_t flags = 0U;
  flags |= task_status.claw_visible ? VISION_STM_CLAW_VISIBLE : 0U;
  flags |= task_status.gripper_closed ? VISION_STM_GRIPPER_CLOSED : 0U;
  flags |= task_status.motors_active ? VISION_STM_MOTORS_ACTIVE : 0U;
  flags |= task_status.auto_approach ? VISION_STM_AUTO_APPROACH : 0U;
  flags |= (task_status.fault != TASK_FAULT_NONE) ? VISION_STM_FAULT : 0U;

  const VisionStmStatus status = {
    .camera_pitch_cdeg = (uint16_t)Camera_GetAngle() * 100U,
    .flags = flags,
    .mode = (uint8_t)state,
    .acknowledged_sequence = task_status.acknowledged_sequence,
    .fault_code = (uint8_t)task_status.fault
  };
  Vision_QueueStmStatus(&status);
  status_sent_ms = now_ms;
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

static void task_enter(TaskState next, uint32_t now_ms)
{
  Motor_Stop();
  state = next;
  task_status.state = next;
  task_status.motors_active = false;
  task_status.auto_approach = false;
  task_status.claw_visible =
      (next >= TASK_GRAB_OBSERVE) && (next <= TASK_CLOSE_CLAW);
  state_started_ms = now_ms;
  step_started_ms = now_ms;

  if (next == TASK_START) {
    start_step = START_EXIT;
    start_reverse_path_mm = Location_GetPose().path_mm;
  } else if (next == TASK_SEARCH) {
    const VisionData vision = Vision_GetSnapshot();
    Camera_SetAngle(APP_SEARCH_CAMERA_ANGLE);
    camera_angle = (float)APP_SEARCH_CAMERA_ANGLE;
    task_status.found = false;
    search_advancing = false;
    search_entry_report_generation = vision.report_generation;
    search_report_gate_open = false;
    task_reset_tracking();
    task_reset_turn_tracker();
  } else if (next == TASK_APPROACH) {
    camera_angle = (float)Camera_GetAngle();
    approach_target_lost = false;
    task_reset_tracking();
  } else if (next == TASK_APPROACH_RECOVER) {
    approach_target_lost = true;
    task_reset_tracking();
    task_reset_turn_tracker();
  } else if (next == TASK_GRAB_OBSERVE) {
    task_reset_tracking();
  } else if (next == TASK_GRAB_ROTATE) {
    task_reset_turn_tracker();
  } else if ((next == TASK_SCATTER_POSITIVE) ||
             (next == TASK_SCATTER_NEGATIVE)) {
    task_reset_turn_tracker();
  } else if (next == TASK_WAIT_NAVIGATION) {
    Camera_SetAngle(90U);
    camera_angle = 90.0f;
  } else if ((next == TASK_NAVIGATE) ||
             (next == TASK_ALIGN_SAFE_ZONE) ||
             (next == TASK_FACE_FIELD_CENTER)) {
    nav_ready = false;
    nav_progress.valid = false;
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
           -APP_CENTERING_CAMERA_STEP_LIMIT_DEG,
           APP_CENTERING_CAMERA_STEP_LIMIT_DEG,
           -APP_CAMERA_INTEGRAL_LIMIT_PX_S,
           APP_CAMERA_INTEGRAL_LIMIT_PX_S);

  Mechanism_Init();
  Lift_SetStartPosition();
  task_status = (TaskStatus){0};
  task_status.remaining_s = APP_MATCH_TIME_S;
  state = TASK_WAIT_CONFIG;
  task_status.state = state;
  state_started_ms = now_ms;
  step_started_ms = now_ms;
  match_started_ms = now_ms;
  status_sent_ms = now_ms - APP_TASK_STATUS_PERIOD_MS;
  command_hold_until_ms = now_ms;
  camera_angle = (float)Camera_GetAngle();
  steering_mm_s = 0.0f;
  tracking_sequence = 0U;
  tracking_tick_ms = 0U;
  mission_sequence = 0U;
  start_reverse_path_mm = 0U;
  search_entry_report_generation = 0U;
  initial_claw_ready = false;
  match_started = false;
  tracking_valid = false;
  mission_sequence_valid = false;
  search_advancing = false;
  steering_active = false;
  nav_ready = false;
  search_report_gate_open = false;
  approach_target_lost = false;
  recover_dir = 1;
  nav_progress = (NavProgress){0};
  task_reset_turn_tracker();
  initialized = true;
}

static bool task_target_is_normal(const VisionData *vision, uint32_t now_ms)
{
  return Vision_IsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS) &&
         vision->classification_valid &&
         (VISION_COUNT_NORMAL(vision->cargo_counts) == 1U) &&
         (VISION_COUNT_CORE(vision->cargo_counts) == 0U) &&
         (VISION_COUNT_CASUALTY(vision->cargo_counts) == 0U) &&
         (VISION_COUNT_DANGER(vision->cargo_counts) == 0U);
}

static float task_tracking_dt(uint32_t tick_ms)
{
  if (!tracking_valid) {
    return APP_VISION_PID_DEFAULT_DT_S;
  }
  float dt_s = (float)(uint32_t)(tick_ms - tracking_tick_ms) * 0.001f;
  if (dt_s < APP_VISION_PID_MIN_DT_S) {
    dt_s = APP_VISION_PID_MIN_DT_S;
  } else if (dt_s > APP_VISION_PID_MAX_DT_S) {
    dt_s = APP_VISION_PID_MAX_DT_S;
  }
  return dt_s;
}

static float task_track_target(const VisionData *vision)
{
  if (tracking_valid && (vision->sequence == tracking_sequence)) {
    return steering_mm_s;
  }
  const float dt_s = task_tracking_dt(vision->tick_ms);
  tracking_sequence = vision->sequence;
  tracking_tick_ms = vision->tick_ms;
  tracking_valid = true;

  const int32_t y_error = (int32_t)APP_VISION_TARGET_Y - vision->y;
  if ((y_error >= -APP_CAMERA_DEAD_ZONE) &&
      (y_error <= APP_CAMERA_DEAD_ZONE)) {
    Pid_Reset(&camera_pid);
  } else {
    camera_angle -= Pid_UpdateDt(&camera_pid,
                                 (float)APP_VISION_TARGET_Y,
                                 (float)vision->y, dt_s);
    /* This is the camera direction and range used by the verified 14:48
     * approach controller: larger servo-3 angles look farther down. */
    if (camera_angle < 90.0f) {
      camera_angle = 90.0f;
    } else if (camera_angle > 165.0f) {
      camera_angle = 165.0f;
    }
    Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
  }

  const int32_t x_error = (int32_t)APP_VISION_TARGET_X - vision->x;
  const int32_t magnitude = (x_error < 0) ? -x_error : x_error;
  if ((steering_active &&
       (magnitude <= APP_STEERING_EXIT_DEAD_ZONE)) ||
      (!steering_active &&
       (magnitude <= APP_STEERING_ENTER_DEAD_ZONE))) {
    steering_active = false;
    steering_mm_s = 0.0f;
    Pid_Reset(&steering_pid);
    return steering_mm_s;
  }

  steering_active = true;
  steering_mm_s = Pid_UpdateDt(&steering_pid,
                               (float)APP_VISION_TARGET_X,
                               (float)vision->x, dt_s) *
                  APP_STEERING_DIRECTION;
  if ((steering_mm_s > 0.0f) &&
      (steering_mm_s < APP_STEERING_MIN_MM_S)) {
    steering_mm_s = APP_STEERING_MIN_MM_S;
  } else if ((steering_mm_s < 0.0f) &&
             (steering_mm_s > -APP_STEERING_MIN_MM_S)) {
    steering_mm_s = -APP_STEERING_MIN_MM_S;
  }
  return steering_mm_s;
}

static void task_process_start(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - state_started_ms) >= APP_START_TIMEOUT_MS) {
    task_stop(TASK_FAULT_START_TIMEOUT, now_ms);
    return;
  }

  if (start_step == START_EXIT) {
    const LocationPose pose = Location_GetPose();
    if (!pose.valid) {
      task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
      return;
    }
    const uint32_t travelled_mm = pose.path_mm - start_reverse_path_mm;
    if (travelled_mm + APP_START_REVERSE_TOLERANCE_MM >=
        APP_START_REVERSE_DISTANCE_MM) {
      Motor_Stop();
      task_status.motors_active = false;
      start_step = START_BRAKE;
      step_started_ms = now_ms;
    } else {
      const uint32_t remaining_mm =
          APP_START_REVERSE_DISTANCE_MM - travelled_mm;
      float speed_mm_s = APP_START_REVERSE_SPEED_MM_S;
      if (remaining_mm <= APP_START_REVERSE_SLOW_REMAINING_MM) {
        speed_mm_s = APP_START_REVERSE_SLOW_SPEED_MM_S;
      } else if (remaining_mm <= APP_START_REVERSE_MID_REMAINING_MM) {
        speed_mm_s = APP_START_REVERSE_MID_SPEED_MM_S;
      }
      /* 180 degrees means body-backward. Motor_MoveAngle records the current
       * IMU yaw when this segment starts and continuously holds that yaw. */
      (void)Motor_MoveAngle(speed_mm_s, 180.0f);
      task_status.motors_active = true;
    }
  } else if (start_step == START_BRAKE) {
    if ((uint32_t)(now_ms - step_started_ms) >= APP_START_BRAKE_WAIT_MS) {
      Lift_SetTravelPosition();
      start_step = START_TOUCH_CLAW;
      step_started_ms = now_ms;
    }
  } else if (start_step == START_TOUCH_CLAW) {
    if (Claw_Touch(now_ms)) {
      start_step = START_TURN;
      step_started_ms = now_ms;
    }
  } else if (start_step == START_TURN) {
    const MotorTurnStatus result = Motor_TurnAngle(APP_START_TURN_DEG);
    task_status.motors_active = result == MOTOR_TURN_RUNNING;
    if (result == MOTOR_TURN_DONE) {
      Motor_Stop();
      task_status.motors_active = false;
      start_step = START_SETTLE;
      step_started_ms = now_ms;
    } else if ((result == MOTOR_TURN_FAULT) ||
               (result == MOTOR_TURN_INVALID)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
  } else if ((uint32_t)(now_ms - step_started_ms) >=
             APP_START_SCAN_WAIT_MS) {
    task_enter(TASK_OPEN_CLAW, now_ms);
  }
}

static void task_process_pile_approach(uint32_t now_ms)
{
  const MotorDistanceStatus result =
      Motor_MoveDistance(APP_PILE_APPROACH_DISTANCE_M,
                         APP_PILE_APPROACH_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_SCATTER_POSITIVE, now_ms);
  } else if (distance_failed(result)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
}

static void task_process_scatter(float speed_mm_s, TaskState next,
                                 uint32_t now_ms)
{
  if (!Location_GetPose().valid) {
    task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_SCATTER_TURN_TIMEOUT_MS) {
    task_stop(TASK_FAULT_START_TIMEOUT, now_ms);
    return;
  }
  if (task_full_turn_reached()) {
    task_enter(next, now_ms);
    return;
  }
  Motor_Move(0.0f, 0.0f, speed_mm_s);
  task_status.motors_active = true;
}

static void task_process_scatter_pause(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_SCATTER_BRAKE_WAIT_MS) {
    task_enter(TASK_SCATTER_NEGATIVE, now_ms);
  }
}

static void task_process_scatter_exit(uint32_t now_ms)
{
  /* Give the chassis a short stationary interval before changing from a
   * 500 mm/s spin to translation in the opposite body direction. */
  if ((uint32_t)(now_ms - state_started_ms) <
      APP_SCATTER_BRAKE_WAIT_MS) {
    return;
  }
  const MotorDistanceStatus result =
      Motor_MoveDistance(-APP_SCATTER_EXIT_DISTANCE_M,
                         APP_SCATTER_EXIT_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_SEARCH, now_ms);
  } else if (distance_failed(result)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
}

static bool read_field_pose(const VisionData *vision, uint32_t now_ms,
                            TaskPose *pose)
{
  if (fused_pose_ready(&vision->fused_pose, now_ms)) {
    pose->x_mm = (float)vision->fused_pose.x_mm;
    pose->y_mm = (float)vision->fused_pose.y_mm;
    pose->heading_deg = (float)vision->fused_pose.heading_cdeg * 0.01f;
    pose->inside_field =
        (task_abs(pose->x_mm) <= APP_LOCATION_FIELD_HALF_MM) &&
        (task_abs(pose->y_mm) <= APP_LOCATION_FIELD_HALF_MM);
    return true;
  }

  const LocationPose local = Location_GetPose();
  if (!local.valid) {
    return false;
  }
  pose->x_mm = (float)local.x_mm;
  pose->y_mm = (float)local.y_mm;
  pose->heading_deg = (float)local.heading_mdeg * 0.001f;
  pose->inside_field = local.inside_field;
  return true;
}

static bool search_path_safe(const VisionData *vision, uint32_t now_ms)
{
  TaskPose pose;
  if (!read_field_pose(vision, now_ms, &pose) || !pose.inside_field) {
    return false;
  }

  const float heading_rad = pose.heading_deg * 0.01745329252f;
  const float distance_mm = APP_SEARCH_ADVANCE_DISTANCE_M * 1000.0f;
  const float end_x = pose.x_mm + cosf(heading_rad) * distance_mm;
  const float end_y = pose.y_mm + sinf(heading_rad) * distance_mm;
  const float limit = APP_LOCATION_FIELD_HALF_MM -
                      APP_SEARCH_FIELD_MARGIN_MM;
  return (task_abs(end_x) <= limit) && (task_abs(end_y) <= limit);
}

static void task_process_search(const VisionData *vision, uint32_t now_ms)
{
  if (!search_report_gate_open &&
      (vision->report_generation != search_entry_report_generation) &&
      Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    search_report_gate_open = true;
  }

  /* Reports received before this SEARCH epoch may still be younger than the
   * normal 250 ms timeout. They remain visible on the LCD, but cannot select
   * a target or enter APPROACH until a new valid report is received. */
  task_status.found = search_report_gate_open &&
                      task_target_is_normal(vision, now_ms);
  if (task_status.found) {
    task_enter(TASK_APPROACH, now_ms);
    return;
  }

  if (search_advancing) {
    const MotorDistanceStatus result =
        Motor_MoveDistance(APP_SEARCH_ADVANCE_DISTANCE_M,
                           APP_SEARCH_ADVANCE_SPEED_MM_S);
    task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
    if (result == MOTOR_DISTANCE_DONE) {
      task_enter(TASK_SEARCH, now_ms);
    } else if ((result == MOTOR_DISTANCE_FAULT) ||
               (result == MOTOR_DISTANCE_INVALID)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }

  if ((uint32_t)(now_ms - state_started_ms) < APP_TARGET_WAIT_MS) {
    return;
  }
  if (task_full_turn_reached()) {
    Motor_Stop();
    task_status.motors_active = false;
    if (search_path_safe(vision, now_ms)) {
      search_advancing = true;
    } else {
      task_enter(TASK_FACE_FIELD_CENTER, now_ms);
    }
    return;
  }
  Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
  task_status.motors_active = true;
}

static void task_process_approach(const VisionData *vision, uint32_t now_ms)
{
  const bool found = task_target_is_normal(vision, now_ms);
  task_status.found = found;
  task_status.auto_approach = true;
  if (!found) {
    Motor_Stop();
    task_status.motors_active = false;
    if (Camera_GetAngle() >= APP_GRAB_LOSS_FORCE_ANGLE_MIN) {
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      camera_angle = (float)APP_GRAB_VIEW_ANGLE;
      task_enter(TASK_GRAB_OBSERVE, now_ms);
    } else if (!approach_target_lost) {
      approach_target_lost = true;
      step_started_ms = now_ms;
    } else if ((uint32_t)(now_ms - step_started_ms) >=
               APP_APPROACH_LOSS_HOLD_MS) {
      task_enter(TASK_APPROACH_RECOVER, now_ms);
    }
    return;
  }

  approach_target_lost = false;

  const float turn = task_track_target(vision);
  if (turn > 0.0f) {
    recover_dir = 1;
  } else if (turn < 0.0f) {
    recover_dir = -1;
  }
  if (camera_angle >= (float)APP_GRAB_VIEW_ANGLE) {
    Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
    camera_angle = (float)APP_GRAB_VIEW_ANGLE;
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
  float speed = APP_APPROACH_SPEED_MM_S;
  if (vision->distance_valid) {
    if (vision->distance_mm <= APP_GRAB_SLOW_DISTANCE_MM) {
      speed = APP_GRAB_SLOW_SPEED_MM_S;
    } else if (vision->distance_mm <= APP_GRAB_MID_DISTANCE_MM) {
      speed = APP_GRAB_MID_SPEED_MM_S;
    }
  }
  Motor_Move(speed, 0.0f, turn);
  task_status.motors_active = true;
}

static void task_raise_camera(void)
{
  uint8_t angle = Camera_GetAngle();
  if (angle > (APP_GRAB_CAMERA_MIN_ANGLE +
               APP_GRAB_CAMERA_RAISE_STEP_DEG)) {
    angle = (uint8_t)(angle - APP_GRAB_CAMERA_RAISE_STEP_DEG);
  } else {
    angle = APP_GRAB_CAMERA_MIN_ANGLE;
  }
  Camera_SetAngle(angle);
  camera_angle = (float)angle;
}

static void task_process_approach_recover(const VisionData *vision,
                                          uint32_t now_ms)
{
  task_status.auto_approach = true;
  task_status.found = task_target_is_normal(vision, now_ms);
  if (task_status.found) {
    task_enter(TASK_APPROACH, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_APPROACH_RECOVERY_TIMEOUT_MS) {
    task_stop(TASK_FAULT_TARGET_LOST, now_ms);
    return;
  }
  if (!Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  if (!Location_GetPose().valid) {
    task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - step_started_ms) <
      APP_APPROACH_RECOVERY_SETTLE_MS) {
    return;
  }
  if (task_full_turn_reached()) {
    Motor_Stop();
    task_status.motors_active = false;
    if (Camera_GetAngle() <= APP_GRAB_CAMERA_MIN_ANGLE) {
      task_stop(TASK_FAULT_TARGET_LOST, now_ms);
      return;
    }
    task_raise_camera();
    recover_dir = (int8_t)-recover_dir;
    task_reset_turn_tracker();
    step_started_ms = now_ms;
    return;
  }
  Motor_Move(0.0f, 0.0f,
             APP_APPROACH_RECOVERY_ROTATE_MM_S *
             (float)recover_dir);
  task_status.motors_active = true;
}

static void task_process_grab_observe(const VisionData *vision,
                                      uint32_t now_ms)
{
  task_status.found = Vision_IsFresh(vision, now_ms,
                                     APP_VISION_TIMEOUT_MS);
  if (!Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS) ||
      task_status.found) {
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_GRAB_INITIAL_OBSERVE_MS) {
    task_raise_camera();
    task_enter(TASK_GRAB_RAISE_WAIT, now_ms);
  }
}

static void task_process_grab_raise_wait(const VisionData *vision,
                                         uint32_t now_ms)
{
  task_status.found = Vision_IsFresh(vision, now_ms,
                                     APP_VISION_TIMEOUT_MS);
  if (task_status.found) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
  } else if (Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS) &&
             ((uint32_t)(now_ms - state_started_ms) >=
              APP_GRAB_RAISE_OBSERVE_MS)) {
    task_enter(TASK_GRAB_ROTATE, now_ms);
  }
}

static void task_process_grab_rotate(const VisionData *vision,
                                     uint32_t now_ms)
{
  task_status.found = Vision_IsFresh(vision, now_ms,
                                     APP_VISION_TIMEOUT_MS);
  if (task_status.found) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
  if (!Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  if (task_full_turn_reached()) {
    task_raise_camera();
    task_enter(TASK_GRAB_RAISE_WAIT, now_ms);
    return;
  }
  Motor_Move(0.0f, 0.0f, APP_GRAB_SCAN_ROTATE_MM_S);
  task_status.motors_active = true;
}

static bool fused_pose_ready(const VisionFusedPose *pose, uint32_t now_ms)
{
  return Vision_FusedPoseIsFresh(pose, now_ms,
                                 APP_FUSED_POSE_TIMEOUT_MS) &&
         ((pose->status & VISION_POSE_VALID) != 0U) &&
         ((pose->status & VISION_POSE_T265_GOOD) != 0U) &&
         ((pose->status & VISION_POSE_T265_UPDATE_REJECTED) == 0U);
}

static float bearing_to(float target_x_mm, float target_y_mm,
                        float x_mm, float y_mm)
{
  float angle_deg = atan2f(target_y_mm - y_mm,
                           target_x_mm - x_mm) * 57.2957795f;
  if (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static bool turn_to(float desired_deg, float current_deg, uint32_t now_ms)
{
  const float error_deg = task_wrap_angle(desired_deg - current_deg);
  if (task_abs(error_deg) <= APP_NAV_HEADING_TOLERANCE_DEG) {
    Motor_Stop();
    nav_ready = true;
    step_started_ms = now_ms;
    return true;
  }

  const MotorTurnStatus result = Motor_TurnAngle(error_deg);
  task_status.motors_active = result == MOTOR_TURN_RUNNING;
  if (result == MOTOR_TURN_DONE) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = true;
    step_started_ms = now_ms;
    return true;
  }
  if ((result == MOTOR_TURN_FAULT) || (result == MOTOR_TURN_INVALID)) {
    return false;
  }
  return true;
}

static float distance_to_target(const VisionMissionCommand *command,
                                const VisionFusedPose *pose)
{
  const float dx = (float)command->target_x_mm - (float)pose->x_mm;
  const float dy = (float)command->target_y_mm - (float)pose->y_mm;
  return sqrtf(dx * dx + dy * dy);
}

static bool navigation_progress_ok(const VisionMissionCommand *command,
                                   float distance_mm, uint32_t now_ms)
{
  const int32_t target_dx =
      (int32_t)nav_progress.target_x_mm - command->target_x_mm;
  const int32_t target_dy =
      (int32_t)nav_progress.target_y_mm - command->target_y_mm;
  const bool target_changed = nav_progress.valid &&
      ((task_abs((float)target_dx) >= APP_NAV_TARGET_CHANGE_MM) ||
       (task_abs((float)target_dy) >= APP_NAV_TARGET_CHANGE_MM));
  if (!nav_progress.valid || target_changed ||
      ((nav_progress.distance_mm - distance_mm) >= APP_NAV_PROGRESS_MM)) {
    nav_progress.distance_mm = distance_mm;
    nav_progress.tick_ms = now_ms;
    nav_progress.target_x_mm = command->target_x_mm;
    nav_progress.target_y_mm = command->target_y_mm;
    nav_progress.valid = true;
    return true;
  }
  return (uint32_t)(now_ms - nav_progress.tick_ms) <
         APP_NAV_NO_PROGRESS_MS;
}

static void task_process_navigation(const VisionData *vision,
                                    uint32_t now_ms)
{
  const bool navigating = state == TASK_NAVIGATE;
  const uint8_t required_command =
      navigating ? VISION_CMD_NAVIGATE_WAYPOINT :
                   VISION_CMD_ALIGN_SAFE_ZONE;
  const bool command_fresh = Vision_MissionIsFresh(
      &vision->mission, now_ms, APP_MISSION_COMMAND_TIMEOUT_MS);
  const bool command_in_grace = navigating && Vision_MissionIsFresh(
      &vision->mission, now_ms, APP_NAV_COMMAND_GRACE_MS);

  if (vision->mission.command != required_command) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
    return;
  }
  if (!command_fresh && !command_in_grace) {
    /* A waypoint is safe to retain briefly, but a prolonged RDK outage must
     * stop the chassis. Remain in NAVIGATE so a later fresh command can resume
     * the route instead of leaving the robot in a permanent fault state. */
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_progress.valid = false;
    return;
  }
  if (!fused_pose_ready(&vision->fused_pose, now_ms)) {
    if (!navigating) {
      task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
      return;
    }
    /* Never navigate blind. A transient T265 quality/freshness drop pauses
     * motion and automatically recovers when valid fused poses return. */
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_progress.valid = false;
    return;
  }

  if (state == TASK_ALIGN_SAFE_ZONE) {
    const float desired_deg =
        (float)vision->mission.heading_cdeg * 0.01f;
    const float current_deg =
        (float)vision->fused_pose.heading_cdeg * 0.01f;
    const float error_deg = task_wrap_angle(desired_deg - current_deg);
    if (nav_ready &&
        ((uint32_t)(now_ms - step_started_ms) >=
         APP_NAV_TURN_SETTLE_MS) &&
        (task_abs(error_deg) >= APP_NAV_REALIGN_DEG)) {
      nav_ready = false;
    }
    if (!nav_ready && !turn_to(desired_deg, current_deg, now_ms)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    } else if (nav_ready) {
      Motor_Stop();
      task_status.motors_active = false;
    }
    return;
  }

  const float distance = distance_to_target(&vision->mission,
                                            &vision->fused_pose);
  if (distance <= APP_NAV_TARGET_HOLD_MM) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_progress.valid = false;
    return;
  }
  const float bearing_deg = bearing_to(
      (float)vision->mission.target_x_mm,
      (float)vision->mission.target_y_mm,
      (float)vision->fused_pose.x_mm,
      (float)vision->fused_pose.y_mm);
  const float heading_deg =
      (float)vision->fused_pose.heading_cdeg * 0.01f;
  const float heading_error = task_wrap_angle(bearing_deg - heading_deg);
  if (nav_ready &&
      (task_abs(heading_error) >= APP_NAV_REALIGN_DEG)) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_progress.valid = false;
    return;
  }
  if (!nav_ready) {
    nav_progress.valid = false;
    if (!turn_to(bearing_deg, heading_deg, now_ms)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }
  if ((uint32_t)(now_ms - step_started_ms) < APP_NAV_TURN_SETTLE_MS) {
    return;
  }
  if (!navigation_progress_ok(&vision->mission, distance, now_ms)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
    return;
  }
  float speed = APP_NAV_FAST_SPEED_MM_S;
  if (!command_fresh || (distance <= APP_NAV_SLOW_DISTANCE_MM)) {
    speed = APP_NAV_SLOW_SPEED_MM_S;
  }
  /* Motor_MoveAngle() returns false while its non-blocking acceleration ramp
   * is still converging; faults are checked centrally on every task tick. */
  (void)Motor_MoveAngle(speed, 0.0f);
  task_status.motors_active = true;
}

static bool distance_failed(MotorDistanceStatus result)
{
  return (result == MOTOR_DISTANCE_FAULT) ||
         (result == MOTOR_DISTANCE_INVALID);
}

static bool ram_command_ok(const VisionMissionCommand *command,
                           uint32_t now_ms)
{
  return Vision_MissionIsFresh(command, now_ms,
                               APP_MISSION_COMMAND_TIMEOUT_MS) &&
         (command->command == VISION_CMD_ENTER_SAFE_ZONE);
}

static void task_process_ram_move(const VisionMissionCommand *command,
                                  float distance_m, float speed_mm_s,
                                  uint32_t settle_ms, TaskState next,
                                  uint32_t now_ms)
{
  if (!ram_command_ok(command, now_ms)) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) < settle_ms) {
    return;
  }
  const MotorDistanceStatus result =
      Motor_MoveDistance(distance_m, speed_mm_s);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(next, now_ms);
  } else if (distance_failed(result)) {
    task_stop(TASK_FAULT_RAM, now_ms);
  }
}

static void task_process_ram_verify(const VisionMissionCommand *command,
                                    uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS)) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) < APP_RAM_VERIFY_WAIT_MS) {
    return;
  }
  if (command->command == VISION_CMD_ENTER_SAFE_ZONE) {
    task_enter(TASK_RAM_BACK, now_ms);
  } else {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
  }
}

static void task_process_safe_exit(uint32_t now_ms)
{
  const MotorDistanceStatus result =
      Motor_MoveDistance(-APP_SAFE_EXIT_DISTANCE_M,
                         APP_SAFE_EXIT_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_FACE_FIELD_CENTER, now_ms);
  } else if (distance_failed(result)) {
    task_stop(TASK_FAULT_RAM, now_ms);
  }
}

static void task_process_face_center(const VisionData *vision,
                                     uint32_t now_ms)
{
  TaskPose pose;
  if (!read_field_pose(vision, now_ms, &pose)) {
    task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
    return;
  }
  const float center_bearing = bearing_to(
      0.0f, 0.0f, pose.x_mm, pose.y_mm);
  if (!turn_to(center_bearing, pose.heading_deg, now_ms)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  } else if (nav_ready) {
    task_enter(TASK_SEARCH, now_ms);
  }
}

static void task_accept_mission(const VisionMissionCommand *command,
                                uint32_t now_ms)
{
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS) ||
      (mission_sequence_valid &&
       (command->sequence == mission_sequence))) {
    return;
  }
  mission_sequence = command->sequence;
  mission_sequence_valid = true;

  const bool holding_grab = task_status.command_received &&
      (task_status.last_command == VISION_CMD_GRAB_CONFIRMED) &&
      ((int32_t)(now_ms - command_hold_until_ms) < 0);
  if (!holding_grab || (command->command == VISION_CMD_STOP) ||
      (command->command == VISION_CMD_ABORT)) {
    task_status.last_command = command->command;
    task_status.command_received = true;
    command_hold_until_ms =
        (command->command == VISION_CMD_GRAB_CONFIRMED) ?
        now_ms + APP_LCD_GRAB_HOLD_MS : now_ms;
  }

  if ((command->command == VISION_CMD_STOP) ||
      (command->command == VISION_CMD_ABORT)) {
    task_status.acknowledged_sequence = command->sequence;
    task_stop(TASK_FAULT_REMOTE_STOP, now_ms);
  } else if (state == TASK_STOPPED) {
    return;
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             (state >= TASK_GRAB_OBSERVE) &&
             (state <= TASK_GRAB_ROTATE)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_CLOSE_CLAW, now_ms);
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             !task_status.gripper_closed &&
             (state >= TASK_GRAB_OBSERVE) &&
             (state <= TASK_GRAB_ROTATE)) {
    /* The RDK emits GRAB_CONFIRMED for only one update before overwriting the
     * relay file with NAVIGATE. Reaching NAVIGATE while the STM32 is explicitly
     * exposing the claw proves that confirmation succeeded upstream, so close
     * the claw if that one-shot frame was missed. */
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_CLOSE_CLAW, now_ms);
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             task_status.gripper_closed &&
             (state != TASK_NAVIGATE)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_NAVIGATE, now_ms);
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             (state == TASK_NAVIGATE)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             task_status.gripper_closed &&
             (state != TASK_ALIGN_SAFE_ZONE)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_ALIGN_SAFE_ZONE, now_ms);
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (state == TASK_ALIGN_SAFE_ZONE)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
              task_status.gripper_closed &&
              (state == TASK_ALIGN_SAFE_ZONE) && nav_ready) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_OPEN_FOR_RAM, now_ms);
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
             ((state == TASK_OPEN_FOR_RAM) ||
              (state == TASK_RAM_BACK) ||
              (state == TASK_RAM_FORWARD) ||
              (state == TASK_RAM_VERIFY))) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_TASK_COMPLETE) &&
             !task_status.gripper_closed &&
             ((state == TASK_RAM_BACK) ||
              (state == TASK_RAM_FORWARD) ||
              (state == TASK_RAM_VERIFY))) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_EXIT_SAFE_ZONE, now_ms);
  }
}

void Task_Process(uint32_t now_ms)
{
  if (!initialized) {
    task_initialize(now_ms);
  }

  if (!initial_claw_ready) {
    Motor_Stop();
    if (Claw_Retract(now_ms)) {
      initial_claw_ready = true;
    }
    task_publish_status(now_ms);
    return;
  }

  const VisionData vision = Vision_GetSnapshot();
  task_update_match_time(now_ms);
  task_status.camera_angle = Camera_GetAngle();
  if (task_motor_fault()) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  task_accept_mission(&vision.mission, now_ms);

  switch (state) {
    case TASK_WAIT_CONFIG:
      Motor_Stop();
      if (vision.config_ready) {
        Location_Reset((LocationStart)vision.start_zone);
        Vision_RequestConfigAck();
        match_started = true;
        match_started_ms = now_ms;
        task_enter(TASK_START, now_ms);
      }
      break;

    case TASK_START:
      task_process_start(now_ms);
      break;

    case TASK_OPEN_CLAW:
      if (Claw_Open(now_ms)) {
#if APP_ENABLE_START_SCATTER
        task_enter(TASK_PILE_APPROACH, now_ms);
#else
        task_enter(TASK_SEARCH, now_ms);
#endif
      }
      break;

    case TASK_PILE_APPROACH:
      task_process_pile_approach(now_ms);
      break;

    case TASK_SCATTER_POSITIVE:
      task_process_scatter(APP_SCATTER_ROTATE_SPEED_MM_S,
                           TASK_SCATTER_PAUSE, now_ms);
      break;

    case TASK_SCATTER_PAUSE:
      task_process_scatter_pause(now_ms);
      break;

    case TASK_SCATTER_NEGATIVE:
      task_process_scatter(-APP_SCATTER_ROTATE_SPEED_MM_S,
                           TASK_SCATTER_EXIT, now_ms);
      break;

    case TASK_SCATTER_EXIT:
      task_process_scatter_exit(now_ms);
      break;

    case TASK_SEARCH:
      task_process_search(&vision, now_ms);
      break;

    case TASK_APPROACH:
      task_process_approach(&vision, now_ms);
      break;

    case TASK_APPROACH_RECOVER:
      task_process_approach_recover(&vision, now_ms);
      break;

    case TASK_GRAB_OBSERVE:
      task_process_grab_observe(&vision, now_ms);
      break;

    case TASK_GRAB_RAISE_WAIT:
      task_process_grab_raise_wait(&vision, now_ms);
      break;

    case TASK_GRAB_ROTATE:
      task_process_grab_rotate(&vision, now_ms);
      break;

    case TASK_CLOSE_CLAW:
      if (Claw_Touch(now_ms)) {
        task_status.gripper_closed = true;
        task_enter(TASK_WAIT_NAVIGATION, now_ms);
      }
      break;

    case TASK_WAIT_NAVIGATION:
      Motor_Stop();
      break;

    case TASK_NAVIGATE:
    case TASK_ALIGN_SAFE_ZONE:
      task_process_navigation(&vision, now_ms);
      break;

    case TASK_OPEN_FOR_RAM:
      if (!ram_command_ok(&vision.mission, now_ms)) {
        task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
      } else if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_enter(TASK_RAM_BACK, now_ms);
      }
      break;

    case TASK_RAM_BACK:
      task_process_ram_move(&vision.mission,
                            -APP_RAM_BACK_DISTANCE_M,
                            APP_RAM_BACK_SPEED_MM_S,
                            0U, TASK_RAM_FORWARD, now_ms);
      break;

    case TASK_RAM_FORWARD:
      task_process_ram_move(&vision.mission,
                            APP_RAM_FORWARD_DISTANCE_M,
                            APP_RAM_FORWARD_SPEED_MM_S,
                            APP_RAM_DIRECTION_SETTLE_MS,
                            TASK_RAM_VERIFY, now_ms);
      break;

    case TASK_RAM_VERIFY:
      task_process_ram_verify(&vision.mission, now_ms);
      break;

    case TASK_EXIT_SAFE_ZONE:
      task_process_safe_exit(now_ms);
      break;

    case TASK_FACE_FIELD_CENTER:
      task_process_face_center(&vision, now_ms);
      break;

    case TASK_STOPPED:
      Motor_Stop();
      task_status.motors_active = false;
      break;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      break;
  }

  task_status.state = state;
  task_status.camera_angle = Camera_GetAngle();
  task_publish_status(now_ms);
}

TaskStatus Task_GetStatus(void)
{
  TaskStatus snapshot;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot = task_status;
  if (primask == 0U) {
    __enable_irq();
  }
  return snapshot;
}
