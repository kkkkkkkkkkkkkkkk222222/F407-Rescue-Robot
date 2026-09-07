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

typedef enum {
  SEARCH_SWEEP_HIGH = 0,
  SEARCH_SWEEP_LOW,
  SEARCH_REALIGN,
  SEARCH_ADVANCE
} SearchPhase;

typedef enum {
  RECOVER_WAIT = 0,
  RECOVER_CAMERA_TO_140,
  RECOVER_HOLD_140,
  RECOVER_CAMERA_TO_90,
  RECOVER_HOLD_90,
  RECOVER_SWEEP_90,
  RECOVER_REALIGN,
  RECOVER_ADVANCE
} RecoverPhase;

typedef enum {
  REMOTE_ROUTE_WAITING = 0,
  REMOTE_ROUTE_RUNNING,
  REMOTE_ROUTE_REACHED,
  REMOTE_ROUTE_COMMAND_INVALID,
  REMOTE_ROUTE_MOTOR_FAULT
} RemoteRouteStatus;

static volatile TaskStatus task_status;
static Pid_t steering_pid;
static Pid_t camera_pid;
static TaskState state;
static TurnTracker turn_tracker;
static SearchPhase search_phase;
static RecoverPhase recover_phase;
static uint32_t state_started_ms;
static uint32_t step_started_ms;
static uint32_t status_sent_ms;
static uint32_t pose_invalid_started_ms;
static uint32_t approach_last_target_ms;
static uint32_t nav_payload_change_ms;
static float camera_angle;
static float steering_mm_s;
static float approach_speed_mm_s;
static float filtered_target_x;
static float filtered_target_y;
static float remote_speed_mm_s;
static float remote_yaw_mm_s;
static uint32_t tracking_tick_ms;
static uint32_t approach_report_generation;
static uint32_t search_counted_report_generation;
static int16_t nav_last_distance_mm;
static uint8_t tracking_sequence;
static uint8_t mission_sequence;
static uint32_t start_reverse_path_mm;
static float start_target_heading_deg;
static float reposition_heading_deg;
static uint32_t scan_entry_report_generation;
static uint8_t search_phase_report_count;
static uint8_t locked_cargo_counts;
static bool initialized;
static bool initial_claw_ready;
static bool tracking_valid;
static bool tracking_filter_valid;
static bool approach_report_generation_valid;
static bool mission_sequence_valid;
static bool steering_active;
static bool nav_ready;
static bool scan_report_gate_open;
static bool reposition_heading_valid;
static bool search_start_low;
static bool pose_invalid_pending;
static bool start_clearance_done;
static bool distance_command_done;
static bool nav_payload_valid;
static bool nav_payload_stale;
static bool nav_forward_active;

static void task_enter(TaskState next, uint32_t now_ms);
static bool distance_failed(MotorDistanceStatus result);
static bool fused_pose_ready(const VisionFusedPose *pose, uint32_t now_ms);

static float task_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float task_step_toward(float current, float target, float step)
{
  if (current < target) {
    const float next = current + step;
    return (next < target) ? next : target;
  }
  if (current > target) {
    const float next = current - step;
    return (next > target) ? next : target;
  }
  return current;
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

static bool task_get_location_pose(LocationPose *pose, uint32_t now_ms)
{
  *pose = Location_GetPose();
  if (pose->valid) {
    pose_invalid_pending = false;
    return true;
  }

  Motor_Stop();
  task_status.motors_active = false;
  if (!pose_invalid_pending) {
    pose_invalid_pending = true;
    pose_invalid_started_ms = now_ms;
  } else if ((uint32_t)(now_ms - pose_invalid_started_ms) >=
             APP_POSE_WAIT_TIMEOUT_MS) {
    task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
  }
  return false;
}

static void task_reset_tracking(void)
{
  tracking_valid = false;
  tracking_filter_valid = false;
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
  flags |= ((state == TASK_NAVIGATE) && distance_command_done) ?
      VISION_STM_DISTANCE_DONE : 0U;
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
  (void)now_ms;
  /* Continuous rescue mode has no match-duration stop. Communication, pose,
   * motor and mechanism watchdogs remain active as safety interlocks. */
  task_status.remaining_s = UINT16_MAX;
}

static void task_enter(TaskState next, uint32_t now_ms)
{
  Motor_Stop();
  state = next;
  task_status.state = next;
  task_status.motors_active = false;
  task_status.auto_approach = false;
  task_status.nav_stale = false;
  task_status.nav_done = false;
  task_status.claw_visible =
      (next >= TASK_GRAB_OBSERVE) && (next <= TASK_CLOSE_CLAW);
  state_started_ms = now_ms;
  step_started_ms = now_ms;
  pose_invalid_pending = false;
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;

  if (next == TASK_START) {
    const LocationPose pose = Location_GetPose();
    start_reverse_path_mm = pose.path_mm;
    start_target_heading_deg = task_wrap_angle(
        (float)pose.heading_mdeg * 0.001f + APP_START_TURN_DEG);
    start_clearance_done = false;
  } else if (next == TASK_SEARCH) {
    const VisionData vision = Vision_GetSnapshot();
    const uint8_t start_angle = search_start_low ?
        APP_SEARCH_LOW_CAMERA_ANGLE : APP_SEARCH_CAMERA_ANGLE;
    Camera_SetAngle(start_angle);
    camera_angle = (float)start_angle;
    task_status.found = false;
    locked_cargo_counts = 0U;
    search_phase = search_start_low ?
        SEARCH_SWEEP_LOW : SEARCH_SWEEP_HIGH;
    search_start_low = false;
    scan_entry_report_generation = vision.report_generation;
    search_counted_report_generation = vision.report_generation;
    search_phase_report_count = 0U;
    scan_report_gate_open = false;
    reposition_heading_valid = false;
    task_reset_tracking();
    task_reset_turn_tracker();
  } else if (next == TASK_APPROACH) {
    camera_angle = (float)Camera_GetAngle();
    approach_speed_mm_s = APP_APPROACH_SPEED_MM_S;
    approach_last_target_ms = now_ms;
    approach_report_generation_valid = false;
    task_reset_tracking();
  } else if (next == TASK_APPROACH_RECOVER) {
    const VisionData vision = Vision_GetSnapshot();
    const LocationPose pose = Location_GetPose();
    recover_phase = RECOVER_WAIT;
    scan_entry_report_generation = vision.report_generation;
    scan_report_gate_open = false;
    reposition_heading_valid = pose.valid;
    if (pose.valid) {
      reposition_heading_deg = (float)pose.heading_mdeg * 0.001f;
    }
    task_reset_tracking();
    task_reset_turn_tracker();
  } else if (next == TASK_GRAB_OBSERVE) {
    task_reset_tracking();
  } else if (next == TASK_GRAB_ROTATE) {
    task_reset_turn_tracker();
  } else if ((next == TASK_SCATTER_POSITIVE) ||
             (next == TASK_SCATTER_NEGATIVE)) {
    task_reset_turn_tracker();
  } else if (next == TASK_EXIT_SAFE_ZONE) {
    Camera_SetAngle(APP_SEARCH_CAMERA_ANGLE);
    camera_angle = (float)APP_SEARCH_CAMERA_ANGLE;
  } else if ((next == TASK_NAVIGATE) ||
             (next == TASK_ALIGN_SAFE_ZONE) ||
             (next == TASK_FACE_FIELD_CENTER)) {
    nav_ready = false;
    distance_command_done = false;
    nav_payload_valid = false;
    nav_payload_stale = false;
    nav_forward_active = false;
    remote_speed_mm_s = 0.0f;
    remote_yaw_mm_s = 0.0f;
    nav_payload_change_ms = now_ms;
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
  task_status.remaining_s = UINT16_MAX;
  state = TASK_WAIT_CONFIG;
  task_status.state = state;
  state_started_ms = now_ms;
  step_started_ms = now_ms;
  status_sent_ms = now_ms - APP_TASK_STATUS_PERIOD_MS;
  pose_invalid_started_ms = now_ms;
  approach_last_target_ms = now_ms;
  nav_payload_change_ms = now_ms;
  camera_angle = (float)Camera_GetAngle();
  steering_mm_s = 0.0f;
  approach_speed_mm_s = APP_APPROACH_SPEED_MM_S;
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
  tracking_sequence = 0U;
  tracking_tick_ms = 0U;
  approach_report_generation = 0U;
  search_counted_report_generation = 0U;
  nav_last_distance_mm = 0;
  mission_sequence = 0U;
  start_reverse_path_mm = 0U;
  start_target_heading_deg = 0.0f;
  reposition_heading_deg = 0.0f;
  scan_entry_report_generation = 0U;
  search_phase_report_count = 0U;
  locked_cargo_counts = 0U;
  initial_claw_ready = false;
  tracking_valid = false;
  tracking_filter_valid = false;
  approach_report_generation_valid = false;
  mission_sequence_valid = false;
  steering_active = false;
  nav_ready = false;
  scan_report_gate_open = false;
  reposition_heading_valid = false;
  search_start_low = false;
  pose_invalid_pending = false;
  start_clearance_done = false;
  distance_command_done = false;
  nav_payload_valid = false;
  nav_payload_stale = false;
  nav_forward_active = false;
  search_phase = SEARCH_SWEEP_HIGH;
  recover_phase = RECOVER_WAIT;
  task_reset_turn_tracker();
  initialized = true;
}

static bool task_target_is_single_cargo(const VisionData *vision,
                                        uint32_t now_ms)
{
  const uint8_t count =
      VISION_COUNT_NORMAL(vision->cargo_counts) +
      VISION_COUNT_CORE(vision->cargo_counts) +
      VISION_COUNT_CASUALTY(vision->cargo_counts) +
      VISION_COUNT_DANGER(vision->cargo_counts);
  return Vision_IsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS) &&
         vision->classification_valid &&
         (count == 1U);
}

static bool task_target_matches_lock(const VisionData *vision,
                                     uint32_t now_ms)
{
  return (locked_cargo_counts != 0U) &&
         task_target_is_single_cargo(vision, now_ms) &&
         (vision->cargo_counts == locked_cargo_counts);
}

static bool task_scan_report_ready(const VisionData *vision,
                                   uint32_t now_ms)
{
  if (!scan_report_gate_open &&
      (vision->report_generation != scan_entry_report_generation) &&
      Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    scan_report_gate_open = true;
  }
  return scan_report_gate_open;
}

static bool task_scan_target_found(const VisionData *vision,
                                   uint32_t now_ms)
{
  return task_scan_report_ready(vision, now_ms) &&
         task_target_is_single_cargo(vision, now_ms);
}

static bool task_scan_locked_target_found(const VisionData *vision,
                                          uint32_t now_ms)
{
  return task_scan_report_ready(vision, now_ms) &&
         task_target_matches_lock(vision, now_ms);
}

static void task_count_search_report(const VisionData *vision,
                                     uint32_t now_ms)
{
  if ((vision->report_generation != search_counted_report_generation) &&
      Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    search_counted_report_generation = vision->report_generation;
    if (search_phase_report_count < UINT8_MAX) {
      ++search_phase_report_count;
    }
  }
}

static void task_reset_search_report_count(const VisionData *vision)
{
  search_counted_report_generation = vision->report_generation;
  search_phase_report_count = 0U;
}

static bool task_scan_camera_to(uint8_t target_angle, uint32_t now_ms)
{
  const uint8_t current_angle = Camera_GetAngle();
  if (current_angle == target_angle) {
    camera_angle = (float)current_angle;
    return true;
  }
  if ((uint32_t)(now_ms - step_started_ms) < APP_CAMERA_SCAN_STEP_MS) {
    return false;
  }

  uint8_t next_angle = current_angle;
  if (current_angle < target_angle) {
    const uint16_t raised = (uint16_t)current_angle +
                            APP_CAMERA_SCAN_STEP_DEG;
    next_angle = (raised < target_angle) ? (uint8_t)raised : target_angle;
  } else {
    const int16_t lowered = (int16_t)current_angle -
                            (int16_t)APP_CAMERA_SCAN_STEP_DEG;
    next_angle = (lowered > (int16_t)target_angle) ?
        (uint8_t)lowered : target_angle;
  }
  Camera_SetAngle(next_angle);
  camera_angle = (float)next_angle;
  step_started_ms = now_ms;
  return next_angle == target_angle;
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

  if (!tracking_filter_valid) {
    filtered_target_x = (float)vision->x;
    filtered_target_y = (float)vision->y;
    tracking_filter_valid = true;
  } else {
    filtered_target_x += APP_VISION_COORD_FILTER_ALPHA *
        ((float)vision->x - filtered_target_x);
    filtered_target_y += APP_VISION_COORD_FILTER_ALPHA *
        ((float)vision->y - filtered_target_y);
  }

  const float y_error = (float)APP_VISION_TARGET_Y - filtered_target_y;
  if ((y_error >= -(float)APP_CAMERA_DEAD_ZONE) &&
      (y_error <= (float)APP_CAMERA_DEAD_ZONE)) {
    Pid_Reset(&camera_pid);
  } else {
    float camera_step = Pid_UpdateDt(&camera_pid,
                                     (float)APP_VISION_TARGET_Y,
                                     filtered_target_y, dt_s);
    const float camera_step_limit =
        APP_CAMERA_TRACK_MAX_RATE_DEG_S * dt_s;
    if (camera_step > camera_step_limit) {
      camera_step = camera_step_limit;
    } else if (camera_step < -camera_step_limit) {
      camera_step = -camera_step_limit;
    }
    camera_angle -= camera_step;
    /* This is the camera direction and range used by the verified 14:48
     * approach controller: larger servo-3 angles look farther down. */
    if (camera_angle < 90.0f) {
      camera_angle = 90.0f;
    } else if (camera_angle > 165.0f) {
      camera_angle = 165.0f;
    }
    Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
  }

  const float x_error = (float)APP_VISION_TARGET_X - filtered_target_x;
  const float magnitude = task_abs(x_error);
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
                               filtered_target_x, dt_s) *
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
    task_enter(TASK_OPEN_CLAW, now_ms);
    return;
  }

  const uint32_t remaining_mm = APP_START_REVERSE_DISTANCE_MM - travelled_mm;
  const float speed_mm_s =
      (remaining_mm <= APP_START_REVERSE_SLOW_REMAINING_MM) ?
      APP_START_REVERSE_SLOW_SPEED_MM_S : APP_START_REVERSE_SPEED_MM_S;

  /* The first 0.60 m is a true encoder-distance move with IMU heading hold.
   * Keep every mechanism folded until that obstacle clearance is complete. */
  if (!start_clearance_done) {
    const MotorDistanceStatus result = Motor_MoveDistance(
        -APP_START_CLEARANCE_DISTANCE_M,
        APP_START_CLEARANCE_SPEED_MM_S);
    task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
    if (result == MOTOR_DISTANCE_DONE) {
      start_clearance_done = true;
      Lift_SetTravelPosition();
      (void)Claw_Open(now_ms);
    } else if (distance_failed(result)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }

  Lift_SetTravelPosition();
  (void)Claw_Open(now_ms);
  const float heading_deg = (float)pose.heading_mdeg * 0.001f;
  float heading_error = task_wrap_angle(start_target_heading_deg - heading_deg);
  if (heading_error <= -179.9f) {
    heading_error = 180.0f;
  }
  float yaw_mm_s = heading_error * APP_START_TURN_KP_MM_S_PER_DEG;
  if (yaw_mm_s > APP_START_TURN_MAX_MM_S) {
    yaw_mm_s = APP_START_TURN_MAX_MM_S;
  } else if (yaw_mm_s < -APP_START_TURN_MAX_MM_S) {
    yaw_mm_s = -APP_START_TURN_MAX_MM_S;
  } else if (task_abs(heading_error) <= APP_START_TURN_TOLERANCE_DEG) {
    yaw_mm_s = 0.0f;
  }

  /* Outside the obstacle zone, keep the remaining path straight on the floor
   * while rotating toward the final heading and opening both claws. */
  if (!Motor_MoveSpin(speed_mm_s, 180.0f, yaw_mm_s)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
    return;
  }
  task_status.motors_active = true;
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

static bool task_choose_reposition_heading(const VisionData *vision,
                                           uint32_t now_ms,
                                           float distance_m,
                                           float *heading_deg)
{
  TaskPose pose;
  if (!read_field_pose(vision, now_ms, &pose)) {
    LocationPose local;
    (void)task_get_location_pose(&local, now_ms);
    return false;
  }
  pose_invalid_pending = false;

  const float heading_rad = *heading_deg * 0.01745329252f;
  const float distance_mm = distance_m * 1000.0f;
  const float end_x = pose.x_mm + cosf(heading_rad) * distance_mm;
  const float end_y = pose.y_mm + sinf(heading_rad) * distance_mm;
  const float limit = APP_LOCATION_FIELD_HALF_MM -
                      APP_SEARCH_FIELD_MARGIN_MM;
  if (!pose.inside_field ||
      (task_abs(end_x) > limit) || (task_abs(end_y) > limit)) {
    *heading_deg = atan2f(-pose.y_mm, -pose.x_mm) * 57.2957795f;
  }
  return true;
}

static MotorTurnStatus task_turn_to_heading(float desired_heading_deg,
                                            uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    return MOTOR_TURN_IDLE;
  }
  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float error_deg = task_wrap_angle(
      desired_heading_deg - current_heading_deg);
  if (task_abs(error_deg) <= APP_NAV_HEADING_TOLERANCE_DEG) {
    Motor_Stop();
    return MOTOR_TURN_DONE;
  }
  return Motor_TurnAngle(error_deg);
}

static void task_process_search(const VisionData *vision, uint32_t now_ms)
{
  /* Reports received before this SEARCH epoch may still be younger than the
   * normal 250 ms timeout. They remain visible on the LCD, but cannot select
   * a target or enter APPROACH until a new valid report is received. */
  const bool camera_ready =
      ((search_phase == SEARCH_SWEEP_HIGH) &&
       ((uint32_t)(now_ms - state_started_ms) >= APP_TARGET_WAIT_MS)) ||
      ((search_phase == SEARCH_SWEEP_LOW) &&
       ((uint32_t)(now_ms - step_started_ms) >=
        APP_CAMERA_SCAN_ENDPOINT_HOLD_MS)) ||
      ((search_phase != SEARCH_SWEEP_HIGH) &&
       (search_phase != SEARCH_SWEEP_LOW));
  if (camera_ready) {
    task_count_search_report(vision, now_ms);
  }
  task_status.found = camera_ready &&
                      task_scan_target_found(vision, now_ms);
  if (task_status.found) {
    locked_cargo_counts = vision->cargo_counts;
    task_enter(TASK_APPROACH, now_ms);
    return;
  }

  switch (search_phase) {
    case SEARCH_SWEEP_HIGH: {
      if ((uint32_t)(now_ms - state_started_ms) < APP_TARGET_WAIT_MS) {
        return;
      }
      LocationPose pose;
      if (!task_get_location_pose(&pose, now_ms)) {
        return;
      }
      if (!reposition_heading_valid) {
        reposition_heading_deg = (float)pose.heading_mdeg * 0.001f;
        reposition_heading_valid = true;
      }
      if (task_full_turn_reached()) {
        Motor_Stop();
        task_status.motors_active = false;
        if (search_phase_report_count <
            APP_SEARCH_MIN_REPORTS_PER_SWEEP) {
          task_reset_turn_tracker();
          task_reset_search_report_count(vision);
          return;
        }
        Camera_SetAngle(APP_SEARCH_LOW_CAMERA_ANGLE);
        camera_angle = (float)APP_SEARCH_LOW_CAMERA_ANGLE;
        search_phase = SEARCH_SWEEP_LOW;
        task_reset_turn_tracker();
        task_reset_search_report_count(vision);
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;
    }

    case SEARCH_SWEEP_LOW:
      if ((uint32_t)(now_ms - step_started_ms) <
          APP_CAMERA_SCAN_ENDPOINT_HOLD_MS) {
        return;
      }
      {
        LocationPose pose;
        if (!task_get_location_pose(&pose, now_ms)) {
          return;
        }
        if (!reposition_heading_valid) {
          reposition_heading_deg = (float)pose.heading_mdeg * 0.001f;
          reposition_heading_valid = true;
        }
      }
      if (task_full_turn_reached()) {
        Motor_Stop();
        task_status.motors_active = false;
        if (search_phase_report_count <
            APP_SEARCH_MIN_REPORTS_PER_SWEEP) {
          task_reset_turn_tracker();
          task_reset_search_report_count(vision);
          return;
        }
        if (!task_choose_reposition_heading(
                vision, now_ms, APP_SEARCH_ADVANCE_DISTANCE_M,
                &reposition_heading_deg)) {
          return;
        }
        search_phase = SEARCH_REALIGN;
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;

    case SEARCH_REALIGN: {
      const MotorTurnStatus result =
          task_turn_to_heading(reposition_heading_deg, now_ms);
      task_status.motors_active = result == MOTOR_TURN_RUNNING;
      if (result == MOTOR_TURN_DONE) {
        Motor_Stop();
        search_phase = SEARCH_ADVANCE;
        step_started_ms = now_ms;
      } else if ((result == MOTOR_TURN_FAULT) ||
                 (result == MOTOR_TURN_INVALID)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      return;
    }

    case SEARCH_ADVANCE: {
      if ((uint32_t)(now_ms - step_started_ms) <
          APP_NAV_TURN_SETTLE_MS) {
        return;
      }
      const MotorDistanceStatus result = Motor_MoveDistance(
          APP_SEARCH_ADVANCE_DISTANCE_M,
          APP_SEARCH_ADVANCE_SPEED_MM_S);
      task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
      if (result == MOTOR_DISTANCE_DONE) {
        task_enter(TASK_SEARCH, now_ms);
      } else if (distance_failed(result)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      return;
    }

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      return;
  }
}

static float task_approach_speed(const VisionData *vision)
{
  float speed = APP_APPROACH_SPEED_MM_S;
  if (vision->distance_valid) {
    if (vision->distance_mm <= APP_GRAB_SLOW_DISTANCE_MM) {
      speed = APP_GRAB_SLOW_SPEED_MM_S;
    } else if (vision->distance_mm <= APP_GRAB_MID_DISTANCE_MM) {
      speed = APP_GRAB_MID_SPEED_MM_S;
    }
  }
  return speed;
}

static float task_approach_freshness_scale(uint32_t age_ms)
{
  if (age_ms <= APP_APPROACH_FRAME_HOLD_MS) {
    return 1.0f;
  }
  if (age_ms >= APP_APPROACH_FRAME_STOP_MS) {
    return 0.0f;
  }
  return (float)(APP_APPROACH_FRAME_STOP_MS - age_ms) /
         (float)(APP_APPROACH_FRAME_STOP_MS -
                 APP_APPROACH_FRAME_HOLD_MS);
}

static void task_process_approach(const VisionData *vision, uint32_t now_ms)
{
  task_status.auto_approach = true;
  const bool new_report = !approach_report_generation_valid ||
      (vision->report_generation != approach_report_generation);

  if (new_report) {
    approach_report_generation = vision->report_generation;
    approach_report_generation_valid = true;
    if (task_target_matches_lock(vision, now_ms)) {
      approach_last_target_ms = now_ms;
      approach_speed_mm_s = task_approach_speed(vision);
      task_status.found = true;
      const float turn = task_track_target(vision);
      if (camera_angle >= (float)APP_GRAB_VIEW_ANGLE) {
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        camera_angle = (float)APP_GRAB_VIEW_ANGLE;
        task_enter(TASK_GRAB_OBSERVE, now_ms);
        return;
      }
      Motor_Move(approach_speed_mm_s, 0.0f, turn);
      task_status.motors_active = true;
      return;
    }
  }

  const uint32_t report_age_ms = now_ms - approach_last_target_ms;
  if (report_age_ms >= APP_APPROACH_FRAME_LOSS_MS) {
    Motor_Stop();
    task_status.found = false;
    task_status.motors_active = false;
    task_enter(TASK_APPROACH_RECOVER, now_ms);
    return;
  }

  const float freshness_scale =
      task_approach_freshness_scale(report_age_ms);
  task_status.found = freshness_scale > 0.0f;
  if (freshness_scale <= 0.0f) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }

  Motor_Move(approach_speed_mm_s * freshness_scale,
             0.0f, steering_mm_s * freshness_scale);
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
  task_status.found = task_scan_locked_target_found(vision, now_ms);
  const bool loss_hold_complete =
      (uint32_t)(now_ms - state_started_ms) >=
      APP_APPROACH_LOSS_HOLD_MS;
  if (task_status.found &&
      ((recover_phase != RECOVER_WAIT) || loss_hold_complete)) {
    task_enter(TASK_APPROACH, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_APPROACH_RECOVERY_TIMEOUT_MS) {
    task_enter(TASK_SEARCH, now_ms);
    return;
  }

  switch (recover_phase) {
    case RECOVER_WAIT: {
      Motor_Stop();
      task_status.motors_active = false;
      if (!reposition_heading_valid) {
        LocationPose pose;
        if (task_get_location_pose(&pose, now_ms)) {
          reposition_heading_deg = (float)pose.heading_mdeg * 0.001f;
          reposition_heading_valid = true;
        }
      }
      if (loss_hold_complete) {
        recover_phase = RECOVER_CAMERA_TO_140;
        step_started_ms = now_ms;
      }
      return;
    }

    case RECOVER_CAMERA_TO_140:
      if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
        recover_phase = RECOVER_HOLD_140;
        step_started_ms = now_ms;
      }
      return;

    case RECOVER_HOLD_140:
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_CAMERA_SCAN_ENDPOINT_HOLD_MS) {
        recover_phase = RECOVER_CAMERA_TO_90;
        step_started_ms = now_ms;
      }
      return;

    case RECOVER_CAMERA_TO_90:
      if (task_scan_camera_to(APP_SEARCH_LOW_CAMERA_ANGLE, now_ms)) {
        recover_phase = RECOVER_HOLD_90;
        step_started_ms = now_ms;
      }
      return;

    case RECOVER_HOLD_90:
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_CAMERA_SCAN_ENDPOINT_HOLD_MS) {
        recover_phase = RECOVER_SWEEP_90;
        task_reset_turn_tracker();
        step_started_ms = now_ms;
      }
      return;

    case RECOVER_SWEEP_90: {
      LocationPose pose;
      if (!task_get_location_pose(&pose, now_ms)) {
        return;
      }
      if (!reposition_heading_valid) {
        reposition_heading_deg = (float)pose.heading_mdeg * 0.001f;
        reposition_heading_valid = true;
      }
      if (task_full_turn_reached()) {
        Motor_Stop();
        task_status.motors_active = false;
        if (!task_choose_reposition_heading(
                vision, now_ms,
                APP_APPROACH_RECOVERY_ADVANCE_DISTANCE_M,
                &reposition_heading_deg)) {
          return;
        }
        recover_phase = RECOVER_REALIGN;
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f,
                 APP_APPROACH_RECOVERY_ROTATE_MM_S);
      task_status.motors_active = true;
      return;
    }

    case RECOVER_REALIGN: {
      const MotorTurnStatus result =
          task_turn_to_heading(reposition_heading_deg, now_ms);
      task_status.motors_active = result == MOTOR_TURN_RUNNING;
      if (result == MOTOR_TURN_DONE) {
        Motor_Stop();
        recover_phase = RECOVER_ADVANCE;
        step_started_ms = now_ms;
      } else if ((result == MOTOR_TURN_FAULT) ||
                 (result == MOTOR_TURN_INVALID)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      return;
    }

    case RECOVER_ADVANCE: {
      if ((uint32_t)(now_ms - step_started_ms) <
          APP_NAV_TURN_SETTLE_MS) {
        return;
      }
      const MotorDistanceStatus result = Motor_MoveDistance(
          APP_APPROACH_RECOVERY_ADVANCE_DISTANCE_M,
          APP_APPROACH_RECOVERY_ADVANCE_SPEED_MM_S);
      task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
      if (result == MOTOR_DISTANCE_DONE) {
        task_enter(TASK_SEARCH, now_ms);
      } else if (distance_failed(result)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      return;
    }

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      return;
  }
}

static void task_process_grab_observe(const VisionData *vision,
                                      uint32_t now_ms)
{
  task_status.found = task_target_matches_lock(vision, now_ms);
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
  task_status.found = task_target_matches_lock(vision, now_ms);
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
  task_status.found = task_target_matches_lock(vision, now_ms);
  if (task_status.found) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
  if (!Vision_ReportIsFresh(vision, now_ms, APP_VISION_TIMEOUT_MS)) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
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

static bool turn_to(float desired_deg, float current_deg,
                    float tolerance_deg, uint32_t now_ms)
{
  const float error_deg = task_wrap_angle(desired_deg - current_deg);
  if (task_abs(error_deg) <= tolerance_deg) {
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

static bool task_distance_command_valid(const VisionMissionCommand *command,
                                        uint8_t expected_command)
{
  const uint8_t required_flags = VISION_CMD_DRIVE_STRAIGHT |
                                 VISION_CMD_USE_FINAL_HEADING |
                                 VISION_CMD_DISTANCE_VALID;
  return (command->command == expected_command) &&
         ((command->flags & required_flags) == required_flags) &&
         (command->target_x_mm >= 0) &&
         (command->target_x_mm <= APP_NAV_REMOTE_MAX_DISTANCE_MM) &&
         (command->target_y_mm == 0);
}

static float task_navigation_end_speed(float cruise_speed_mm_s)
{
  const float scaled_speed = cruise_speed_mm_s * APP_NAV_END_SPEED_RATIO;
  return (scaled_speed < APP_NAV_MIN_END_SPEED_MM_S) ?
      APP_NAV_MIN_END_SPEED_MM_S : scaled_speed;
}

static void task_reset_remote_targets(void)
{
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
}

static float task_remote_route_speed(int16_t remaining_mm,
                                     float cruise_speed_mm_s)
{
  const float end_speed_mm_s = task_navigation_end_speed(cruise_speed_mm_s);
  if (remaining_mm >= (int16_t)APP_NAV_LINEAR_SLOWDOWN_MM) {
    return cruise_speed_mm_s;
  }
  const float usable_mm = APP_NAV_LINEAR_SLOWDOWN_MM -
                          (float)APP_NAV_REMOTE_STOP_DISTANCE_MM;
  float ratio = ((float)remaining_mm -
                 (float)APP_NAV_REMOTE_STOP_DISTANCE_MM) / usable_mm;
  if (ratio < 0.0f) {
    ratio = 0.0f;
  } else if (ratio > 1.0f) {
    ratio = 1.0f;
  }
  return end_speed_mm_s +
      (cruise_speed_mm_s - end_speed_mm_s) * ratio;
}

static bool task_nav_payload_changed(const VisionMissionCommand *command,
                                     uint32_t now_ms)
{
  const int32_t distance_change =
      (int32_t)command->target_x_mm - (int32_t)nav_last_distance_mm;
  const bool changed = !nav_payload_valid ||
      (distance_change >= APP_NAV_REMOTE_PROGRESS_MM) ||
      (distance_change <= -APP_NAV_REMOTE_PROGRESS_MM);
  if (changed) {
    nav_last_distance_mm = command->target_x_mm;
    nav_payload_change_ms = now_ms;
    nav_payload_valid = true;
    nav_payload_stale = false;
    task_status.nav_stale = false;
  }
  return changed;
}

static float task_remote_heading_correction(float heading_error_deg)
{
  const float magnitude = task_abs(heading_error_deg);
  if (magnitude <= APP_NAV_HEADING_TOLERANCE_DEG) {
    return 0.0f;
  }
  float correction = (magnitude - APP_NAV_HEADING_TOLERANCE_DEG) *
                     APP_NAV_HEADING_KP_MM_S_PER_DEG;
  if (heading_error_deg < 0.0f) {
    correction = -correction;
  }
  if (correction > APP_NAV_HEADING_MAX_MM_S) {
    correction = APP_NAV_HEADING_MAX_MM_S;
  } else if (correction < -APP_NAV_HEADING_MAX_MM_S) {
    correction = -APP_NAV_HEADING_MAX_MM_S;
  }
  return correction;
}

static RemoteRouteStatus task_follow_remote_route(
    const VisionMissionCommand *command, uint8_t expected_command,
    float cruise_speed_mm_s, uint32_t now_ms)
{
  if (!task_distance_command_valid(command, expected_command)) {
    return REMOTE_ROUTE_COMMAND_INVALID;
  }

  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    nav_forward_active = false;
    task_reset_remote_targets();
    return REMOTE_ROUTE_WAITING;
  }
  (void)task_nav_payload_changed(command, now_ms);

  if ((command->target_x_mm <=
       (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM) ||
      (distance_command_done &&
       (command->target_x_mm <=
        (int16_t)APP_NAV_REMOTE_RESUME_DISTANCE_MM))) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    task_reset_remote_targets();
    return REMOTE_ROUTE_REACHED;
  }
  distance_command_done = false;
  task_status.nav_done = false;

  const float desired_heading_deg = (float)command->heading_cdeg * 0.01f;
  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float heading_error_deg = task_wrap_angle(
      desired_heading_deg - current_heading_deg);
  if (nav_ready &&
      (task_abs(heading_error_deg) >= APP_NAV_REALIGN_DEG)) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return REMOTE_ROUTE_WAITING;
  }
  if (!nav_ready) {
    nav_forward_active = false;
    task_reset_remote_targets();
    if (!turn_to(desired_heading_deg, current_heading_deg,
                 APP_NAV_HEADING_TOLERANCE_DEG, now_ms)) {
      return REMOTE_ROUTE_MOTOR_FAULT;
    }
    return REMOTE_ROUTE_WAITING;
  }
  if ((uint32_t)(now_ms - step_started_ms) < APP_NAV_TURN_SETTLE_MS) {
    return REMOTE_ROUTE_WAITING;
  }

  if (!nav_forward_active) {
    nav_forward_active = true;
    nav_payload_change_ms = now_ms;
  } else if ((uint32_t)(now_ms - nav_payload_change_ms) >=
             APP_NAV_REMOTE_PROGRESS_TIMEOUT_MS) {
    nav_payload_stale = true;
    task_status.nav_stale = true;
  }
  if (nav_payload_stale) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return REMOTE_ROUTE_WAITING;
  }

  const float target_speed_mm_s = task_remote_route_speed(
      command->target_x_mm, cruise_speed_mm_s);
  const float target_yaw_mm_s =
      task_remote_heading_correction(heading_error_deg);
  const float speed_rate_mm_s2 =
      (target_speed_mm_s >= remote_speed_mm_s) ?
      APP_NAV_SPEED_ACCEL_MM_S2 : APP_NAV_SPEED_DECEL_MM_S2;
  remote_speed_mm_s = task_step_toward(
      remote_speed_mm_s, target_speed_mm_s,
      speed_rate_mm_s2 * APP_TASK_PERIOD_MS * 0.001f);
  remote_yaw_mm_s = task_step_toward(
      remote_yaw_mm_s, target_yaw_mm_s,
      APP_NAV_YAW_ACCEL_MM_S2 * APP_TASK_PERIOD_MS * 0.001f);
  Motor_Move(remote_speed_mm_s, 0.0f, remote_yaw_mm_s);
  task_status.motors_active = true;
  return REMOTE_ROUTE_RUNNING;
}

static void task_process_navigation(const VisionData *vision,
                                    uint32_t now_ms)
{
  const bool navigating = state == TASK_NAVIGATE;
  const bool command_fresh = Vision_MissionIsFresh(
      &vision->mission, now_ms, APP_MISSION_COMMAND_TIMEOUT_MS);

  /* Dynamic RDK remaining distance makes every STOP resumable. */
  if (navigating &&
      (vision->mission.command == VISION_CMD_STOP)) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return;
  }

  if (!command_fresh) {
    /* Never move on an expired externally closed-loop target. */
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return;
  }
  if (vision->mission.command == VISION_CMD_ALIGN_SAFE_ZONE) {
    /* Old RDK software may still send ALIGN. Do not rotate or fault; hold NAV
     * until the updated planner sends ENTER_SAFE_ZONE directly. */
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return;
  }

  if (vision->mission.command != VISION_CMD_NAVIGATE_WAYPOINT) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
    return;
  }
  const RemoteRouteStatus result = task_follow_remote_route(
      &vision->mission, VISION_CMD_NAVIGATE_WAYPOINT,
      APP_NAV_FAST_SPEED_MM_S, now_ms);
  if (result == REMOTE_ROUTE_COMMAND_INVALID) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
  } else if (result == REMOTE_ROUTE_MOTOR_FAULT) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
}

static bool distance_failed(MotorDistanceStatus result)
{
  return (result == MOTOR_DISTANCE_FAULT) ||
         (result == MOTOR_DISTANCE_INVALID);
}

static bool delivery_enter_command_ok(const VisionMissionCommand *command,
                                      uint32_t now_ms)
{
  return Vision_MissionIsFresh(command, now_ms,
                               APP_MISSION_COMMAND_TIMEOUT_MS) &&
         (command->command == VISION_CMD_ENTER_SAFE_ZONE);
}

static void task_process_delivery_verify(
    const VisionMissionCommand *command, uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS) ||
      ((command->command != VISION_CMD_ENTER_SAFE_ZONE) &&
       (command->command != VISION_CMD_TASK_COMPLETE))) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) <
      APP_DELIVERY_VERIFY_WAIT_MS) {
    return;
  }
  if (command->command == VISION_CMD_TASK_COMPLETE) {
    task_enter(TASK_EXIT_SAFE_ZONE, now_ms);
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
  const VisionMissionCommand *command = &vision->mission;
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS) ||
      (command->command != VISION_CMD_RETURN_CENTER)) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return;
  }
  const RemoteRouteStatus result = task_follow_remote_route(
      command, VISION_CMD_RETURN_CENTER,
      APP_RETURN_CENTER_SPEED_MM_S, now_ms);
  if (result == REMOTE_ROUTE_REACHED) {
    search_start_low = true;
    task_enter(TASK_SEARCH, now_ms);
  } else if (result == REMOTE_ROUTE_COMMAND_INVALID) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
  } else if (result == REMOTE_ROUTE_MOTOR_FAULT) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
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

  task_status.last_command = command->command;
  task_status.command_received = true;

  if (command->command == VISION_CMD_ABORT) {
    task_status.acknowledged_sequence = command->sequence;
    task_stop(TASK_FAULT_REMOTE_STOP, now_ms);
  } else if ((command->command == VISION_CMD_STOP) &&
             (state == TASK_NAVIGATE)) {
    /* A fresh external remaining distance makes every NAV hold resumable. */
    task_status.acknowledged_sequence = command->sequence;
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
  } else if (command->command == VISION_CMD_STOP) {
    task_status.acknowledged_sequence = command->sequence;
    task_stop(TASK_FAULT_REMOTE_STOP, now_ms);
  } else if (state == TASK_STOPPED) {
    return;
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             (state >= TASK_GRAB_OBSERVE) &&
             (state <= TASK_GRAB_ROTATE)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_CLOSE_CLAW, now_ms);
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             ((state == TASK_CLOSE_CLAW) ||
              (state == TASK_WAIT_NAVIGATION))) {
    /* GRAB_CONFIRMED is now repeated until GRIPPER_CLOSED is reported.  ACK
     * every new sequence, but never restart an in-progress/completed motion. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             task_status.gripper_closed &&
             (state == TASK_WAIT_NAVIGATION)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_NAVIGATE, now_ms);
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             (state == TASK_NAVIGATE)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (state == TASK_NAVIGATE)) {
    /* Compatibility hold only; ALIGN no longer changes Task state. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
              task_status.gripper_closed &&
              (state == TASK_NAVIGATE)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_OPEN_FOR_RAM, now_ms);
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
             ((state == TASK_OPEN_FOR_RAM) ||
              (state == TASK_RAM_VERIFY))) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_TASK_COMPLETE) &&
             !task_status.gripper_closed &&
             (state == TASK_RAM_VERIFY)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_FACE_FIELD_CENTER)) {
    task_status.acknowledged_sequence = command->sequence;
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
      task_process_navigation(&vision, now_ms);
      break;

    case TASK_ALIGN_SAFE_ZONE:
      /* Reserved legacy state; normal control never enters it. */
      Motor_Stop();
      task_status.motors_active = false;
      break;

    case TASK_OPEN_FOR_RAM:
      if (!delivery_enter_command_ok(&vision.mission, now_ms)) {
        task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
      } else if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_enter(TASK_RAM_VERIFY, now_ms);
      }
      break;

    case TASK_RAM_VERIFY:
      task_process_delivery_verify(&vision.mission, now_ms);
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
