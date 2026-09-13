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

typedef enum {
  SEARCH_CAMERA_TO_90 = 0,
  SEARCH_HOLD_90,
  SEARCH_SWEEP_90,
  SEARCH_CAMERA_TO_120,
  SEARCH_HOLD_120,
  SEARCH_SWEEP_120
} SearchPhase;

typedef enum {
  APPROACH_TRACK = 0,
  APPROACH_CLUSTER_ALIGN,
  APPROACH_ALIGN_125,
  APPROACH_CAMERA_TO_140,
  APPROACH_ADVANCE_140,
  APPROACH_RAISE_REACQUIRE
} ApproachPhase;

typedef enum {
  REMOTE_ROUTE_WAITING = 0,
  REMOTE_ROUTE_RUNNING,
  REMOTE_ROUTE_REACHED,
  REMOTE_ROUTE_COMMAND_INVALID,
  REMOTE_ROUTE_MOTOR_FAULT
} RemoteRouteStatus;

typedef enum {
  REMOTE_ACTION_NONE = 0,
  REMOTE_ACTION_YIELD,
  REMOTE_ACTION_ESCAPE,
  REMOTE_ACTION_RELEASE_LEFT,
  REMOTE_ACTION_RELEASE_RIGHT,
  REMOTE_ACTION_RELEASE_BOTH,
  REMOTE_ACTION_DISPERSE,
  REMOTE_ACTION_LANE
} RemoteAction;

typedef struct {
  RemoteAction type;
  uint8_t phase;
  uint8_t command;
  int16_t arg_a;
  int16_t arg_b;
  uint32_t started_ms;
  uint32_t phase_path_mm;
  int32_t reference_heading_mdeg;
  bool done;
  bool paused;
  bool reference_heading_valid;
  uint32_t paused_ms;
} RemoteActionState;

typedef struct {
  int16_t remaining_mm;
  int32_t heading_mdeg;
  uint32_t path_mm;
  uint32_t changed_ms;
  bool valid;
  bool stale;
} NavProgress;

static volatile TaskStatus task_status;
static Pid_t steering_pid;
static Pid_t camera_pid;
static TaskState state;
static TurnTracker turn_tracker;
static SearchPhase search_phase;
static ApproachPhase approach_phase;
static uint32_t state_started_ms;
static uint32_t step_started_ms;
static uint32_t status_sent_ms;
static uint32_t pose_invalid_started_ms;
static uint32_t nav_terminal_candidate_ms;
static uint32_t nav_terminal_latched_ms;
static uint32_t nav_final_push_started_ms;
static uint32_t nav_final_push_paused_ms;
static float camera_angle;
static float steering_mm_s;
static float steering_target_mm_s;
static float approach_speed_mm_s;
static float approach_locked_heading_deg;
static float filtered_target_x;
static float filtered_target_y;
static float remote_speed_mm_s;
static float remote_yaw_mm_s;
static float nav_locked_heading_deg;
static uint32_t tracking_tick_ms;
static uint32_t approach_report_generation;
static uint32_t approach_phase_path_mm;
static uint32_t approach_reacquire_generation;
static uint32_t tracking_report_generation;
static uint8_t mission_sequence;
static uint32_t start_reverse_path_mm;
static float start_target_heading_deg;
static uint32_t scan_entry_report_generation;
static uint32_t nav_final_push_start_path_mm;
static uint8_t locked_cargo_counts;
static uint8_t configured_color;
static bool initialized;
static bool initial_claw_ready;
static bool tracking_valid;
static bool tracking_filter_valid;
static bool approach_report_generation_valid;
static bool mission_sequence_valid;
static bool steering_active;
static bool nav_ready;
static bool scan_report_gate_open;
static bool pose_invalid_pending;
static bool start_clearance_done;
static bool distance_command_done;
static bool nav_terminal_candidate;
static bool nav_terminal_latched;
static bool nav_forward_active;
static bool nav_heading_locked;
static bool nav_final_push_active;
static bool nav_final_push_done;
static bool nav_final_push_paused;
static bool first_delivery_done;
static bool complete_flow_active;
static bool mission_paused;
static bool audit_received;
static bool audit_valid;
static bool audit_initial_stash;
static bool audit_destination_injury;
static bool cargo_recheck_pending;
static bool route_to_stash;
static bool stash_backoff_pending;
static bool cluster_target_active;
static uint8_t audit_consistent_count;
static uint8_t audit_last_left_class;
static uint8_t audit_last_right_class;
static uint8_t audit_last_counts;
static uint8_t audit_last_flags;
static uint8_t audit_last_total_count;
static uint8_t remote_target_sequence;
static uint32_t remote_target_generation;
static bool remote_target_sequence_valid;
static RemoteActionState remote_action;
static NavProgress nav_progress;

static void task_enter(TaskState next, uint32_t now_ms);
static bool distance_failed(MotorDistanceStatus result);
static bool delivery_enter_command_ok(const VisionMissionCommand *command);
static float nav_yaw(float heading_error_deg);

static void nav_reset_progress(uint32_t now_ms)
{
  nav_progress = (NavProgress){.changed_ms = now_ms};
}

static float task_abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static bool task_mission_valid(const VisionMissionCommand *command)
{
  return (command != NULL) && command->received &&
         ((command->flags & VISION_CMD_VALID) != 0U);
}

static bool task_report_valid(const VisionData *vision)
{
  return (vision != NULL) && vision->valid && (vision->tick_ms != 0U);
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

static float task_safe_zone_heading(void)
{
  float heading_deg = (configured_color == VISION_COLOR_RED) ?
      APP_SAFE_ZONE_RED_HEADING_DEG : APP_SAFE_ZONE_BLUE_HEADING_DEG;
  if (!first_delivery_done) {
    heading_deg += (configured_color == VISION_COLOR_RED) ?
        APP_FIRST_DELIVERY_RED_OFFSET_DEG :
        APP_FIRST_DELIVERY_BLUE_OFFSET_DEG;
  }
  if (heading_deg < 0.0f) {
    heading_deg += 360.0f;
  }
  if (heading_deg >= 360.0f) {
    heading_deg -= 360.0f;
  }
  return heading_deg;
}

static bool task_side_flag_valid(uint8_t flags)
{
  if (configured_color == VISION_COLOR_RED) {
    return (flags & VISION_CMD_RED_SIDE) != 0U;
  }
  if (configured_color == VISION_COLOR_BLUE) {
    return (flags & VISION_CMD_RED_SIDE) == 0U;
  }
  return false;
}

static void task_pause_final_push(uint32_t now_ms)
{
  if (nav_final_push_active && !nav_final_push_paused) {
    nav_final_push_paused = true;
    nav_final_push_paused_ms = now_ms;
  }
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
  steering_target_mm_s = 0.0f;
  Pid_Reset(&steering_pid);
  Pid_Reset(&camera_pid);
}

static bool task_yield_allowed(void)
{
  if ((state == TASK_APPROACH) || (state == TASK_NAVIGATE) ||
      (state == TASK_FACE_FIELD_CENTER)) {
    return true;
  }
  if ((state != TASK_REMOTE_ACTION) || !remote_action.done) {
    return false;
  }
  if ((remote_action.type == REMOTE_ACTION_RELEASE_BOTH) &&
      (remote_action.arg_b == 1)) {
    /* The ambiguous-side branch already performs its own backoff, impact and
     * return.  It finishes empty and goes back to SEARCH, never YIELD. */
    return false;
  }
  return (remote_action.type == REMOTE_ACTION_RELEASE_LEFT) ||
         (remote_action.type == REMOTE_ACTION_RELEASE_RIGHT) ||
         (remote_action.type == REMOTE_ACTION_RELEASE_BOTH) ||
         (remote_action.type == REMOTE_ACTION_ESCAPE);
}

static bool task_escape_allowed(void)
{
  return (state == TASK_APPROACH) || (state == TASK_NAVIGATE) ||
         (state == TASK_FACE_FIELD_CENTER) ||
         ((state == TASK_REMOTE_ACTION) && remote_action.done &&
          (remote_action.type == REMOTE_ACTION_YIELD));
}

static bool task_lane_allowed(void)
{
  return (state == TASK_APPROACH) || (state == TASK_NAVIGATE);
}

static void task_reset_turn_tracker(void)
{
  turn_tracker.last_heading_mdeg = 0;
  turn_tracker.accumulated_mdeg = 0U;
  turn_tracker.valid = false;
}

static bool task_turn_magnitude_reached(uint32_t target_mdeg)
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
  return turn_tracker.accumulated_mdeg >= target_mdeg;
}

static bool task_full_turn_reached(void)
{
  return task_turn_magnitude_reached(APP_SEARCH_FULL_TURN_MDEG);
}

static uint8_t task_protocol_mode(void)
{
  if (complete_flow_active) {
    if (state == TASK_APPROACH) {
      return 20U;
    }
    if (state == TASK_DISPERSE_READY) {
      return 37U;
    }
    if ((state >= TASK_GRAB_OBSERVE) && (state <= TASK_GRAB_ROTATE)) {
      return 21U;
    }
    if (state == TASK_WAIT_NAVIGATION) {
      return 22U;
    }
    if ((state == TASK_REMOTE_ACTION) && remote_action.done) {
      switch (remote_action.type) {
        case REMOTE_ACTION_YIELD:        return 30U;
        case REMOTE_ACTION_ESCAPE:       return 31U;
        case REMOTE_ACTION_RELEASE_LEFT: return 32U;
        case REMOTE_ACTION_RELEASE_RIGHT:return 33U;
        case REMOTE_ACTION_RELEASE_BOTH: return 34U;
        case REMOTE_ACTION_DISPERSE:     return 35U;
        case REMOTE_ACTION_LANE:         return 36U;
        default:                         break;
      }
    }
  }
  return (uint8_t)state;
}

static bool task_audit_class_is(uint8_t value, uint8_t cargo)
{
  return value == cargo;
}

static bool task_audit_has_mixed_material(uint8_t left, uint8_t right)
{
  const bool has_green = task_audit_class_is(left, VISION_CARGO_GREEN) ||
                         task_audit_class_is(right, VISION_CARGO_GREEN);
  const bool has_core = task_audit_class_is(left, VISION_CARGO_CORE) ||
                        task_audit_class_is(right, VISION_CARGO_CORE);
  return task_audit_class_is(left, VISION_CARGO_MIXED_MATERIAL) ||
         task_audit_class_is(right, VISION_CARGO_MIXED_MATERIAL) ||
         (has_green && has_core);
}

static bool task_validate_audit(const VisionMissionCommand *command)
{
  const uint8_t left_count = command->audit_counts & 0x03U;
  const uint8_t right_count = (command->audit_counts >> 2) & 0x03U;
  const uint8_t total = command->audit_total_count;
  const uint8_t left = command->audit_left_class;
  const uint8_t right = command->audit_right_class;
  const bool dangerous =
      ((command->audit_flags & VISION_AUDIT_DANGER_PRESENT) != 0U) ||
      task_audit_class_is(left, VISION_CARGO_DANGER) ||
      task_audit_class_is(right, VISION_CARGO_DANGER);
  const bool unknown =
      ((command->audit_flags & VISION_AUDIT_UNKNOWN_PRESENT) != 0U) ||
      task_audit_class_is(left, VISION_CARGO_UNKNOWN) ||
      task_audit_class_is(right, VISION_CARGO_UNKNOWN);
  const bool injury = task_audit_class_is(left, VISION_CARGO_INJURED) ||
                      task_audit_class_is(right, VISION_CARGO_INJURED);
  const bool injury_mixed =
      ((command->audit_flags & VISION_AUDIT_INJURY_MIXED) != 0U) ||
      (injury && (total != 1U));

  /* The opening temporary stash only clears the centre pile. Per-side counts
   * are saturated diagnostics and neither category nor quantity is a formal
   * delivery interlock for this move; only a non-empty stable audit matters. */
  if (audit_initial_stash) {
    return total > 0U;
  }

  if ((total == 0U) || (total > 3U) ||
      ((uint8_t)(left_count + right_count) != total) ||
      dangerous || unknown || injury_mixed) {
    return false;
  }
  if (!first_delivery_done) {
    return (total == 1U) &&
           ((task_audit_class_is(left, VISION_CARGO_GREEN) &&
             (left_count == 1U)) ||
            (task_audit_class_is(right, VISION_CARGO_GREEN) &&
             (right_count == 1U)));
  }
  if (audit_destination_injury) {
    return (total == 1U) && injury;
  }
  /* The current mechanism cannot unload green and core cargo together.
   * Report the audit as invalid so the upper computer can release one side,
   * back off, re-audit, and deliver the retained single cargo normally. */
  return !injury && !task_audit_has_mixed_material(left, right) &&
         ((left == VISION_CARGO_NONE) ||
          (left == VISION_CARGO_GREEN) ||
          (left == VISION_CARGO_CORE)) &&
         ((right == VISION_CARGO_NONE) ||
          (right == VISION_CARGO_GREEN) ||
          (right == VISION_CARGO_CORE));
}

static bool task_release_command_valid(uint8_t command)
{
  const uint8_t left_count = audit_last_counts & 0x03U;
  const uint8_t right_count = (audit_last_counts >> 2) & 0x03U;

  if (!audit_received || (audit_last_total_count == 0U)) {
    return false;
  }
  if (cargo_recheck_pending) {
    return command == VISION_CMD_RELEASE_BOTH;
  }
  if (command == VISION_CMD_RELEASE_LEFT) {
    return left_count > 0U;
  }
  if (command == VISION_CMD_RELEASE_RIGHT) {
    return right_count > 0U;
  }
  return command == VISION_CMD_RELEASE_BOTH;
}

static void task_latch_audit(const VisionMissionCommand *command)
{
  const uint8_t semantic_flags = command->audit_flags &
      (uint8_t)~VISION_AUDIT_STABLE;
  const bool same =
      (command->audit_left_class == audit_last_left_class) &&
      (command->audit_right_class == audit_last_right_class) &&
      (command->audit_counts == audit_last_counts) &&
      (semantic_flags == audit_last_flags) &&
      (command->audit_total_count == audit_last_total_count);
  if (same) {
    if (audit_consistent_count < 255U) {
      ++audit_consistent_count;
    }
  } else {
    audit_last_left_class = command->audit_left_class;
    audit_last_right_class = command->audit_right_class;
    audit_last_counts = command->audit_counts;
    audit_last_flags = semantic_flags;
    audit_last_total_count = command->audit_total_count;
    audit_consistent_count = 1U;
  }
  audit_initial_stash =
      (command->audit_flags & VISION_AUDIT_INITIAL_STASH) != 0U;
  audit_destination_injury =
      (command->audit_flags & VISION_AUDIT_DESTINATION_INJURY) != 0U;
  task_status.audit_left_class = command->audit_left_class;
  task_status.audit_right_class = command->audit_right_class;
  task_status.audit_total_count = command->audit_total_count;
  /* bdcf0f2 switches to GRAB on its third matching camera frame, so only the
   * first two non-STABLE audit frames reach the UART. Accept those two equal
   * payloads; a future explicit STABLE frame is accepted immediately. */
  audit_received = (audit_consistent_count >= 2U) ||
      ((command->audit_flags & VISION_AUDIT_STABLE) != 0U);
  audit_valid = audit_received && task_validate_audit(command);
  task_status.audit_ready = audit_received;
  task_status.audit_valid = audit_valid;
}

static void task_clear_audit_result(void)
{
  audit_received = false;
  audit_valid = false;
  audit_consistent_count = 0U;
  audit_last_left_class = 0U;
  audit_last_right_class = 0U;
  audit_last_counts = 0U;
  audit_last_flags = 0U;
  audit_last_total_count = 0U;
  task_status.audit_left_class = 0U;
  task_status.audit_right_class = 0U;
  task_status.audit_total_count = 0U;
  task_status.audit_ready = false;
  task_status.audit_valid = false;
}

static void task_apply_complete_target(VisionData *vision)
{
  if (!complete_flow_active) {
    return;
  }
  const VisionMissionCommand *command = &vision->mission;
  if (task_mission_valid(command) &&
      (command->command == VISION_CMD_APPROACH_TARGET)) {
    if (!remote_target_sequence_valid ||
        (remote_target_sequence != command->sequence)) {
      remote_target_sequence = command->sequence;
      remote_target_sequence_valid = true;
      ++remote_target_generation;
    }
    vision->x = (uint16_t)command->target_x_mm;
    vision->y = (uint16_t)command->target_y_mm;
    vision->distance_mm = 0U;
    vision->tick_ms = command->tick_ms;
    vision->report_generation = remote_target_generation;
    vision->cargo_counts = 1U;
    vision->found = true;
    vision->classification_valid = true;
    vision->distance_valid = false;
    vision->valid = true;
  } else {
    vision->found = false;
  }
}

static void task_start_remote_action(RemoteAction type,
                                     const VisionMissionCommand *command,
                                     uint32_t now_ms)
{
  const LocationPose pose = Location_GetPose();
  task_enter(TASK_REMOTE_ACTION, now_ms);
  remote_action = (RemoteActionState){
    .type = type,
    .phase = 0U,
    .command = command->command,
    .arg_a = command->target_x_mm,
    .arg_b = command->target_y_mm,
    .started_ms = now_ms,
    .phase_path_mm = pose.path_mm,
    .reference_heading_mdeg = pose.heading_mdeg,
    .done = false,
    .paused = false,
    .reference_heading_valid = pose.valid,
    .paused_ms = now_ms
  };
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
    .mode = task_protocol_mode(),
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
  task_status.nav_final_push = false;
  task_status.nav_heading_locked = false;
  task_status.nav_locked_heading_deg = 0U;
  task_status.claw_visible =
      (next >= TASK_GRAB_OBSERVE) && (next <= TASK_CLOSE_CLAW);
  state_started_ms = now_ms;
  step_started_ms = now_ms;
  pose_invalid_pending = false;
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
  nav_locked_heading_deg = 0.0f;
  nav_heading_locked = false;
  nav_final_push_active = false;
  nav_final_push_done = false;
  nav_final_push_paused = false;
  nav_terminal_candidate = false;
  nav_terminal_latched = false;

  if (next == TASK_START) {
    const LocationPose pose = Location_GetPose();
    start_reverse_path_mm = pose.path_mm;
    start_target_heading_deg = task_wrap_angle(
        (float)pose.heading_mdeg * 0.001f + APP_START_TURN_DEG);
    start_clearance_done = false;
  } else if (next == TASK_SEARCH) {
    const VisionData vision = Vision_GetSnapshot();
    camera_angle = (float)Camera_GetAngle();
    task_status.found = false;
    locked_cargo_counts = 0U;
    search_phase = SEARCH_CAMERA_TO_90;
    scan_entry_report_generation = vision.report_generation;
    scan_report_gate_open = false;
    audit_received = false;
    audit_valid = false;
    audit_initial_stash = false;
    audit_destination_injury = false;
    cargo_recheck_pending = false;
    route_to_stash = false;
    stash_backoff_pending = false;
    cluster_target_active = false;
    audit_consistent_count = 0U;
    audit_last_left_class = 0U;
    audit_last_right_class = 0U;
    audit_last_counts = 0U;
    audit_last_flags = 0U;
    audit_last_total_count = 0U;
    task_status.audit_left_class = 0U;
    task_status.audit_right_class = 0U;
    task_status.audit_total_count = 0U;
    task_status.audit_ready = false;
    task_status.audit_valid = false;
    task_reset_tracking();
    task_reset_turn_tracker();
  } else if (next == TASK_APPROACH) {
    camera_angle = (float)Camera_GetAngle();
    approach_speed_mm_s = APP_APPROACH_SPEED_MM_S;
    approach_phase = APPROACH_TRACK;
    approach_phase_path_mm = 0U;
    approach_reacquire_generation = 0U;
    approach_locked_heading_deg = 0.0f;
    approach_report_generation_valid = false;
    task_reset_tracking();
  } else if (next == TASK_APPROACH_RECOVER) {
    const VisionData vision = Vision_GetSnapshot();
    scan_entry_report_generation = vision.report_generation;
    scan_report_gate_open = false;
    task_reset_tracking();
  } else if (next == TASK_GRAB_OBSERVE) {
    task_reset_tracking();
  } else if (next == TASK_GRAB_ROTATE) {
    task_reset_turn_tracker();
  } else if ((next == TASK_NAVIGATE) ||
             (next == TASK_FACE_FIELD_CENTER)) {
    nav_ready = false;
    distance_command_done = false;
    nav_reset_progress(now_ms);
    nav_forward_active = false;
    remote_speed_mm_s = 0.0f;
    remote_yaw_mm_s = 0.0f;
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
  nav_terminal_candidate_ms = now_ms;
  nav_terminal_latched_ms = now_ms;
  nav_final_push_started_ms = now_ms;
  nav_final_push_paused_ms = now_ms;
  camera_angle = (float)Camera_GetAngle();
  steering_mm_s = 0.0f;
  steering_target_mm_s = 0.0f;
  approach_speed_mm_s = APP_APPROACH_SPEED_MM_S;
  approach_locked_heading_deg = 0.0f;
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
  nav_locked_heading_deg = 0.0f;
  tracking_report_generation = 0U;
  tracking_tick_ms = 0U;
  approach_report_generation = 0U;
  approach_phase_path_mm = 0U;
  approach_reacquire_generation = 0U;
  mission_sequence = 0U;
  start_reverse_path_mm = 0U;
  start_target_heading_deg = 0.0f;
  scan_entry_report_generation = 0U;
  nav_final_push_start_path_mm = 0U;
  locked_cargo_counts = 0U;
  configured_color = 0U;
  initial_claw_ready = false;
  tracking_valid = false;
  tracking_filter_valid = false;
  approach_report_generation_valid = false;
  mission_sequence_valid = false;
  steering_active = false;
  nav_ready = false;
  scan_report_gate_open = false;
  pose_invalid_pending = false;
  start_clearance_done = false;
  distance_command_done = false;
  nav_reset_progress(now_ms);
  nav_terminal_candidate = false;
  nav_terminal_latched = false;
  nav_forward_active = false;
  nav_heading_locked = false;
  nav_final_push_active = false;
  nav_final_push_done = false;
  nav_final_push_paused = false;
  first_delivery_done = false;
  complete_flow_active = false;
  mission_paused = false;
  audit_received = false;
  audit_valid = false;
  audit_initial_stash = false;
  audit_destination_injury = false;
  cargo_recheck_pending = false;
  route_to_stash = false;
  stash_backoff_pending = false;
  cluster_target_active = false;
  audit_consistent_count = 0U;
  audit_last_left_class = 0U;
  audit_last_right_class = 0U;
  audit_last_counts = 0U;
  audit_last_flags = 0U;
  audit_last_total_count = 0U;
  remote_target_sequence = 0U;
  remote_target_generation = 0U;
  remote_target_sequence_valid = false;
  remote_action = (RemoteActionState){0};
  search_phase = SEARCH_CAMERA_TO_90;
  approach_phase = APPROACH_TRACK;
  task_reset_turn_tracker();
  initialized = true;
}

static bool task_target_is_single_cargo(const VisionData *vision)
{
  const uint8_t count =
      VISION_COUNT_NORMAL(vision->cargo_counts) +
      VISION_COUNT_CORE(vision->cargo_counts) +
      VISION_COUNT_CASUALTY(vision->cargo_counts) +
      VISION_COUNT_DANGER(vision->cargo_counts);
  return task_report_valid(vision) && vision->found &&
         vision->classification_valid &&
         (count == 1U);
}

static bool task_target_matches_lock(const VisionData *vision)
{
  return (locked_cargo_counts != 0U) &&
         task_target_is_single_cargo(vision) &&
         (vision->cargo_counts == locked_cargo_counts);
}

static bool task_scan_report_ready(const VisionData *vision)
{
  if (!scan_report_gate_open &&
      (vision->report_generation != scan_entry_report_generation) &&
      task_report_valid(vision)) {
    scan_report_gate_open = true;
  }
  return scan_report_gate_open;
}

static bool task_scan_target_found(const VisionData *vision)
{
  return task_scan_report_ready(vision) &&
         task_target_is_single_cargo(vision);
}

static bool task_scan_locked_target_found(const VisionData *vision)
{
  return task_scan_report_ready(vision) &&
         task_target_matches_lock(vision);
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

static float task_track_target(const VisionData *vision, bool track_camera)
{
  /* Legacy target reports and TYPE=0x18 APPROACH_TARGET commands both advance
   * report_generation, but they do not share one 8-bit sequence field. */
  if (tracking_valid &&
      (vision->report_generation == tracking_report_generation)) {
    return steering_target_mm_s;
  }
  const float dt_s = task_tracking_dt(vision->tick_ms);
  tracking_report_generation = vision->report_generation;
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

  if (track_camera) {
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
      /* Larger servo-3 angles look farther down. */
      if (camera_angle < 90.0f) {
        camera_angle = 90.0f;
      } else if (camera_angle > 165.0f) {
        camera_angle = 165.0f;
      }
      Camera_SetAngle((uint8_t)(camera_angle + 0.5f));
    }
  } else {
    Pid_Reset(&camera_pid);
  }

  const float x_error = (float)APP_VISION_TARGET_X - filtered_target_x;
  const float magnitude = task_abs(x_error);
  if ((steering_active &&
       (magnitude <= APP_STEERING_EXIT_DEAD_ZONE)) ||
      (!steering_active &&
       (magnitude <= APP_STEERING_ENTER_DEAD_ZONE))) {
    steering_active = false;
    steering_target_mm_s = 0.0f;
    Pid_Reset(&steering_pid);
    return steering_target_mm_s;
  }

  steering_active = true;
  steering_target_mm_s = Pid_UpdateDt(&steering_pid,
                                      (float)APP_VISION_TARGET_X,
                                      filtered_target_x, dt_s) *
                         APP_STEERING_DIRECTION;
  if ((steering_target_mm_s > 0.0f) &&
      (steering_target_mm_s < APP_STEERING_MIN_MM_S)) {
    steering_target_mm_s = APP_STEERING_MIN_MM_S;
  } else if ((steering_target_mm_s < 0.0f) &&
             (steering_target_mm_s > -APP_STEERING_MIN_MM_S)) {
    steering_target_mm_s = -APP_STEERING_MIN_MM_S;
  }
  return steering_target_mm_s;
}

static void task_process_start(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - state_started_ms) >= APP_START_TIMEOUT_MS) {
    task_stop(TASK_FAULT_START_TIMEOUT, now_ms);
    return;
  }

  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
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

static void task_process_search(const VisionData *vision, uint32_t now_ms)
{
  /* A report cached before this SEARCH epoch remains visible on the LCD, but
   * cannot select a target until a new valid report generation arrives. */
  task_status.found = task_scan_target_found(vision);
  if (task_status.found) {
    locked_cargo_counts = vision->cargo_counts;
    task_enter(TASK_APPROACH, now_ms);
    return;
  }

  switch (search_phase) {
    case SEARCH_CAMERA_TO_90:
      Motor_Stop();
      task_status.motors_active = false;
      if (task_scan_camera_to(APP_SEARCH_LOW_CAMERA_ANGLE, now_ms)) {
        search_phase = SEARCH_HOLD_90;
        step_started_ms = now_ms;
      }
      return;

    case SEARCH_HOLD_90:
      Motor_Stop();
      task_status.motors_active = false;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_CAMERA_SCAN_ENDPOINT_HOLD_MS) {
        search_phase = SEARCH_SWEEP_90;
        task_reset_turn_tracker();
      }
      return;

    case SEARCH_SWEEP_90:
      if (task_full_turn_reached()) {
        Motor_Stop();
        task_status.motors_active = false;
        search_phase = SEARCH_CAMERA_TO_120;
        task_reset_turn_tracker();
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;

    case SEARCH_CAMERA_TO_120:
      Motor_Stop();
      task_status.motors_active = false;
      if (task_scan_camera_to(APP_SEARCH_HIGH_CAMERA_ANGLE, now_ms)) {
        search_phase = SEARCH_HOLD_120;
        step_started_ms = now_ms;
      }
      return;

    case SEARCH_HOLD_120:
      Motor_Stop();
      task_status.motors_active = false;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_CAMERA_SCAN_ENDPOINT_HOLD_MS) {
        search_phase = SEARCH_SWEEP_120;
        task_reset_turn_tracker();
      }
      return;

    case SEARCH_SWEEP_120:
      if (task_full_turn_reached()) {
        Motor_Stop();
        task_status.motors_active = false;
        /* The upper computer owns the decision made after a complete 720 deg
         * two-height scan. Stay in SEARCH and begin another scan instead of
         * starting an unrelated local return-to-centre route. */
        search_phase = SEARCH_CAMERA_TO_90;
        task_reset_turn_tracker();
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;

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

static float task_approach_aligned_speed(float requested_speed_mm_s)
{
  if (!tracking_filter_valid) {
    return requested_speed_mm_s;
  }
  const float error_px = task_abs(
      (float)APP_VISION_TARGET_X - filtered_target_x);
  if (error_px <= APP_APPROACH_FULL_SPEED_ERROR_PX) {
    return requested_speed_mm_s;
  }

  float speed_limit_mm_s = APP_APPROACH_ALIGN_MIN_SPEED_MM_S;
  if (error_px < APP_APPROACH_SLOW_SPEED_ERROR_PX) {
    const float ratio = (APP_APPROACH_SLOW_SPEED_ERROR_PX - error_px) /
        (APP_APPROACH_SLOW_SPEED_ERROR_PX -
         APP_APPROACH_FULL_SPEED_ERROR_PX);
    speed_limit_mm_s +=
        (APP_APPROACH_SPEED_MM_S - APP_APPROACH_ALIGN_MIN_SPEED_MM_S) *
        ratio;
  }
  return (requested_speed_mm_s < speed_limit_mm_s) ?
      requested_speed_mm_s : speed_limit_mm_s;
}

static float task_approach_steering(void)
{
  steering_mm_s = task_step_toward(
      steering_mm_s, steering_target_mm_s,
      APP_STEERING_RATE_MM_S2 * APP_TASK_PERIOD_MS * 0.001f);
  return steering_mm_s;
}

static void task_process_approach(const VisionData *vision, uint32_t now_ms)
{
  task_status.auto_approach = true;
  const bool new_report = !approach_report_generation_valid ||
      (vision->report_generation != approach_report_generation);
  const bool new_locked_target = new_report &&
      task_target_matches_lock(vision);
  if (new_report) {
    approach_report_generation = vision->report_generation;
    approach_report_generation_valid = true;
  }

  switch (approach_phase) {
    case APPROACH_TRACK:
      if (new_locked_target) {
        approach_speed_mm_s = task_approach_speed(vision);
        task_status.found = true;
        (void)task_track_target(vision, true);
      }
      if (cluster_target_active) {
        if (Camera_GetAngle() >= APP_DISPERSE_APPROACH_ANGLE) {
          Camera_SetAngle(APP_DISPERSE_APPROACH_ANGLE);
          camera_angle = (float)APP_DISPERSE_APPROACH_ANGLE;
          approach_phase = APPROACH_CLUSTER_ALIGN;
          Motor_Stop();
          task_status.motors_active = false;
          return;
        }
        const float cluster_speed =
            (approach_speed_mm_s < APP_DISPERSE_APPROACH_SPEED_MM_S) ?
                approach_speed_mm_s : APP_DISPERSE_APPROACH_SPEED_MM_S;
        Motor_Move(task_approach_aligned_speed(cluster_speed),
                   0.0f, task_approach_steering());
        task_status.motors_active = true;
        return;
      }
      if (Camera_GetAngle() >= APP_GRAB_ALIGN_CAMERA_ANGLE) {
        Camera_SetAngle(APP_GRAB_ALIGN_CAMERA_ANGLE);
        camera_angle = (float)APP_GRAB_ALIGN_CAMERA_ANGLE;
        approach_phase = APPROACH_ALIGN_125;
        Motor_Stop();
        task_status.motors_active = false;
        return;
      }
      Motor_Move(task_approach_aligned_speed(approach_speed_mm_s),
                 0.0f, task_approach_steering());
      task_status.motors_active = true;
      return;

    case APPROACH_CLUSTER_ALIGN:
      Camera_SetAngle(APP_DISPERSE_APPROACH_ANGLE);
      camera_angle = (float)APP_DISPERSE_APPROACH_ANGLE;
      if (new_locked_target) {
        task_status.found = true;
        (void)task_track_target(vision, false);
      }
      if (tracking_filter_valid &&
          (task_abs(filtered_target_x - (float)APP_VISION_TARGET_X) <=
           APP_DISPERSE_ALIGN_X_ERROR_PX)) {
        Motor_Stop();
        task_status.motors_active = false;
        task_enter(TASK_DISPERSE_READY, now_ms);
        return;
      }
      Motor_Move(0.0f, 0.0f, task_approach_steering());
      task_status.motors_active = task_abs(steering_mm_s) > 0.5f;
      return;

    case APPROACH_ALIGN_125:
      Camera_SetAngle(APP_GRAB_ALIGN_CAMERA_ANGLE);
      camera_angle = (float)APP_GRAB_ALIGN_CAMERA_ANGLE;
      if (new_locked_target) {
        task_status.found = true;
        (void)task_track_target(vision, false);
      }
      if (tracking_filter_valid &&
          (task_abs(filtered_target_x - (float)APP_VISION_TARGET_X) <=
           APP_GRAB_ALIGN_X_ERROR_PX)) {
        Motor_Stop();
        task_status.motors_active = false;
        approach_phase = APPROACH_CAMERA_TO_140;
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, task_approach_steering());
      task_status.motors_active = task_abs(steering_mm_s) > 0.5f;
      return;

    case APPROACH_CAMERA_TO_140:
      Motor_Stop();
      task_status.motors_active = false;
      if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
        LocationPose pose;
        if (task_get_location_pose(&pose, now_ms)) {
          approach_phase_path_mm = pose.path_mm;
          approach_locked_heading_deg =
              (float)pose.heading_mdeg * 0.001f;
          approach_reacquire_generation = vision->report_generation;
          approach_phase = APPROACH_ADVANCE_140;
        }
      }
      return;

    case APPROACH_ADVANCE_140:
      task_status.found = false;
      if (new_locked_target &&
          (vision->report_generation != approach_reacquire_generation)) {
        Motor_Stop();
        task_status.motors_active = false;
        task_status.found = true;
        if (task_abs((float)vision->x - (float)APP_VISION_TARGET_X) <=
            APP_GRAB_REACQUIRE_X_ERROR_PX) {
          Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
          camera_angle = (float)APP_GRAB_VIEW_ANGLE;
          task_enter(TASK_GRAB_OBSERVE, now_ms);
        } else {
          Camera_SetAngle(APP_GRAB_ALIGN_CAMERA_ANGLE);
          camera_angle = (float)APP_GRAB_ALIGN_CAMERA_ANGLE;
          approach_phase = APPROACH_ALIGN_125;
          task_reset_tracking();
          (void)task_track_target(vision, false);
        }
        return;
      }
      {
        LocationPose pose;
        if (!task_get_location_pose(&pose, now_ms)) {
          return;
        }
        if ((pose.path_mm - approach_phase_path_mm) >=
            APP_GRAB_REACQUIRE_DISTANCE_MM) {
          Motor_Stop();
          task_status.motors_active = false;
          approach_reacquire_generation = vision->report_generation;
          approach_phase = APPROACH_RAISE_REACQUIRE;
          step_started_ms = now_ms;
          return;
        }
        const float heading_deg = (float)pose.heading_mdeg * 0.001f;
        const float heading_error = task_wrap_angle(
            approach_locked_heading_deg - heading_deg);
        Motor_Move(APP_GRAB_REACQUIRE_SPEED_MM_S, 0.0f,
                   nav_yaw(heading_error));
        task_status.motors_active = true;
      }
      return;

    case APPROACH_RAISE_REACQUIRE:
      Motor_Stop();
      task_status.motors_active = false;
      task_status.found = false;
      if (new_locked_target &&
          (vision->report_generation != approach_reacquire_generation)) {
        approach_phase = APPROACH_TRACK;
        approach_speed_mm_s = APP_GRAB_REACQUIRE_SPEED_MM_S;
        task_reset_tracking();
        (void)task_track_target(vision, true);
        task_status.found = true;
        Motor_Move(task_approach_aligned_speed(approach_speed_mm_s),
                   0.0f, task_approach_steering());
        task_status.motors_active = true;
        return;
      }
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_GRAB_REACQUIRE_STEP_MS) {
        const uint8_t current_angle = Camera_GetAngle();
        if (current_angle <= APP_GRAB_CAMERA_MIN_ANGLE) {
          task_enter(TASK_SEARCH, now_ms);
          return;
        }
        Camera_SetAngle((uint8_t)(current_angle - 1U));
        camera_angle = (float)Camera_GetAngle();
        step_started_ms = now_ms;
      }
      return;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      return;
  }
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
  task_status.found = task_scan_locked_target_found(vision);
  if (task_status.found &&
      ((uint32_t)(now_ms - state_started_ms) >=
       APP_APPROACH_LOSS_HOLD_MS)) {
    task_enter(TASK_APPROACH, now_ms);
    return;
  }
  Motor_Stop();
  task_status.motors_active = false;
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_APPROACH_LOSS_HOLD_MS) {
    /* Reuse the unified 90-degree sweep and odometry return-to-center flow.
     * SEARCH still accepts a new valid target while the camera is moving. */
    task_enter(TASK_SEARCH, now_ms);
  }
}

static void task_process_grab_observe(const VisionData *vision,
                                      uint32_t now_ms)
{
  if (complete_flow_active) {
    (void)vision;
    (void)now_ms;
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  task_status.found = task_target_matches_lock(vision);
  if (!task_report_valid(vision) ||
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
  if (complete_flow_active) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
  task_status.found = task_target_matches_lock(vision);
  if (task_status.found) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
  } else if (task_report_valid(vision) &&
             ((uint32_t)(now_ms - state_started_ms) >=
              APP_GRAB_RAISE_OBSERVE_MS)) {
    task_enter(TASK_GRAB_ROTATE, now_ms);
  }
}

static void task_process_grab_rotate(const VisionData *vision,
                                     uint32_t now_ms)
{
  if (complete_flow_active) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
  task_status.found = task_target_matches_lock(vision);
  if (task_status.found) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
  if (!task_report_valid(vision)) {
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
         task_side_flag_valid(command->flags) &&
         (command->target_x_mm >= 0) &&
         (command->target_x_mm <= APP_NAV_REMOTE_MAX_DISTANCE_MM) &&
         (command->target_y_mm == 0);
}

static float nav_end_speed(float cruise_speed_mm_s)
{
  const float scaled_speed = cruise_speed_mm_s * APP_NAV_END_SPEED_RATIO;
  return (scaled_speed < APP_NAV_MIN_END_SPEED_MM_S) ?
      APP_NAV_MIN_END_SPEED_MM_S : scaled_speed;
}

static void nav_stop_output(void)
{
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
}

static float nav_speed(int16_t remaining_mm, float cruise_speed_mm_s)
{
  const float end_speed_mm_s = nav_end_speed(cruise_speed_mm_s);
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
  float speed_mm_s = end_speed_mm_s +
      (cruise_speed_mm_s - end_speed_mm_s) * ratio;
  if ((cruise_speed_mm_s > APP_NAV_FINAL_APPROACH_SPEED_MM_S) &&
      (remaining_mm <= (int16_t)APP_NAV_FINAL_APPROACH_DISTANCE_MM) &&
      (speed_mm_s > APP_NAV_FINAL_APPROACH_SPEED_MM_S)) {
    speed_mm_s = APP_NAV_FINAL_APPROACH_SPEED_MM_S;
  }
  return speed_mm_s;
}

static void nav_track_progress(const VisionMissionCommand *command,
                               const LocationPose *pose, uint32_t now_ms)
{
  const int32_t distance_change =
      (int32_t)command->target_x_mm - (int32_t)nav_progress.remaining_mm;
  int32_t heading_change = pose->heading_mdeg - nav_progress.heading_mdeg;
  if (heading_change > 180000) {
    heading_change -= 360000;
  } else if (heading_change < -180000) {
    heading_change += 360000;
  }
  const uint32_t path_change = (pose->path_mm >= nav_progress.path_mm) ?
      pose->path_mm - nav_progress.path_mm :
      nav_progress.path_mm - pose->path_mm;
  const bool changed = !nav_progress.valid ||
      (distance_change >= APP_NAV_PROGRESS_DISTANCE_MM) ||
      (distance_change <= -APP_NAV_PROGRESS_DISTANCE_MM) ||
      (heading_change >= APP_NAV_PROGRESS_HEADING_MDEG) ||
      (heading_change <= -APP_NAV_PROGRESS_HEADING_MDEG) ||
      (path_change >= APP_NAV_PROGRESS_PATH_MM);

  if (changed) {
    nav_progress.remaining_mm = command->target_x_mm;
    nav_progress.heading_mdeg = pose->heading_mdeg;
    nav_progress.path_mm = pose->path_mm;
    nav_progress.changed_ms = now_ms;
    nav_progress.valid = true;
    nav_progress.stale = false;
    task_status.nav_stale = false;
  }
}

static bool nav_terminal_timeout(int16_t remaining_mm, uint32_t now_ms)
{
  if (nav_terminal_latched) {
    if (remaining_mm > (int16_t)APP_NAV_TERMINAL_RELEASE_MM) {
      nav_terminal_candidate = false;
      nav_terminal_latched = false;
      return false;
    }
    return (uint32_t)(now_ms - nav_terminal_latched_ms) >=
           APP_NAV_TERMINAL_TIMEOUT_MS;
  }

  if (remaining_mm > (int16_t)APP_NAV_TERMINAL_WINDOW_MM) {
    nav_terminal_candidate = false;
    return false;
  }
  if (!nav_terminal_candidate) {
    nav_terminal_candidate = true;
    nav_terminal_candidate_ms = now_ms;
    return false;
  }
  if ((uint32_t)(now_ms - nav_terminal_candidate_ms) <
      APP_NAV_TERMINAL_STABLE_MS) {
    return false;
  }

  nav_terminal_latched = true;
  nav_terminal_latched_ms = now_ms;
  return false;
}

static float nav_yaw(float heading_error_deg)
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

static RemoteRouteStatus nav_push(uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    task_pause_final_push(now_ms);
    nav_forward_active = false;
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
  }

  if (!nav_heading_locked) {
    nav_locked_heading_deg = task_safe_zone_heading();
    nav_heading_locked = true;
    task_status.nav_heading_locked = true;
    task_status.nav_locked_heading_deg =
        (uint16_t)(nav_locked_heading_deg + 0.5f) % 360U;
    nav_ready = false;
    Motor_Stop();
    task_status.motors_active = false;
  }

  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float heading_error_deg = task_wrap_angle(
      nav_locked_heading_deg - current_heading_deg);
  if (!nav_final_push_active && nav_ready &&
      (task_abs(heading_error_deg) >
       APP_NAV_FINAL_TURN_TOLERANCE_DEG)) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    return REMOTE_ROUTE_WAITING;
  }
  if (nav_final_push_active && nav_ready &&
      (task_abs(heading_error_deg) >= APP_NAV_REALIGN_DEG)) {
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    return REMOTE_ROUTE_WAITING;
  }
  if (!nav_ready) {
    if (!turn_to(nav_locked_heading_deg, current_heading_deg,
                 APP_NAV_FINAL_TURN_TOLERANCE_DEG, now_ms)) {
      return REMOTE_ROUTE_MOTOR_FAULT;
    }
    return REMOTE_ROUTE_WAITING;
  }
  if ((uint32_t)(now_ms - step_started_ms) < APP_NAV_TURN_SETTLE_MS) {
    return REMOTE_ROUTE_WAITING;
  }

  if (!nav_final_push_active) {
    nav_final_push_active = true;
    nav_final_push_started_ms = now_ms;
    nav_final_push_start_path_mm = pose.path_mm;
    nav_forward_active = false;
    task_status.nav_final_push = true;
    nav_stop_output();
  } else if (nav_final_push_paused) {
    nav_final_push_started_ms += now_ms - nav_final_push_paused_ms;
    nav_final_push_paused = false;
  }

  const uint32_t elapsed_ms = now_ms - nav_final_push_started_ms;
  const uint32_t travelled_mm = pose.path_mm - nav_final_push_start_path_mm;
  const bool encoder_limit_reached =
      (elapsed_ms >= APP_NAV_FINAL_PUSH_MIN_TIME_MS) &&
      (travelled_mm >= APP_NAV_FINAL_PUSH_MAX_DISTANCE_MM);
  if ((elapsed_ms >= APP_NAV_FINAL_PUSH_TIME_MS) ||
      encoder_limit_reached) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_final_push_active = false;
    nav_final_push_done = true;
    nav_final_push_paused = false;
    distance_command_done = true;
    task_status.nav_done = true;
    task_status.nav_final_push = false;
    nav_stop_output();
    return REMOTE_ROUTE_REACHED;
  }

  const float yaw_mm_s = nav_yaw(heading_error_deg);
  Motor_Move(APP_NAV_FINAL_PUSH_SPEED_MM_S, 0.0f, yaw_mm_s);
  task_status.motors_active = true;
  return REMOTE_ROUTE_RUNNING;
}

static RemoteRouteStatus nav_follow(
    const VisionMissionCommand *command, uint8_t expected_command,
    float cruise_speed_mm_s, uint32_t now_ms)
{
  if (!task_distance_command_valid(command, expected_command)) {
    return REMOTE_ROUTE_COMMAND_INVALID;
  }

  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    task_pause_final_push(now_ms);
    nav_forward_active = false;
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
  }
  nav_track_progress(command, &pose, now_ms);

  if (nav_final_push_done) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    nav_stop_output();
    return REMOTE_ROUTE_REACHED;
  }
  if ((expected_command == VISION_CMD_RETURN_CENTER) &&
      ((command->target_x_mm <=
        (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM) ||
       (distance_command_done &&
        (command->target_x_mm <=
         (int16_t)APP_NAV_REMOTE_RESUME_DISTANCE_MM)))) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    nav_stop_output();
    return REMOTE_ROUTE_REACHED;
  }
  if (route_to_stash &&
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      (command->target_x_mm <=
       (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM)) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    nav_stop_output();
    return REMOTE_ROUTE_REACHED;
  }
  distance_command_done = false;
  task_status.nav_done = false;

  const bool delivery_route =
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) && !route_to_stash;
  const bool terminal_timeout = delivery_route &&
      nav_terminal_timeout(command->target_x_mm, now_ms);

  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float command_heading_deg = (float)command->heading_cdeg * 0.01f;
  const bool return_route =
      expected_command == VISION_CMD_RETURN_CENTER;
  const bool lock_allowed =
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      !route_to_stash;

  if (nav_heading_locked &&
      (!lock_allowed ||
       (!nav_final_push_active &&
        (command->target_x_mm >
         (int16_t)APP_NAV_HEADING_UNLOCK_DISTANCE_MM)))) {
    nav_heading_locked = false;
    task_status.nav_heading_locked = false;
  }
  if (!nav_heading_locked && lock_allowed &&
      (command->target_x_mm <=
       (int16_t)APP_NAV_HEADING_LOCK_DISTANCE_MM)) {
    nav_locked_heading_deg = task_safe_zone_heading();
    nav_heading_locked = true;
    task_status.nav_heading_locked = true;
    task_status.nav_locked_heading_deg =
        (uint16_t)(nav_locked_heading_deg + 0.5f) % 360U;
    /* Stop translation and complete the configured final alignment before
     * pushing. The first delivery applies the calibrated side offset. */
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
  }

  const float desired_heading_deg = nav_heading_locked ?
      nav_locked_heading_deg : command_heading_deg;
  const float heading_error_deg = task_wrap_angle(
      desired_heading_deg - current_heading_deg);
  const float route_realign_deg = return_route ?
      APP_RETURN_CENTER_REALIGN_DEG : APP_NAV_REALIGN_DEG;
  if (nav_ready &&
      ((nav_heading_locked &&
        (task_abs(heading_error_deg) >
         APP_NAV_FINAL_TURN_TOLERANCE_DEG)) ||
       (!nav_heading_locked &&
        (task_abs(heading_error_deg) >= route_realign_deg)))) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
  }
  if (!nav_ready) {
    nav_forward_active = false;
    nav_stop_output();
    const float turn_tolerance_deg = nav_heading_locked ?
        APP_NAV_FINAL_TURN_TOLERANCE_DEG :
        (return_route ? APP_RETURN_CENTER_TURN_TOLERANCE_DEG :
                        APP_NAV_HEADING_TOLERANCE_DEG);
    if (!turn_to(desired_heading_deg, current_heading_deg,
                 turn_tolerance_deg, now_ms)) {
      return REMOTE_ROUTE_MOTOR_FAULT;
    }
    return REMOTE_ROUTE_WAITING;
  }
  if ((uint32_t)(now_ms - step_started_ms) < APP_NAV_TURN_SETTLE_MS) {
    return REMOTE_ROUTE_WAITING;
  }

  if (delivery_route &&
      ((command->target_x_mm <=
        (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM) ||
       nav_final_push_active || terminal_timeout)) {
    return nav_push(now_ms);
  }

  if (!nav_forward_active) {
    nav_forward_active = true;
    nav_progress.changed_ms = now_ms;
  } else if (!nav_terminal_candidate && !nav_terminal_latched &&
             ((uint32_t)(now_ms - nav_progress.changed_ms) >=
              APP_NAV_PROGRESS_TIMEOUT_MS)) {
    nav_progress.stale = true;
    task_status.nav_stale = true;
  }
  if (delivery_route &&
      (nav_terminal_candidate || nav_terminal_latched)) {
    /* The terminal envelope is only a background timeout. Keep the original
     * D-driven motion active and prevent its small pose residual from being
     * mistaken for a navigation freeze. */
    nav_progress.stale = false;
    task_status.nav_stale = false;
  }
  if (nav_progress.stale) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
  }

  const float route_speed_mm_s = nav_speed(
      command->target_x_mm, cruise_speed_mm_s);
  float target_yaw_mm_s = nav_yaw(heading_error_deg);
  if (return_route) {
    if (target_yaw_mm_s > APP_RETURN_CENTER_HEADING_MAX_MM_S) {
      target_yaw_mm_s = APP_RETURN_CENTER_HEADING_MAX_MM_S;
    } else if (target_yaw_mm_s < -APP_RETURN_CENTER_HEADING_MAX_MM_S) {
      target_yaw_mm_s = -APP_RETURN_CENTER_HEADING_MAX_MM_S;
    }
  }
  const float speed_rate_mm_s2 =
      (task_abs(route_speed_mm_s) >= task_abs(remote_speed_mm_s)) ?
      APP_NAV_SPEED_ACCEL_MM_S2 : APP_NAV_SPEED_DECEL_MM_S2;
  remote_speed_mm_s = task_step_toward(
      remote_speed_mm_s, route_speed_mm_s,
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

  /* Dynamic RDK remaining distance makes every STOP resumable. */
  if (navigating &&
      (vision->mission.command == VISION_CMD_STOP)) {
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
    return;
  }

  if (vision->mission.command == VISION_CMD_ENTER_SAFE_ZONE) {
    if (!delivery_enter_command_ok(&vision->mission)) {
      task_pause_final_push(now_ms);
      Motor_Stop();
      task_status.motors_active = false;
      nav_ready = false;
      nav_forward_active = false;
      nav_stop_output();
      return;
    }
    const RemoteRouteStatus result = nav_push(now_ms);
    if (result == REMOTE_ROUTE_REACHED) {
      task_enter(TASK_OPEN_FOR_RAM, now_ms);
    } else if (result == REMOTE_ROUTE_MOTOR_FAULT) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }
  if (vision->mission.command == VISION_CMD_ALIGN_SAFE_ZONE) {
    /* Old RDK software may still send ALIGN. Do not rotate or fault; hold NAV
     * until the updated planner sends ENTER_SAFE_ZONE directly. */
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
    return;
  }

  if (vision->mission.command != VISION_CMD_NAVIGATE_WAYPOINT) {
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
    return;
  }
  const RemoteRouteStatus result = nav_follow(
      &vision->mission, VISION_CMD_NAVIGATE_WAYPOINT,
      APP_NAV_FAST_SPEED_MM_S, now_ms);
  if (result == REMOTE_ROUTE_COMMAND_INVALID) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
  } else if (result == REMOTE_ROUTE_MOTOR_FAULT) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
}

static bool distance_failed(MotorDistanceStatus result)
{
  return (result == MOTOR_DISTANCE_FAULT) ||
         (result == MOTOR_DISTANCE_INVALID);
}

static bool delivery_enter_command_ok(const VisionMissionCommand *command)
{
  const uint8_t required_flags = VISION_CMD_DRIVE_STRAIGHT |
                                 VISION_CMD_USE_FINAL_HEADING;
  return task_mission_valid(command) &&
         (command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
         ((command->flags & required_flags) == required_flags) &&
         task_side_flag_valid(command->flags);
}

static void task_process_delivery_verify(
    const VisionMissionCommand *command, uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  if (!task_mission_valid(command) ||
      ((command->command != VISION_CMD_ENTER_SAFE_ZONE) &&
       (command->command != VISION_CMD_TASK_COMPLETE))) {
    /* This state is already stationary. A delayed camera/planner cycle must
     * not turn a recoverable communication gap into a latched STOP. */
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  if ((uint32_t)(now_ms - state_started_ms) <
      APP_DELIVERY_VERIFY_WAIT_MS) {
    return;
  }
  if (command->command == VISION_CMD_TASK_COMPLETE) {
    first_delivery_done = true;
    Camera_SetAngle(APP_DELIVERY_CAMERA_ANGLE);
    camera_angle = (float)APP_DELIVERY_CAMERA_ANGLE;
    task_enter(TASK_EXIT_SAFE_ZONE, now_ms);
  }
}

static void task_process_face_center(const VisionData *vision,
                                     uint32_t now_ms)
{
  const VisionMissionCommand *command = &vision->mission;
  if (!task_mission_valid(command) ||
      (command->command != VISION_CMD_RETURN_CENTER)) {
    Motor_Stop();
    task_status.motors_active = false;
    if (!stash_backoff_pending) {
      nav_ready = false;
    }
    nav_forward_active = false;
    nav_stop_output();
    return;
  }

  if (stash_backoff_pending) {
    LocationPose pose;
    if (!task_get_location_pose(&pose, now_ms)) {
      return;
    }
    if (!nav_ready) {
      /* Hold the release heading and measure one uninterrupted path away from
       * the temporary pile before consuming the remote centre heading. */
      nav_progress.path_mm = pose.path_mm;
      nav_locked_heading_deg = (float)pose.heading_mdeg * 0.001f;
      nav_ready = true;
    }
    const uint32_t travelled_mm = pose.path_mm - nav_progress.path_mm;
    const uint32_t target_mm =
        (uint32_t)(APP_STASH_RETURN_BACKOFF_M * 1000.0f + 0.5f);
    if (travelled_mm >= target_mm) {
      Motor_Stop();
      task_status.motors_active = false;
      stash_backoff_pending = false;
      nav_ready = false;
      nav_reset_progress(now_ms);
      step_started_ms = now_ms;
      return;
    }
    const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
    const float heading_error_deg = task_wrap_angle(
        nav_locked_heading_deg - current_heading_deg);
    Motor_Move(-APP_STASH_RETURN_BACKOFF_SPEED_MM_S, 0.0f,
               nav_yaw(heading_error_deg));
    task_status.motors_active = true;
    return;
  }

  const RemoteRouteStatus result = nav_follow(
      command, VISION_CMD_RETURN_CENTER,
      APP_RETURN_CENTER_SPEED_MM_S, now_ms);
  if (result == REMOTE_ROUTE_REACHED) {
    task_enter(TASK_SEARCH, now_ms);
  } else if (result == REMOTE_ROUTE_COMMAND_INVALID) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
  } else if (result == REMOTE_ROUTE_MOTOR_FAULT) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
}

static void task_remote_action_advance(uint32_t now_ms)
{
  const LocationPose pose = Location_GetPose();
  Motor_Stop();
  task_status.motors_active = false;
  ++remote_action.phase;
  remote_action.phase_path_mm = pose.path_mm;
  step_started_ms = now_ms;
  task_reset_turn_tracker();
}

static void task_remote_action_finish(void)
{
  Motor_Stop();
  task_status.motors_active = false;
  remote_action.done = true;
}

static void task_pause_action(uint32_t now_ms, bool cancel_disperse)
{
  Motor_Stop();
  task_status.motors_active = false;
  if (!remote_action.paused) {
    remote_action.paused = true;
    remote_action.paused_ms = now_ms;
  }
  if (cancel_disperse &&
      (remote_action.type == REMOTE_ACTION_DISPERSE) &&
      ((uint32_t)(now_ms - remote_action.paused_ms) >=
       APP_REMOTE_DISPERSE_HOLD_CANCEL_MS)) {
    task_enter(TASK_SEARCH, now_ms);
  }
}

static void task_pause_runtime(uint32_t now_ms)
{
  if ((state == TASK_REMOTE_ACTION) && !remote_action.done) {
    /* PAUSE is recoverable and must never start the HOLD cancellation timer. */
    task_pause_action(now_ms, false);
    return;
  }

  Motor_Stop();
  task_status.motors_active = false;
  if (!stash_backoff_pending) {
    nav_ready = false;
  }
  nav_forward_active = false;
  task_pause_final_push(now_ms);
}

static bool task_remote_distance(float distance_m, float speed_mm_s,
                                 uint32_t now_ms)
{
  const MotorDistanceStatus result = Motor_MoveDistance(distance_m, speed_mm_s);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_remote_action_advance(now_ms);
    return true;
  }
  if (distance_failed(result)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  return false;
}

static bool task_remote_lateral(int16_t distance_mm, float speed_mm_s,
                                uint32_t now_ms)
{
  if (distance_mm == 0) {
    task_remote_action_advance(now_ms);
    return true;
  }
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    Motor_Stop();
    task_status.motors_active = false;
    return false;
  }
  const uint32_t travelled_mm = pose.path_mm - remote_action.phase_path_mm;
  if (travelled_mm >= (uint32_t)((distance_mm < 0) ? -distance_mm : distance_mm)) {
    task_remote_action_advance(now_ms);
    return true;
  }
  (void)Motor_MoveAngle(speed_mm_s,
                        (distance_mm > 0) ? 90.0f : 270.0f);
  task_status.motors_active = true;
  return false;
}

static bool task_remote_turn(float angle_deg, uint32_t now_ms)
{
  const MotorTurnStatus result = Motor_TurnAngle(angle_deg);
  task_status.motors_active = result == MOTOR_TURN_RUNNING;
  if (result == MOTOR_TURN_DONE) {
    task_remote_action_advance(now_ms);
    return true;
  }
  if ((result == MOTOR_TURN_FAULT) || (result == MOTOR_TURN_INVALID)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  return false;
}

static bool task_remote_disperse_turn(float offset_deg, uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    return false;
  }
  if (!remote_action.reference_heading_valid) {
    remote_action.reference_heading_mdeg = pose.heading_mdeg;
    remote_action.reference_heading_valid = true;
  }
  const float reference_deg =
      (float)remote_action.reference_heading_mdeg * 0.001f;
  const float desired_deg = task_wrap_angle(reference_deg + offset_deg);
  const float current_deg = (float)pose.heading_mdeg * 0.001f;
  const float error_deg = task_wrap_angle(desired_deg - current_deg);
  const MotorTurnStatus result = Motor_TurnAngleSpeed(
      error_deg, APP_REMOTE_DISPERSE_TURN_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_TURN_RUNNING;
  if (result == MOTOR_TURN_DONE) {
    task_remote_action_advance(now_ms);
    return true;
  }
  if ((result == MOTOR_TURN_FAULT) || (result == MOTOR_TURN_INVALID)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  return false;
}

static void task_process_remote_action(const VisionMissionCommand *command,
                                       uint32_t now_ms)
{
  if (remote_action.done) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  if (!task_mission_valid(command) ||
      (command->command != remote_action.command)) {
    task_pause_action(now_ms, command->command == VISION_CMD_HOLD);
    return;
  }
  if (remote_action.paused) {
    remote_action.started_ms += now_ms - remote_action.paused_ms;
    remote_action.paused = false;
  }
  const uint32_t action_timeout_ms =
      (remote_action.type == REMOTE_ACTION_DISPERSE) ?
          APP_REMOTE_DISPERSE_TIMEOUT_MS :
      ((remote_action.type == REMOTE_ACTION_RELEASE_BOTH) &&
       (remote_action.arg_b == 1)) ?
          APP_CARGO_IMPACT_TIMEOUT_MS : APP_REMOTE_ACTION_TIMEOUT_MS;
  if ((uint32_t)(now_ms - remote_action.started_ms) >= action_timeout_ms) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
    return;
  }

  switch (remote_action.type) {
    case REMOTE_ACTION_RELEASE_LEFT:
      if (Claw_OpenLeft(now_ms)) {
        /* Left is fully open; right holds the retained cargo at 110 degrees. */
        task_status.gripper_closed = false;
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_RELEASE_RIGHT:
      if (Claw_OpenRight(now_ms)) {
        /* Right is fully open; left holds the retained cargo at 70 degrees. */
        task_status.gripper_closed = false;
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_RELEASE_BOTH:
      if (remote_action.arg_b == 1) {
        if (remote_action.phase == 0U) {
          if (Claw_Open(now_ms)) {
            task_status.gripper_closed = false;
            task_remote_action_advance(now_ms);
          }
        } else if (remote_action.phase == 1U) {
          (void)task_remote_distance(-APP_CARGO_IMPACT_DISTANCE_M,
                                     APP_CARGO_IMPACT_BACK_SPEED_MM_S,
                                     now_ms);
        } else if (remote_action.phase == 2U) {
          if (Claw_Touch(now_ms)) {
            task_remote_action_advance(now_ms);
          }
        } else if (remote_action.phase == 3U) {
          (void)task_remote_distance(APP_CARGO_IMPACT_DISTANCE_M,
                                     APP_CARGO_IMPACT_SPEED_MM_S, now_ms);
        } else if (remote_action.phase == 4U) {
          (void)task_remote_distance(-APP_CARGO_IMPACT_DISTANCE_M,
                                     APP_CARGO_IMPACT_BACK_SPEED_MM_S,
                                     now_ms);
        } else if (remote_action.phase == 5U) {
          /* This branch has dropped and rammed the whole ambiguous batch;
           * unlike unilateral release, no cargo remains for an in-claw audit.
           * Reopen before SEARCH so the next object can enter normally. */
          if (Claw_Open(now_ms)) {
            task_status.gripper_closed = false;
            cargo_recheck_pending = false;
            task_clear_audit_result();
            task_remote_action_finish();
          }
        }
      } else if (Claw_Open(now_ms)) {
        /* Temporary-stash release and a second failed audit still require a
         * complete dual release. Stash RETURN retreats before turning. */
        stash_backoff_pending = route_to_stash;
        task_status.gripper_closed = false;
        cargo_recheck_pending = false;
        task_clear_audit_result();
        route_to_stash = false;
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_YIELD:
      if (!cargo_recheck_pending) {
        if (task_remote_distance((float)remote_action.arg_a * 0.001f,
                                 APP_REMOTE_YIELD_SPEED_MM_S, now_ms)) {
          task_remote_action_finish();
        }
        return;
      }
      {
        const int32_t requested_mm = remote_action.arg_a;
        const int32_t direction = (requested_mm < 0) ? -1 : 1;
        const uint32_t magnitude_mm = (uint32_t)(
            (requested_mm < 0) ? -requested_mm : requested_mm);
        const uint32_t start_mm =
            (magnitude_mm < APP_CARGO_SEPARATE_START_MM) ?
                magnitude_mm : APP_CARGO_SEPARATE_START_MM;
        const uint32_t fast_mm = magnitude_mm - start_mm;
        if (remote_action.phase == 0U) {
          (void)task_remote_distance(
              (float)(direction * (int32_t)start_mm) * 0.001f,
              APP_CARGO_SEPARATE_START_SPEED_MM_S, now_ms);
        } else if (remote_action.phase == 1U) {
          if (fast_mm == 0U) {
            task_remote_action_advance(now_ms);
          } else {
            (void)task_remote_distance(
                (float)(direction * (int32_t)fast_mm) * 0.001f,
                APP_REMOTE_YIELD_SPEED_MM_S, now_ms);
          }
        } else if (remote_action.phase == 2U) {
        /* A unilateral release keeps the opposite cargo at the normal touch
         * angle.  After physical separation, look back into the claw before
         * accepting either navigation or a new search target. */
          if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
            task_remote_action_advance(now_ms);
          }
        } else if ((uint32_t)(now_ms - step_started_ms) >=
                   APP_CARGO_RECHECK_SETTLE_MS) {
          task_remote_action_finish();
        }
      }
      return;

    case REMOTE_ACTION_ESCAPE:
      /* Recovery may be requested after an interrupted mechanism sequence.
       * Always restore the large lift to its travel clearance before moving. */
      Lift_SetTravelPosition();
      if (remote_action.phase == 0U) {
        (void)task_remote_turn((float)remote_action.arg_a, now_ms);
      } else if (remote_action.phase == 1U) {
        if (task_remote_lateral(remote_action.arg_b,
                                APP_REMOTE_ESCAPE_LATERAL_SPEED_MM_S,
                                now_ms)) {
          task_remote_action_finish();
        }
      }
      return;

    case REMOTE_ACTION_LANE:
      if (remote_action.phase == 0U) {
        (void)task_remote_lateral(remote_action.arg_a,
                                  APP_REMOTE_LANE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 1U) {
        if (remote_action.arg_b == 0) {
          task_remote_action_finish();
        } else if (task_remote_distance(
                       (float)remote_action.arg_b * 0.001f,
                       APP_REMOTE_LANE_SPEED_MM_S, now_ms)) {
          task_remote_action_finish();
        }
      }
      return;

    case REMOTE_ACTION_DISPERSE:
      if (remote_action.phase == 0U) {
        /* Use the firm normal Touch pose as the contact surface.  Do not set
         * GRIPPER_CLOSED: this is an empty-claw disperse action, not cargo. */
        if (Claw_Touch(now_ms)) {
          task_remote_action_advance(now_ms);
        }
      } else if (remote_action.phase == 1U) {
        (void)task_remote_disperse_turn(45.0f, now_ms);
      } else if (remote_action.phase == 2U) {
        (void)task_remote_disperse_turn(-45.0f, now_ms);
      } else if (remote_action.phase == 3U) {
        (void)task_remote_disperse_turn(90.0f, now_ms);
      } else if (remote_action.phase == 4U) {
        (void)task_remote_disperse_turn(-90.0f, now_ms);
      } else if (remote_action.phase == 5U) {
        if (task_remote_distance(-APP_REMOTE_DISPERSE_BACKOFF_M,
                                 APP_REMOTE_DISPERSE_BACK_SPEED_MM_S,
                                 now_ms)) {
          task_remote_action_finish();
        }
      }
      return;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      return;
  }
}

static void task_accept_mission(const VisionMissionCommand *command,
                                uint32_t now_ms)
{
  if (!task_mission_valid(command) ||
      (mission_sequence_valid &&
       (command->sequence == mission_sequence))) {
    return;
  }
  const uint8_t previous_ack = task_status.acknowledged_sequence;
  mission_sequence = command->sequence;
  mission_sequence_valid = true;

  if (command->command >= VISION_CMD_APPROACH_TARGET) {
    complete_flow_active = true;
  }

  task_status.last_command = command->command;
  task_status.command_received = true;

  if (command->command == VISION_CMD_ABORT) {
    task_status.acknowledged_sequence = command->sequence;
    task_stop(TASK_FAULT_REMOTE_STOP, now_ms);
  } else if (command->command == VISION_CMD_PAUSE) {
    task_status.acknowledged_sequence = command->sequence;
    if ((state != TASK_WAIT_CONFIG) && (state != TASK_START) &&
        (state != TASK_OPEN_CLAW)) {
      mission_paused = true;
      task_pause_runtime(now_ms);
    }
  } else if (command->command == VISION_CMD_HOLD) {
    task_status.acknowledged_sequence = command->sequence;
    if ((state == TASK_NAVIGATE) && route_to_stash &&
               !task_status.gripper_closed && nav_progress.valid &&
               (distance_command_done ||
                (nav_progress.remaining_mm <=
                 APP_STASH_ROUTE_HOLD_ACCEPT_MM))) {
      /* RETURN_STASH uses the existing NAV frame while the claws are empty.
       * It may change back to SEARCH/HOLD on its tight map tolerance just
       * before D reaches zero.  Accept that hand-off only after a genuine NAV
       * payload is already within 100 mm; a distant HOLD remains a stop. */
      task_enter(TASK_SEARCH, now_ms);
    } else if ((state == TASK_REMOTE_ACTION) && remote_action.done &&
               ((remote_action.type == REMOTE_ACTION_DISPERSE) ||
                ((remote_action.type == REMOTE_ACTION_RELEASE_BOTH) &&
                 (remote_action.arg_b == 1)))) {
      task_enter(TASK_SEARCH, now_ms);
    } else if (state != TASK_SEARCH) {
      if ((state == TASK_REMOTE_ACTION) && !remote_action.done) {
        /* One HOLD frame is sufficient: the periodic action processor keeps
         * the one-second DISPERSE cancellation timer running after it ages. */
        task_pause_action(now_ms, true);
      } else {
        Motor_Stop();
        task_status.motors_active = false;
        if (!stash_backoff_pending) {
          nav_ready = false;
        }
        nav_forward_active = false;
      }
    }
  } else if ((command->command == VISION_CMD_CARGO_AUDIT) &&
             (((state >= TASK_GRAB_OBSERVE) &&
               (state <= TASK_GRAB_ROTATE)) ||
              ((state == TASK_REMOTE_ACTION) && remote_action.done &&
               cargo_recheck_pending))) {
    task_status.acknowledged_sequence = command->sequence;
    if (state == TASK_REMOTE_ACTION) {
      task_enter(TASK_GRAB_OBSERVE, now_ms);
    }
    task_latch_audit(command);
  } else if ((command->command == VISION_CMD_APPROACH_TARGET) &&
             !stash_backoff_pending &&
             !cargo_recheck_pending &&
             ((state == TASK_SEARCH) ||
              (state == TASK_APPROACH_RECOVER) ||
              ((state == TASK_REMOTE_ACTION) && remote_action.done))) {
    task_status.acknowledged_sequence = command->sequence;
    locked_cargo_counts = 1U;
    task_enter(TASK_APPROACH, now_ms);
    cluster_target_active =
        (command->flags & VISION_CMD_CLUSTER_TARGET) != 0U;
  } else if ((command->command == VISION_CMD_APPROACH_TARGET) &&
             (state == TASK_APPROACH)) {
    task_status.acknowledged_sequence = command->sequence;
    const bool cluster =
        (command->flags & VISION_CMD_CLUSTER_TARGET) != 0U;
    if (cluster != cluster_target_active) {
      task_enter(TASK_APPROACH, now_ms);
      cluster_target_active = cluster;
    }
  } else if ((command->command == VISION_CMD_APPROACH_TARGET) &&
             (state == TASK_DISPERSE_READY)) {
    task_status.acknowledged_sequence = command->sequence;
    if ((command->flags & VISION_CMD_CLUSTER_TARGET) == 0U) {
      task_enter(TASK_APPROACH, now_ms);
      cluster_target_active = false;
    }
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
             (state <= TASK_GRAB_ROTATE) &&
             (!complete_flow_active || (audit_received && audit_valid))) {
    task_status.acknowledged_sequence = command->sequence;
    if (cargo_recheck_pending) {
      /* Backoff has physically separated the released cargo.  Close both
       * claws to the ordinary touch angles again so the retained cargo cannot
       * slip out during navigation. */
      cargo_recheck_pending = false;
    }
    task_enter(TASK_CLOSE_CLAW, now_ms);
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             ((state == TASK_CLOSE_CLAW) ||
              (state == TASK_WAIT_NAVIGATION))) {
    /* GRAB_CONFIRMED is now repeated until GRIPPER_CLOSED is reported.  ACK
     * every new sequence, but never restart an in-progress/completed motion. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             task_status.gripper_closed &&
             ((state == TASK_WAIT_NAVIGATION) ||
              ((state == TASK_REMOTE_ACTION) && remote_action.done))) {
    task_status.acknowledged_sequence = command->sequence;
    route_to_stash = audit_initial_stash;
    task_enter(TASK_NAVIGATE, now_ms);
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             !task_status.gripper_closed && (state == TASK_SEARCH) &&
             task_distance_command_valid(command,
                                         VISION_CMD_NAVIGATE_WAYPOINT)) {
    /* The current upper-computer RETURN_STASH state reuses NAV because the
     * wire protocol has no separate empty-claw waypoint opcode.  Constrain
     * this compatibility entry to SEARCH + open claws + a fully validated
     * distance/heading payload, and mark it as a stash route so safe-zone
     * heading lock and final pushing can never run. */
    task_status.acknowledged_sequence = command->sequence;
    route_to_stash = true;
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
             !route_to_stash && (state == TASK_NAVIGATE)) {
    task_status.acknowledged_sequence = command->sequence;
    if (nav_final_push_done) {
      task_enter(TASK_OPEN_FOR_RAM, now_ms);
    }
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
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_REMOTE_ACTION) && remote_action.done) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_FACE_FIELD_CENTER, now_ms);
  } else if ((state == TASK_REMOTE_ACTION) &&
             (command->command == remote_action.command)) {
    /* The upper computer repeats commands with new SEQs. ACK each one while
     * keeping the already-running or completed physical action idempotent. */
    task_status.acknowledged_sequence = command->sequence;
    if (remote_action.paused) {
      remote_action.started_ms += now_ms - remote_action.paused_ms;
      remote_action.paused = false;
    }
  } else if ((command->command == VISION_CMD_YIELD_BACKOFF) &&
             task_yield_allowed()) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_YIELD, command, now_ms);
  } else if ((command->command == VISION_CMD_ESCAPE_MANEUVER) &&
             task_escape_allowed()) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_ESCAPE, command, now_ms);
  } else if ((command->command == VISION_CMD_CHANGE_LANE) &&
             task_lane_allowed()) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_LANE, command, now_ms);
  } else if ((command->command == VISION_CMD_DISPERSE_PILE) &&
             !stash_backoff_pending &&
             !cargo_recheck_pending &&
             !task_status.gripper_closed &&
             (state == TASK_DISPERSE_READY)) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_DISPERSE, command, now_ms);
  } else if (((command->command == VISION_CMD_RELEASE_LEFT) ||
              (command->command == VISION_CMD_RELEASE_RIGHT) ||
              (command->command == VISION_CMD_RELEASE_BOTH)) &&
             audit_received && (!audit_valid || audit_initial_stash) &&
             task_release_command_valid(command->command) &&
             !((state == TASK_REMOTE_ACTION) && !remote_action.done)) {
    task_status.acknowledged_sequence = command->sequence;
    /* Execute exactly the side selected from the latest stable audit. A failed
     * re-audit is only allowed to request RELEASE_BOTH. */
    const RemoteAction action =
        (command->command == VISION_CMD_RELEASE_LEFT) ?
            REMOTE_ACTION_RELEASE_LEFT :
        (command->command == VISION_CMD_RELEASE_RIGHT) ?
            REMOTE_ACTION_RELEASE_RIGHT : REMOTE_ACTION_RELEASE_BOTH;
    const bool split_unknown_side =
        (action == REMOTE_ACTION_RELEASE_BOTH) &&
        !audit_initial_stash && !cargo_recheck_pending;
    task_start_remote_action(action, command, now_ms);
    /* RELEASE_BOTH has two contexts without changing the wire protocol:
     * 0 = real dual release, 1 = first ambiguous audit, local ram-and-search. */
    remote_action.arg_b = split_unknown_side ? 1 : 0;
  }

  /* A PAUSE remains latched even if that frame later becomes stale. Only a
   * different, state-valid command (proved by its ACK changing) resumes the
   * task. A sequence wrap can delay resume by one frame, never resume motion
   * on a rejected command. */
  if ((command->command != VISION_CMD_PAUSE) &&
      (task_status.acknowledged_sequence != previous_ack)) {
    mission_paused = false;
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

  VisionData vision = Vision_GetSnapshot();
  task_update_match_time(now_ms);
  task_status.camera_angle = Camera_GetAngle();
  task_accept_mission(&vision.mission, now_ms);
  task_apply_complete_target(&vision);
  if (mission_paused &&
      (state != TASK_WAIT_CONFIG) && (state != TASK_START) &&
      (state != TASK_OPEN_CLAW)) {
    task_pause_runtime(now_ms);
    task_publish_status(now_ms);
    return;
  }
  const bool local_approach_reacquire =
      (state == TASK_APPROACH) &&
      ((approach_phase == APPROACH_CAMERA_TO_140) ||
       (approach_phase == APPROACH_ADVANCE_140) ||
       (approach_phase == APPROACH_RAISE_REACQUIRE));
  if (complete_flow_active && !local_approach_reacquire &&
      task_mission_valid(&vision.mission) &&
      (vision.mission.command == VISION_CMD_HOLD) &&
      (state != TASK_WAIT_CONFIG) && (state != TASK_START) &&
      (state != TASK_OPEN_CLAW) && (state != TASK_SEARCH)) {
    if ((state == TASK_REMOTE_ACTION) && !remote_action.done) {
      task_pause_action(now_ms, true);
    } else {
      Motor_Stop();
      task_status.motors_active = false;
    }
    if (state == TASK_APPROACH) {
      /* Normal tracking/alignment HOLD is a stationary upper-level decision.
       * The already-triggered local reacquire phases bypass this branch. */
      task_status.auto_approach = true;
      task_status.found = false;
    } else if (state == TASK_APPROACH_RECOVER) {
      task_status.auto_approach = true;
      task_status.found = false;
    }
    task_publish_status(now_ms);
    return;
  }

  switch (state) {
    case TASK_WAIT_CONFIG:
      Motor_Stop();
      if (vision.config_ready) {
        configured_color = vision.color;
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
        task_enter(TASK_SEARCH, now_ms);
      }
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

    case TASK_DISPERSE_READY:
      Motor_Stop();
      task_status.motors_active = false;
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

    case TASK_OPEN_FOR_RAM:
      if (!delivery_enter_command_ok(&vision.mission)) {
        /* Opening is safe to pause. Keep the state and wait for a fresh,
         * matching ENTER command instead of latching a communication fault. */
        Motor_Stop();
        task_status.motors_active = false;
      } else if (task_status.gripper_closed) {
        if (Claw_Open(now_ms)) {
          task_status.gripper_closed = false;
          Camera_SetAngle(APP_DELIVERY_CAMERA_ANGLE);
          camera_angle = (float)APP_DELIVERY_CAMERA_ANGLE;
          step_started_ms = now_ms;
        }
      } else if ((uint32_t)(now_ms - step_started_ms) >=
                 APP_DELIVERY_CAMERA_SETTLE_MS) {
        task_enter(TASK_RAM_VERIFY, now_ms);
      }
      break;

    case TASK_RAM_VERIFY:
      task_process_delivery_verify(&vision.mission, now_ms);
      break;

    case TASK_EXIT_SAFE_ZONE:
      {
        const MotorDistanceStatus result = Motor_MoveDistance(
            -APP_DELIVERY_EXIT_DISTANCE_M,
            APP_DELIVERY_EXIT_SPEED_MM_S);
        task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
        if (result == MOTOR_DISTANCE_DONE) {
          task_enter(TASK_FACE_FIELD_CENTER, now_ms);
        } else if (distance_failed(result)) {
          task_stop(TASK_FAULT_MOTOR, now_ms);
        }
      }
      break;

    case TASK_FACE_FIELD_CENTER:
      task_process_face_center(&vision, now_ms);
      break;

    case TASK_REMOTE_ACTION:
      task_process_remote_action(&vision.mission, now_ms);
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
  snapshot.audit_recheck_pending = cargo_recheck_pending;
  if (primask == 0U) {
    __enable_irq();
  }
  return snapshot;
}
