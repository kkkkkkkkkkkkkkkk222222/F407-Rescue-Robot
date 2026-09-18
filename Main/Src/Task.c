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
  SEARCH_RETURN_CAMERA_TO_120 = 0,
  SEARCH_RETURN_HOLD_120,
  SEARCH_CAMERA_TO_120,
  SEARCH_HOLD_120,
  SEARCH_SWEEP_120,
  SEARCH_CAMERA_TO_90,
  SEARCH_HOLD_90,
  SEARCH_SWEEP_90,
  SEARCH_WAIT_RETURN
} SearchPhase;

typedef enum {
  APPROACH_TRACK = 0,
  APPROACH_CLUSTER_ALIGN,
  APPROACH_CLUSTER_CAMERA_TO_140,
  APPROACH_CLUSTER_SETTLE_140,
  APPROACH_CLUSTER_CAPTURE,
  APPROACH_ALIGN_125,
  APPROACH_CAMERA_TO_140,
  APPROACH_SETTLE_140
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
  bool done;
  bool paused;
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

typedef enum {
  AUDIT_SEMANTIC_INVALID = 0,
  AUDIT_SEMANTIC_STASH_NONEMPTY,
  AUDIT_SEMANTIC_FIRST_GREEN,
  AUDIT_SEMANTIC_MATERIAL_LEGAL,
  AUDIT_SEMANTIC_INJURY_SINGLE
} AuditSemantic;

typedef enum {
  POST_AUDIT_WAIT_PRIMARY = 0,
  POST_AUDIT_NUDGE_LOW,
  POST_AUDIT_WAIT_LOW,
  POST_AUDIT_NUDGE_HIGH,
  POST_AUDIT_WAIT_HIGH,
  POST_AUDIT_RELEASE
} PostGrabAuditPhase;

typedef enum {
  SAFE_SWEEP_PARK_LOAD = 0,
  SAFE_SWEEP_RELEASE_LOAD,
  SAFE_SWEEP_RETURN_CENTER_OPEN,
  SAFE_SWEEP_APPROACH_OBSTACLE,
  SAFE_SWEEP_GRAB_OBSTACLE,
  SAFE_SWEEP_MOVE_OBSTACLE_ASIDE,
  SAFE_SWEEP_RELEASE_OBSTACLE,
  SAFE_SWEEP_RETURN_CENTER_AFTER_RELEASE,
  SAFE_SWEEP_REVERSE_TO_STAGE,
  SAFE_SWEEP_MOVE_TO_LOAD,
  SAFE_SWEEP_REGRAB_LOAD,
  SAFE_SWEEP_RECENTER_LOAD,
  SAFE_SWEEP_MODE39_HOLD
} SafeSweepPhase;

typedef enum {
  SWEEP_PICKUP_SEARCH = 0,
  SWEEP_PICKUP_TRACK,
  SWEEP_PICKUP_ALIGN_125,
  SWEEP_PICKUP_CAMERA_TO_140,
  SWEEP_PICKUP_SETTLE_140
} SweepPickupPhase;

/* Visual CLEAR has no lateral translations. Every placement is turn, drive,
 * release, reverse; every visual pickup retraces its own sampled path. */
typedef enum {
  VS_TURN_PARK, VS_PARK, VS_RELEASE_LOAD, VS_BACK_PARK, VS_FACE_OBSTACLE,
  VS_GRAB_OBSTACLE, VS_BACK_OBSTACLE, VS_FACE_H,
  VS_TURN_DROP, VS_DROP, VS_RELEASE_OBSTACLE, VS_BACK_DROP,
  VS_FACE_LOAD, VS_GRAB_LOAD, VS_BACK_LOAD, VS_FINAL_H,
  VS_FAILED_OPEN, VS_FAILED_BACK, VS_FAILED_H, VS_POST_AUDIT
} VisualSweepPhase;
typedef struct { int32_t x, y; float heading; } SweepTracePoint;
typedef enum {
  SWEEP_REVERSE_SELECT = 0, SWEEP_REVERSE_ALIGN, SWEEP_REVERSE_DRIVE,
  SWEEP_REVERSE_DONE, SWEEP_REVERSE_FINISHED
} SweepReversePhase;
typedef struct {
  SweepReversePhase phase;
  SweepTracePoint cursor;
  SweepTracePoint origin;
  SweepTracePoint target;
  uint16_t consume_to;
  uint16_t initial_count;
  float ux, uy, length, heading;
  bool rotation_only;
} SweepReverse;
static SweepReverse sweep_reverse;
static VisualSweepPhase visual_sweep_phase;
static SweepTracePoint sweep_trace[APP_SAFE_SWEEP_TRACE_CAPACITY];
static uint16_t sweep_trace_count;
static bool sweep_trace_recording;
static bool sweep_retrieving;
static bool sweep_original_injury;
static uint8_t sweep_original_left, sweep_original_right, sweep_original_total;
static uint32_t sweep_retrieve_started_ms;
static uint32_t sweep_budget_path_mm;
static LocationPose sweep_budget_pose;
static uint32_t sweep_forward_used_mm;
static uint32_t sweep_budget_exhausted_ms;
static bool sweep_budget_exhausted;
static bool sweep_forward_commanded;
static void task_sweep_fail(uint32_t now_ms);
static void task_sweep_sample(uint32_t now_ms);

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
static uint32_t nav_realign_started_ms;
static uint32_t return_search_ack_started_ms;
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
static float safe_align_target_deg;
static uint32_t tracking_tick_ms;
static uint32_t approach_report_generation;
static uint32_t approach_phase_path_mm;
static uint32_t tracking_report_generation;
static uint8_t mission_sequence;
static uint32_t start_reverse_path_mm;
static float start_target_heading_deg;
static uint32_t scan_entry_report_generation;
static uint32_t nav_final_push_start_path_mm;
static uint32_t safe_enter_start_path_mm;
static uint32_t nav_stage_start_path_mm;
static uint32_t nav_stage_limit_mm;
static uint32_t delivery_exit_start_path_mm;
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
static bool nav_realign_pending;
static bool safe_enter_path_valid;
static bool nav_stage_guard_valid;
static bool delivery_exit_path_valid;
static bool delivery_stage_only;
static bool safe_align_done;
static bool safe_align_visual_started;
static bool delivery_enter_active;
static bool first_delivery_done;
static bool first_green_bump_done;
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
static bool cluster_claws_ready;
/* +1 keeps the left cargo, -1 keeps the right cargo. */
static int8_t separation_keep_side;
static uint8_t audit_consistent_count;
static uint8_t audit_seen_ids[3];
static uint8_t audit_last_left_class;
static uint8_t audit_last_right_class;
static uint8_t audit_last_counts;
static uint8_t audit_last_flags;
static uint8_t audit_last_total_count;
static uint8_t audit_last_id;
static AuditSemantic audit_last_semantic;
static bool audit_core_seen_in_streak;
static bool grab_core_advance_required;
static bool grab_core_advance_started;
static uint32_t grab_core_advance_start_path_mm;
static float grab_core_advance_heading_deg;
static PostGrabAuditPhase post_grab_audit_phase;
static uint32_t post_grab_audit_phase_started_ms;
static uint32_t post_grab_invalid_started_ms;
static uint32_t approach_hold_started_ms;
static bool post_grab_invalid_waiting;
static bool approach_hold_pending;
static SafeSweepPhase safe_sweep_phase;
static int16_t safe_sweep_forward_mm;
static int16_t safe_sweep_lateral_mm;
static uint32_t safe_sweep_phase_path_mm;
static bool safe_sweep_phase_path_valid;
static float safe_sweep_heading_deg;
static bool safe_sweep_recheck_pending;
static bool safe_sweep_visual_pickup;
static int32_t safe_sweep_center_x_mm;
static int32_t safe_sweep_center_y_mm;
static SweepPickupPhase sweep_pickup_phase;
static bool sweep_pickup_target_active;
static uint8_t sweep_audit_consistent_count;
static uint8_t sweep_seen_ids[3];
static uint8_t sweep_audit_last_id;
static bool sweep_audit_last_id_valid;
static bool sweep_audit_valid;
static uint32_t sweep_audit_started_ms;
static float boundary_recovery_heading_deg;
static bool boundary_claw_opened;
static bool boundary_turn_done;
static uint8_t remote_target_sequence;
static uint32_t remote_target_generation;
static bool remote_target_sequence_valid;
static RemoteActionState remote_action;
static NavProgress nav_progress;

static void task_enter(TaskState next, uint32_t now_ms);
static bool distance_failed(MotorDistanceStatus result);
static bool delivery_enter_command_ok(const VisionMissionCommand *command);
static float nav_yaw(float heading_error_deg);
static bool task_safe_sweep_command_valid(
    const VisionMissionCommand *command);
static void task_reset_sweep_audit(uint32_t now_ms);

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

static float task_heading_360(float angle_deg)
{
  while (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  while (angle_deg >= 360.0f) {
    angle_deg -= 360.0f;
  }
  return angle_deg;
}

static float task_safe_zone_heading(void)
{
  const float heading_deg = (configured_color == VISION_COLOR_RED) ?
      APP_SAFE_ZONE_RED_HEADING_DEG : APP_SAFE_ZONE_BLUE_HEADING_DEG;
  return task_heading_360(heading_deg);
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
  if (fault == TASK_FAULT_REMOTE_STOP) {
    /* A new confirmed TYPE=0x11 configuration is the only supported restart
     * request after an operator ABORT/STOP. Hardware faults remain latched. */
    Vision_RearmConfig();
  }
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
  /* YIELD is the physical separation step after a confirmed unilateral
   * release.  Accepting it directly from APPROACH/NAV creates an ordinary
   * backoff with no re-audit flag, leaving the upper and lower state machines
   * waiting for different next events.  Other recovery uses ESCAPE. */
  return (state == TASK_REMOTE_ACTION) && remote_action.done &&
         cargo_recheck_pending &&
         ((remote_action.type == REMOTE_ACTION_RELEASE_LEFT) ||
          (remote_action.type == REMOTE_ACTION_RELEASE_RIGHT));
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
  /* Legacy danger avoidance is intentionally disabled. SEARCH/APPROACH and
   * the route to the 0.60 m staging point ignore claw-external objects; only
   * CLEAR_SAFE_ZONE may move cargo/obstacles after final visual ALIGN. */
  return false;
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
  if (state == TASK_ALIGN_SAFE_ZONE) {
    return safe_align_done ? 11U : 10U;
  }
  if (complete_flow_active) {
    if (state == TASK_APPROACH) {
      if (approach_phase == APPROACH_CLUSTER_CAPTURE) {
        return 38U;
      }
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

static AuditSemantic task_audit_semantic(
    const VisionMissionCommand *command)
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
  const bool initial_stash =
      (command->audit_flags & VISION_AUDIT_INITIAL_STASH) != 0U;
  const bool destination_injury =
      (command->audit_flags & VISION_AUDIT_DESTINATION_INJURY) != 0U;

  /* The opening temporary stash only clears the centre pile. Per-side counts
   * are saturated diagnostics and neither category nor quantity is a formal
   * delivery interlock for this move; only a non-empty stable audit matters. */
  if (initial_stash) {
    return (total > 0U) ?
        AUDIT_SEMANTIC_STASH_NONEMPTY : AUDIT_SEMANTIC_INVALID;
  }

  if ((total == 0U) || (total > 2U) ||
      ((uint8_t)(left_count + right_count) != total) ||
      dangerous || unknown || injury_mixed) {
    return AUDIT_SEMANTIC_INVALID;
  }
  if (!first_delivery_done) {
    return (!destination_injury && (total == 1U) &&
           ((task_audit_class_is(left, VISION_CARGO_GREEN) &&
             (left_count == 1U)) ||
            (task_audit_class_is(right, VISION_CARGO_GREEN) &&
             (right_count == 1U)))) ?
        AUDIT_SEMANTIC_FIRST_GREEN : AUDIT_SEMANTIC_INVALID;
  }
  if (injury) {
    return (destination_injury && (total == 1U)) ?
        AUDIT_SEMANTIC_INJURY_SINGLE : AUDIT_SEMANTIC_INVALID;
  }
  /* Match the upper computer's post-first-delivery policy: one to two
   * ordinary/core supplies may be transported together, including a side
   * reported as MIXED_MATERIAL. Danger, unknown and injury mixtures were
   * rejected above. */
  const bool material =
         !destination_injury &&
         ((left == VISION_CARGO_NONE) ||
          (left == VISION_CARGO_GREEN) ||
          (left == VISION_CARGO_CORE) ||
          (left == VISION_CARGO_MIXED_MATERIAL)) &&
         ((right == VISION_CARGO_NONE) ||
          (right == VISION_CARGO_GREEN) ||
          (right == VISION_CARGO_CORE) ||
          (right == VISION_CARGO_MIXED_MATERIAL));
  return material ? AUDIT_SEMANTIC_MATERIAL_LEGAL :
                    AUDIT_SEMANTIC_INVALID;
}

static bool task_audit_is_empty(const VisionMissionCommand *command)
{
  const uint8_t blocking_flags = VISION_AUDIT_DANGER_PRESENT |
      VISION_AUDIT_UNKNOWN_PRESENT | VISION_AUDIT_INJURY_MIXED;
  return (command->audit_total_count == 0U) &&
         ((command->audit_counts & 0x0FU) == 0U) &&
         (command->audit_left_class == VISION_CARGO_NONE) &&
         (command->audit_right_class == VISION_CARGO_NONE) &&
         ((command->audit_flags & blocking_flags) == 0U);
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

static bool task_disperse_keep_side_valid(
    const VisionMissionCommand *command)
{
  if ((command == NULL) ||
      ((command->flags & VISION_CMD_SIDE_VALID) == 0U)) {
    return true;
  }
  const uint8_t left_count = audit_last_counts & 0x03U;
  const uint8_t right_count = (audit_last_counts >> 2) & 0x03U;
  const bool selected_nonempty =
      ((command->flags & VISION_CMD_TARGET_RIGHT) != 0U) ?
          (right_count > 0U) : (left_count > 0U);
  const bool has_unassigned_cargo =
      ((audit_last_flags & VISION_AUDIT_UNKNOWN_PRESENT) != 0U) &&
      (audit_last_total_count > (uint8_t)(left_count + right_count));
  /* Aggressive 140-degree sorting may force an unassigned centre object to
   * the host-selected side. Explicit non-empty side counts remain preferred;
   * UNKNOWN_PRESENT is the only permitted fallback. */
  return selected_nonempty || has_unassigned_cargo;
}

static bool task_audit_has_green(void)
{
  return (audit_last_left_class == VISION_CARGO_GREEN) ||
         (audit_last_right_class == VISION_CARGO_GREEN) ||
         (audit_last_left_class == VISION_CARGO_MIXED_MATERIAL) ||
         (audit_last_right_class == VISION_CARGO_MIXED_MATERIAL);
}

static void task_begin_close_claw(uint32_t now_ms)
{
  const bool advance_for_core = complete_flow_active &&
      audit_core_seen_in_streak;
  task_enter(TASK_CLOSE_CLAW, now_ms);
  grab_core_advance_required = advance_for_core;
  grab_core_advance_started = false;
  grab_core_advance_start_path_mm = 0U;
  grab_core_advance_heading_deg = 0.0f;
}

static TaskCommandReject task_disperse_reject_reason(
    const VisionMissionCommand *command)
{
  const bool first_green_bump = (command != NULL) &&
      ((command->flags & VISION_CMD_FIRST_GREEN_BUMP) != 0U);
  if (!((state == TASK_DISPERSE_READY) ||
        (state == TASK_GRAB_OBSERVE) ||
        (state == TASK_POST_GRAB_AUDIT))) {
    return TASK_COMMAND_REJECT_STATE;
  }
  if (!audit_received) {
    return TASK_COMMAND_REJECT_AUDIT;
  }
  if (audit_last_total_count == 0U) {
    return TASK_COMMAND_REJECT_EMPTY;
  }
  if (audit_valid) {
    /* A locally legal batch must be grabbed and transported. Never let a
     * stale upper-computer selected_batch force further physical separation. */
    return TASK_COMMAND_REJECT_AUDIT;
  }
  if (first_green_bump &&
      ((state == TASK_POST_GRAB_AUDIT) || first_delivery_done ||
       first_green_bump_done || audit_initial_stash ||
       (audit_last_total_count < 2U) || !task_audit_has_green())) {
    return TASK_COMMAND_REJECT_AUDIT;
  }
  if (!task_disperse_keep_side_valid(command)) {
    return TASK_COMMAND_REJECT_SIDE;
  }
  return TASK_COMMAND_REJECT_NONE;
}

static void task_latch_audit(const VisionMissionCommand *command)
{
  const uint8_t semantic_flags = command->audit_flags &
      (uint8_t)~VISION_AUDIT_STABLE;
  const AuditSemantic semantic = task_audit_semantic(command);
  const bool current_has_core =
      (command->audit_left_class == VISION_CARGO_CORE) ||
      (command->audit_right_class == VISION_CARGO_CORE) ||
      (command->audit_left_class == VISION_CARGO_MIXED_MATERIAL) ||
      (command->audit_right_class == VISION_CARGO_MIXED_MATERIAL);
  bool same = false;
  if ((semantic != AUDIT_SEMANTIC_INVALID) &&
      (semantic == audit_last_semantic)) {
    /* Legal cargo is normalized by task meaning, not claw side or exact
     * ordinary/core distribution. Material batches only need the same total;
     * stash merely needs to remain non-empty. */
    same = (semantic == AUDIT_SEMANTIC_STASH_NONEMPTY) ||
           (command->audit_total_count == audit_last_total_count);
  } else if ((semantic == AUDIT_SEMANTIC_INVALID) &&
             (audit_last_semantic == AUDIT_SEMANTIC_INVALID)) {
    /* Invalid decisions are normalized to total count and illegal semantic
     * flags. mixed_material versus split green/core and left/right movement
     * must not make the two controllers disagree on the three-frame gate. */
    same = (semantic_flags == audit_last_flags) &&
           (command->audit_total_count == audit_last_total_count);
  }
  audit_initial_stash =
      (command->audit_flags & VISION_AUDIT_INITIAL_STASH) != 0U;
  audit_destination_injury =
      (command->audit_flags & VISION_AUDIT_DESTINATION_INJURY) != 0U;
  if (same) {
    /* The upper computer may retransmit one camera result with many UART
     * SEQs. Count only a changed audit_id as another visual frame. */
    bool seen = false;
    for (uint8_t i = 0U; i < audit_consistent_count && i < 3U; ++i) {
      seen = seen || (audit_seen_ids[i] == command->audit_id);
    }
    if (!seen && (audit_consistent_count < 3U)) {
      audit_seen_ids[audit_consistent_count++] = command->audit_id;
    }
  } else {
    audit_consistent_count = 1U;
    audit_seen_ids[0] = command->audit_id;
  }
  if (same) {
    audit_core_seen_in_streak =
        audit_core_seen_in_streak || current_has_core;
  } else {
    audit_core_seen_in_streak =
        (semantic != AUDIT_SEMANTIC_INVALID) && current_has_core;
  }
  /* Always preserve the newest side assignment for release/disperse choices,
   * while the normalized semantic above controls final GRAB continuity. */
  audit_last_left_class = command->audit_left_class;
  audit_last_right_class = command->audit_right_class;
  audit_last_counts = command->audit_counts;
  audit_last_flags = semantic_flags;
  audit_last_total_count = command->audit_total_count;
  audit_last_id = command->audit_id;
  audit_last_semantic = semantic;
  task_status.audit_left_class = command->audit_left_class;
  task_status.audit_right_class = command->audit_right_class;
  task_status.audit_total_count = command->audit_total_count;
  /* STABLE is descriptive only. Three distinct audit_id values are required
   * for every pre-grab, post-grab, cluster and separation audit. */
  audit_received = audit_consistent_count >= 3U;
  audit_valid = audit_received &&
      (semantic != AUDIT_SEMANTIC_INVALID);
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
  audit_last_id = 0U;
  audit_last_semantic = AUDIT_SEMANTIC_INVALID;
  audit_core_seen_in_streak = false;
  grab_core_advance_required = false;
  grab_core_advance_started = false;
  grab_core_advance_start_path_mm = 0U;
  grab_core_advance_heading_deg = 0.0f;
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
    .done = false,
    .paused = false,
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
  flags |= (((state == TASK_SAFE_SWEEP_AUDIT) && sweep_audit_valid) ||
            ((state != TASK_SAFE_SWEEP_AUDIT) &&
             audit_received && audit_valid)) ?
      VISION_STM_AUDIT_VALID : 0U;
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
  const bool return_just_completed =
      (state == TASK_FACE_FIELD_CENTER) && (next == TASK_SEARCH);
  Motor_Stop();
  state = next;
  if (next == TASK_STOPPED) {
    sweep_trace_recording = false;
    sweep_forward_commanded = false;
  }
  task_status.state = next;
  task_status.motors_active = false;
  task_status.auto_approach = false;
  task_status.nav_stale = false;
  task_status.nav_done = false;
  task_status.nav_final_push = false;
  task_status.nav_heading_locked = false;
  task_status.nav_locked_heading_deg = 0U;
  task_status.claw_visible =
      (((next >= TASK_GRAB_OBSERVE) && (next <= TASK_CLOSE_CLAW)) ||
       (next == TASK_POST_GRAB_AUDIT) ||
       (next == TASK_SAFE_SWEEP_AUDIT) ||
       (next == TASK_SAFE_SWEEP_RETRIEVE_AUDIT));
  state_started_ms = now_ms;
  step_started_ms = now_ms;
  pose_invalid_pending = false;
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
  nav_locked_heading_deg = 0.0f;
  safe_align_target_deg = 0.0f;
  nav_heading_locked = false;
  nav_final_push_active = false;
  nav_final_push_done = false;
  nav_final_push_paused = false;
  safe_enter_path_valid = false;
  safe_enter_start_path_mm = 0U;
  nav_stage_guard_valid = false;
  nav_stage_start_path_mm = 0U;
  nav_stage_limit_mm = 0U;
  delivery_exit_path_valid = false;
  delivery_exit_start_path_mm = 0U;
  delivery_stage_only = false;
  safe_align_done = false;
  safe_align_visual_started = false;
  delivery_enter_active = false;
  nav_terminal_candidate = false;
  nav_terminal_latched = false;
  nav_realign_pending = false;
  return_search_ack_started_ms =
      return_just_completed ? now_ms : 0U;
  if (next != TASK_APPROACH) {
    approach_hold_pending = false;
  }

  if (next == TASK_START) {
    const LocationPose pose = Location_GetPose();
    start_reverse_path_mm = pose.path_mm;
    start_target_heading_deg = task_wrap_angle(
        (float)pose.heading_mdeg * 0.001f + APP_START_TURN_DEG);
    start_clearance_done = false;
  } else if (next == TASK_SEARCH) {
    sweep_trace_recording = false;
    sweep_retrieving = false;
    sweep_forward_commanded = false;
    const VisionData vision = Vision_GetSnapshot();
    camera_angle = (float)Camera_GetAngle();
    task_status.found = false;
    locked_cargo_counts = 0U;
    search_phase = return_just_completed ?
        SEARCH_RETURN_CAMERA_TO_120 : SEARCH_CAMERA_TO_120;
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
    cluster_claws_ready = false;
    safe_sweep_recheck_pending = false;
    safe_sweep_visual_pickup = false;
    sweep_pickup_target_active = false;
    sweep_audit_consistent_count = 0U;
    sweep_audit_last_id_valid = false;
    sweep_audit_valid = false;
    audit_consistent_count = 0U;
    audit_last_left_class = 0U;
    audit_last_right_class = 0U;
    audit_last_counts = 0U;
    audit_last_flags = 0U;
    audit_last_total_count = 0U;
    audit_last_id = 0U;
    audit_last_semantic = AUDIT_SEMANTIC_INVALID;
    audit_core_seen_in_streak = false;
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
    cluster_claws_ready = false;
    approach_locked_heading_deg = 0.0f;
    approach_report_generation_valid = false;
    approach_hold_started_ms = now_ms;
    approach_hold_pending = false;
    task_reset_tracking();
  } else if (next == TASK_APPROACH_RECOVER) {
    const VisionData vision = Vision_GetSnapshot();
    scan_entry_report_generation = vision.report_generation;
    scan_report_gate_open = false;
    task_reset_tracking();
  } else if (next == TASK_GRAB_OBSERVE) {
    task_reset_tracking();
  } else if (next == TASK_POST_GRAB_AUDIT) {
    post_grab_audit_phase = POST_AUDIT_WAIT_PRIMARY;
    post_grab_audit_phase_started_ms = now_ms;
    post_grab_invalid_started_ms = now_ms;
    post_grab_invalid_waiting = false;
  } else if (next == TASK_SAFE_SWEEP) {
    safe_sweep_phase = SAFE_SWEEP_PARK_LOAD;
    safe_sweep_phase_path_mm = 0U;
    safe_sweep_phase_path_valid = false;
  } else if (next == TASK_SAFE_SWEEP_DONE) {
    task_status.gripper_closed = true;
    task_status.nav_heading_locked = true;
    task_status.nav_locked_heading_deg =
        (uint16_t)(safe_sweep_heading_deg + 0.5f) % 360U;
  } else if ((next == TASK_SAFE_SWEEP_APPROACH) ||
             (next == TASK_SAFE_SWEEP_RETRIEVE)) {
    sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
    sweep_pickup_target_active = false;
    approach_report_generation_valid = false;
    task_reset_tracking();
    Camera_SetAngle(APP_SEARCH_HIGH_CAMERA_ANGLE);
    camera_angle = (float)APP_SEARCH_HIGH_CAMERA_ANGLE;
  } else if ((next == TASK_SAFE_SWEEP_AUDIT) ||
             (next == TASK_SAFE_SWEEP_RETRIEVE_AUDIT)) {
    task_clear_audit_result();
    sweep_audit_consistent_count = 0U;
    sweep_audit_last_id_valid = false;
    sweep_audit_valid = false;
    sweep_audit_started_ms = now_ms;
    task_status.audit_ready = false;
    task_status.audit_valid = false;
    task_status.claw_visible = true;
  } else if (next == TASK_BOUNDARY_RECOVER) {
    sweep_trace_recording = false;
    sweep_retrieving = false;
    sweep_forward_commanded = false;
    safe_sweep_recheck_pending = false;
    task_clear_audit_result();
    nav_ready = false;
    boundary_claw_opened = false;
    boundary_turn_done = false;
  } else if (next == TASK_GRAB_ROTATE) {
    task_reset_turn_tracker();
  } else if ((next == TASK_NAVIGATE) ||
             (next == TASK_FACE_FIELD_CENTER)) {
    if (next == TASK_FACE_FIELD_CENTER) {
      /* Keep a medium-height view throughout every return route.  Once the
       * centre is reached SEARCH will explicitly visit 120 deg, then 90 deg,
       * before accepting a new target frame. */
      Camera_SetAngle(APP_SEARCH_HIGH_CAMERA_ANGLE);
      camera_angle = (float)APP_SEARCH_HIGH_CAMERA_ANGLE;
    }
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
  mission_sequence = 0U;
  start_reverse_path_mm = 0U;
  start_target_heading_deg = 0.0f;
  scan_entry_report_generation = 0U;
  nav_final_push_start_path_mm = 0U;
  safe_enter_start_path_mm = 0U;
  nav_stage_start_path_mm = 0U;
  nav_stage_limit_mm = 0U;
  delivery_exit_start_path_mm = 0U;
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
  safe_enter_path_valid = false;
  nav_stage_guard_valid = false;
  delivery_exit_path_valid = false;
  first_delivery_done = false;
  first_green_bump_done = false;
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
  cluster_claws_ready = false;
  separation_keep_side = 0;
  audit_consistent_count = 0U;
  audit_last_left_class = 0U;
  audit_last_right_class = 0U;
  audit_last_counts = 0U;
  audit_last_flags = 0U;
  audit_last_total_count = 0U;
  audit_last_id = 0U;
  audit_last_semantic = AUDIT_SEMANTIC_INVALID;
  audit_core_seen_in_streak = false;
  grab_core_advance_required = false;
  grab_core_advance_started = false;
  grab_core_advance_start_path_mm = 0U;
  grab_core_advance_heading_deg = 0.0f;
  post_grab_audit_phase = POST_AUDIT_WAIT_PRIMARY;
  post_grab_audit_phase_started_ms = now_ms;
  post_grab_invalid_started_ms = now_ms;
  approach_hold_started_ms = now_ms;
  post_grab_invalid_waiting = false;
  approach_hold_pending = false;
  safe_sweep_phase = SAFE_SWEEP_PARK_LOAD;
  safe_sweep_forward_mm = 0;
  safe_sweep_lateral_mm = 0;
  safe_sweep_phase_path_mm = 0U;
  safe_sweep_phase_path_valid = false;
  safe_sweep_heading_deg = 0.0f;
  safe_sweep_recheck_pending = false;
  safe_sweep_visual_pickup = false;
  safe_sweep_center_x_mm = 0;
  safe_sweep_center_y_mm = 0;
  sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
  sweep_pickup_target_active = false;
  sweep_audit_consistent_count = 0U;
  sweep_audit_last_id = 0U;
  sweep_audit_last_id_valid = false;
  sweep_audit_valid = false;
  sweep_audit_started_ms = now_ms;
  boundary_recovery_heading_deg = 0.0f;
  boundary_claw_opened = false;
  boundary_turn_done = false;
  remote_target_sequence = 0U;
  remote_target_generation = 0U;
  remote_target_sequence_valid = false;
  remote_action = (RemoteActionState){0};
  search_phase = SEARCH_CAMERA_TO_120;
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

static bool task_search_accepts_remote_target(void)
{
  /* A target confirmed exactly at the end of the second circle still wins
   * over RETURN. Only the post-return camera preparation suppresses it. */
  return (search_phase != SEARCH_RETURN_CAMERA_TO_120) &&
         (search_phase != SEARCH_RETURN_HOLD_120);
}

static bool task_approach_runs_without_target(void)
{
  if (state != TASK_APPROACH) {
    return false;
  }
  return (approach_phase == APPROACH_CLUSTER_CAMERA_TO_140) ||
         (approach_phase == APPROACH_CLUSTER_SETTLE_140) ||
         (approach_phase == APPROACH_CLUSTER_CAPTURE) ||
         (approach_phase == APPROACH_CAMERA_TO_140) ||
         (approach_phase == APPROACH_SETTLE_140);
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
  const bool return_camera_preparing =
      (search_phase == SEARCH_RETURN_CAMERA_TO_120) ||
      (search_phase == SEARCH_RETURN_HOLD_120);
  task_status.found = false;
  if (!return_camera_preparing) {
    task_status.found = task_scan_target_found(vision);
    if (task_status.found) {
      locked_cargo_counts = vision->cargo_counts;
      task_enter(TASK_APPROACH, now_ms);
      return;
    }
  }

  switch (search_phase) {
    case SEARCH_RETURN_CAMERA_TO_120:
      Motor_Stop();
      task_status.motors_active = false;
      if (task_scan_camera_to(APP_SEARCH_HIGH_CAMERA_ANGLE, now_ms)) {
        search_phase = SEARCH_RETURN_HOLD_120;
        step_started_ms = now_ms;
      }
      return;

    case SEARCH_RETURN_HOLD_120:
      Motor_Stop();
      task_status.motors_active = false;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_CAMERA_SCAN_ENDPOINT_HOLD_MS) {
        /* Frames seen on the return route or while the camera was moving do
         * not select the next target. Start the requested 120-degree sweep
         * only after one new report can pass this frame floor. */
        scan_entry_report_generation = vision->report_generation;
        scan_report_gate_open = false;
        search_phase = SEARCH_SWEEP_120;
        task_reset_turn_tracker();
      }
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
        search_phase = SEARCH_CAMERA_TO_90;
        task_reset_turn_tracker();
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;

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
        /* Two complete 120/90-degree sweeps found nothing. The upper computer
         * owns the RETURN command, but its T265 yaw accumulator may have
         * missed a few samples. Continue a slow scan instead of stopping at
         * 719.x degrees and creating a permanent two-controller wait. */
        search_phase = SEARCH_WAIT_RETURN;
        task_reset_turn_tracker();
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;

    case SEARCH_WAIT_RETURN:
      Motor_Move(0.0f, 0.0f, APP_SEARCH_WAIT_RETURN_SPEED_MM_S);
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
      if (cluster_target_active && !cluster_claws_ready) {
        Motor_Stop();
        task_status.motors_active = false;
        /* A pile is approached with both claws fully open.  Do not infer
         * cargo entry from camera angle or image Y and do not back away or
         * close both claws before the fresh 140-degree claw audit. */
        if (Claw_Open(now_ms)) {
          cluster_claws_ready = true;
          task_reset_tracking();
        }
        return;
      }
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
        task_clear_audit_result();
        approach_phase = APPROACH_CLUSTER_CAMERA_TO_140;
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, task_approach_steering());
      task_status.motors_active = task_abs(steering_mm_s) > 0.5f;
      return;

    case APPROACH_CLUSTER_CAMERA_TO_140:
      Motor_Stop();
      task_status.motors_active = false;
      if (task_scan_camera_to(APP_DISPERSE_CAPTURE_ANGLE, now_ms)) {
        approach_phase = APPROACH_CLUSTER_SETTLE_140;
        step_started_ms = now_ms;
      }
      return;

    case APPROACH_CLUSTER_SETTLE_140:
      Motor_Stop();
      task_status.motors_active = false;
      Camera_SetAngle(APP_DISPERSE_CAPTURE_ANGLE);
      camera_angle = (float)APP_DISPERSE_CAPTURE_ANGLE;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_GRAB_CAMERA_SETTLE_MS) {
        LocationPose pose;
        if (task_get_location_pose(&pose, now_ms)) {
          approach_phase_path_mm = pose.path_mm;
          approach_locked_heading_deg =
              (float)pose.heading_mdeg * 0.001f;
          task_clear_audit_result();
          task_status.claw_visible = true;
          approach_phase = APPROACH_CLUSTER_CAPTURE;
          step_started_ms = now_ms;
        }
      }
      return;

    case APPROACH_CLUSTER_CAPTURE:
      Camera_SetAngle(APP_DISPERSE_CAPTURE_ANGLE);
      camera_angle = (float)APP_DISPERSE_CAPTURE_ANGLE;
      if (audit_received && task_mission_valid(&vision->mission) &&
          (vision->mission.command == VISION_CMD_CARGO_AUDIT) &&
          (vision->mission.audit_total_count > 0U)) {
        Motor_Stop();
        task_status.motors_active = false;
        task_enter(TASK_DISPERSE_READY, now_ms);
        return;
      }
      {
        LocationPose pose;
        if (!task_get_location_pose(&pose, now_ms)) {
          return;
        }
        if (((pose.path_mm - approach_phase_path_mm) >=
             APP_DISPERSE_CAPTURE_DISTANCE_MM) ||
            ((uint32_t)(now_ms - step_started_ms) >=
             APP_DISPERSE_CAPTURE_TIMEOUT_MS)) {
          Motor_Stop();
          task_status.motors_active = false;
          task_status.claw_visible = false;
          cluster_target_active = false;
          task_enter(TASK_OPEN_CLAW, now_ms);
          return;
        }
        const float heading_deg = (float)pose.heading_mdeg * 0.001f;
        const float heading_error = task_wrap_angle(
            approach_locked_heading_deg - heading_deg);
        Motor_Move(APP_DISPERSE_CAPTURE_SPEED_MM_S, 0.0f,
                   nav_yaw(heading_error));
        task_status.motors_active = true;
      }
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
        /* Camera_GetAngle() is the commanded angle, not servo feedback. Give
         * the mechanism time to physically reach 140 deg before accepting
         * any frame as evidence that cargo is inside the claws. */
        approach_phase = APPROACH_SETTLE_140;
        step_started_ms = now_ms;
      }
      return;

    case APPROACH_SETTLE_140:
      Motor_Stop();
      task_status.motors_active = false;
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      camera_angle = (float)APP_GRAB_VIEW_ANGLE;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_GRAB_CAMERA_SETTLE_MS) {
        LocationPose pose;
        if (task_get_location_pose(&pose, now_ms)) {
          approach_phase_path_mm = pose.path_mm;
          approach_locked_heading_deg =
              (float)pose.heading_mdeg * 0.001f;
          /* Near-field cargo can disappear from the detector exactly when it
           * enters the claws. Open the claw-audit gate immediately instead of
           * requiring the old target track to reappear at 140 degrees. */
          task_clear_audit_result();
          task_enter(TASK_GRAB_OBSERVE, now_ms);
        }
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
    const VisionMissionCommand *command = &vision->mission;
    const bool three_frame_cargo_audit = task_mission_valid(command) &&
        (command->command == VISION_CMD_CARGO_AUDIT) &&
        (audit_consistent_count >= 3U) &&
        (command->audit_total_count > 0U);

    /* A post-release re-audit must remain stationary.  During an ordinary
     * pickup, however, CLAW_VISIBLE means "start inspecting", not "cargo is
     * already inside".  Crawl straight until the upper computer confirms a
     * stable non-empty claw audit, then stop before it sends GRAB. */
    if (cargo_recheck_pending || three_frame_cargo_audit) {
      Motor_Stop();
      task_status.motors_active = false;
      return;
    }

    LocationPose pose;
    if (!task_get_location_pose(&pose, now_ms)) {
      return;
    }
    if ((pose.path_mm - approach_phase_path_mm) >=
        APP_GRAB_REACQUIRE_DISTANCE_MM) {
      Motor_Stop();
      task_status.motors_active = false;
      task_enter(TASK_APPROACH_RECOVER, now_ms);
      return;
    }
    const float heading_deg = (float)pose.heading_mdeg * 0.001f;
    const float heading_error = task_wrap_angle(
        approach_locked_heading_deg - heading_deg);
    Motor_Move(APP_GRAB_WATCH_CRAWL_SPEED_MM_S, 0.0f,
               nav_yaw(heading_error));
    task_status.motors_active = true;
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

static void task_process_post_grab_audit(uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  task_status.gripper_closed = true;
  task_status.claw_visible = true;

  if (audit_received && !audit_valid) {
    if (!post_grab_invalid_waiting) {
      post_grab_invalid_waiting = true;
      post_grab_invalid_started_ms = now_ms;
    }
    if ((uint32_t)(now_ms - post_grab_invalid_started_ms) <
        APP_POST_GRAB_DECISION_WAIT_MS) {
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      camera_angle = (float)APP_GRAB_VIEW_ANGLE;
      return;
    }
    post_grab_audit_phase = POST_AUDIT_RELEASE;
  } else {
    post_grab_invalid_waiting = false;
  }

  switch (post_grab_audit_phase) {
    case POST_AUDIT_WAIT_PRIMARY:
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      if ((uint32_t)(now_ms - post_grab_audit_phase_started_ms) >=
          APP_POST_GRAB_AUDIT_WINDOW_MS) {
        Camera_SetAngle((uint8_t)(APP_GRAB_VIEW_ANGLE -
                                  APP_POST_GRAB_CAMERA_NUDGE_DEG));
        post_grab_audit_phase = POST_AUDIT_NUDGE_LOW;
        post_grab_audit_phase_started_ms = now_ms;
      }
      break;

    case POST_AUDIT_NUDGE_LOW:
      Camera_SetAngle((uint8_t)(APP_GRAB_VIEW_ANGLE -
                                APP_POST_GRAB_CAMERA_NUDGE_DEG));
      if ((uint32_t)(now_ms - post_grab_audit_phase_started_ms) >=
          APP_POST_GRAB_CAMERA_NUDGE_MS) {
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        task_clear_audit_result();
        post_grab_audit_phase = POST_AUDIT_WAIT_LOW;
        post_grab_audit_phase_started_ms = now_ms;
      }
      break;

    case POST_AUDIT_WAIT_LOW:
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      if ((uint32_t)(now_ms - post_grab_audit_phase_started_ms) >=
          APP_POST_GRAB_AUDIT_WINDOW_MS) {
        Camera_SetAngle((uint8_t)(APP_GRAB_VIEW_ANGLE +
                                  APP_POST_GRAB_CAMERA_NUDGE_DEG));
        post_grab_audit_phase = POST_AUDIT_NUDGE_HIGH;
        post_grab_audit_phase_started_ms = now_ms;
      }
      break;

    case POST_AUDIT_NUDGE_HIGH:
      Camera_SetAngle((uint8_t)(APP_GRAB_VIEW_ANGLE +
                                APP_POST_GRAB_CAMERA_NUDGE_DEG));
      if ((uint32_t)(now_ms - post_grab_audit_phase_started_ms) >=
          APP_POST_GRAB_CAMERA_NUDGE_MS) {
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        task_clear_audit_result();
        post_grab_audit_phase = POST_AUDIT_WAIT_HIGH;
        post_grab_audit_phase_started_ms = now_ms;
      }
      break;

    case POST_AUDIT_WAIT_HIGH:
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      if ((uint32_t)(now_ms - post_grab_audit_phase_started_ms) >=
          APP_POST_GRAB_AUDIT_WINDOW_MS) {
        post_grab_audit_phase = POST_AUDIT_RELEASE;
      }
      break;

    case POST_AUDIT_RELEASE:
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_clear_audit_result();
        task_enter(TASK_SEARCH, now_ms);
      }
      break;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      break;
  }
  camera_angle = (float)Camera_GetAngle();
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
  if ((command == NULL) || (command->command != expected_command)) {
    return false;
  }

  const uint8_t direction_flags = VISION_CMD_DRIVE_STRAIGHT |
                                  VISION_CMD_USE_FINAL_HEADING;
  const uint8_t direction = command->flags & direction_flags;
  const bool stage_nav =
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      ((command->flags & VISION_CMD_STAGE_ONLY) != 0U);
  const bool direction_valid = stage_nav ?
      ((direction == 0U) || (direction == direction_flags)) :
      (direction == direction_flags);

  return ((command->flags & VISION_CMD_DISTANCE_VALID) != 0U) &&
         direction_valid && task_side_flag_valid(command->flags) &&
         (command->target_x_mm >= 0) &&
         (command->target_x_mm <= APP_NAV_REMOTE_MAX_DISTANCE_MM) &&
         (command->target_y_mm == 0) &&
         (command->heading_cdeg < 36000U);
}

static bool task_stage_command(const VisionMissionCommand *command)
{
  return (command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
         ((command->flags & VISION_CMD_STAGE_ONLY) != 0U);
}

static bool task_align_command_valid(const VisionMissionCommand *command,
                                     bool visual)
{
  if (!task_mission_valid(command) ||
      (command->command != VISION_CMD_ALIGN_SAFE_ZONE) ||
      !task_side_flag_valid(command->flags)) {
    return false;
  }
  const bool command_visual =
      (command->flags & VISION_CMD_VISUAL_CORRECTION_VALID) != 0U;
  if (command_visual != visual) {
    return false;
  }
  if (visual) {
    return ((command->flags & VISION_CMD_USE_FINAL_HEADING) == 0U) &&
           (command->target_y_mm == 0) && (command->heading_cdeg == 0U);
  }
  return ((command->flags & VISION_CMD_USE_FINAL_HEADING) != 0U) &&
         (command->target_x_mm == 0) && (command->target_y_mm == 0) &&
         (command->heading_cdeg < 36000U);
}

static bool task_safe_sweep_command_valid(
    const VisionMissionCommand *command)
{
  if (!task_mission_valid(command) ||
      (command->command != VISION_CMD_CLEAR_SAFE_ZONE) ||
      !task_side_flag_valid(command->flags)) {
    return false;
  }
  const uint8_t allowed = VISION_CMD_VALID | VISION_CMD_RED_SIDE;
  const int32_t lateral_abs = (command->target_y_mm < 0) ?
      -(int32_t)command->target_y_mm : command->target_y_mm;
  return ((command->flags & (uint8_t)~allowed) == 0U) &&
         ((command->target_x_mm == 0) ||
          ((command->target_x_mm >= APP_SAFE_SWEEP_MIN_FORWARD_MM) &&
           (command->target_x_mm <= APP_SAFE_SWEEP_MAX_FORWARD_MM))) &&
         (lateral_abs == ((command->target_x_mm == 0) ?
             APP_SAFE_SWEEP_PLACEMENT_MM : APP_SAFE_SWEEP_LATERAL_MM)) &&
         (command->heading_cdeg == 0U);
}

static float task_visual_heading_correction(int16_t pixel_error)
{
  float correction_deg = APP_SAFE_VISUAL_YAW_SIGN *
      atanf((float)pixel_error / APP_SAFE_VISUAL_FOCAL_LENGTH_PX) *
      (180.0f / 3.14159265358979323846f);
  if (correction_deg > APP_SAFE_VISUAL_MAX_CORRECTION_DEG) {
    correction_deg = APP_SAFE_VISUAL_MAX_CORRECTION_DEG;
  } else if (correction_deg < -APP_SAFE_VISUAL_MAX_CORRECTION_DEG) {
    correction_deg = -APP_SAFE_VISUAL_MAX_CORRECTION_DEG;
  }
  return correction_deg;
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

static bool nav_realign_due(float heading_error_deg, float threshold_deg,
                            uint32_t hold_ms, uint32_t now_ms)
{
  if (task_abs(heading_error_deg) <= threshold_deg) {
    nav_realign_pending = false;
    return false;
  }
  if (!nav_realign_pending) {
    nav_realign_pending = true;
    nav_realign_started_ms = now_ms;
    return false;
  }
  if ((uint32_t)(now_ms - nav_realign_started_ms) < hold_ms) {
    return false;
  }
  nav_realign_pending = false;
  return true;
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
    nav_realign_pending = false;
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    return REMOTE_ROUTE_WAITING;
  }
  if (nav_final_push_active && nav_ready &&
      nav_realign_due(heading_error_deg, APP_NAV_FINAL_REALIGN_DEG,
                      APP_NAV_FINAL_REALIGN_HOLD_MS, now_ms)) {
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    return REMOTE_ROUTE_WAITING;
  }
  if (!nav_ready) {
    nav_realign_pending = false;
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
    nav_realign_pending = false;
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
  const bool stage_route =
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      task_stage_command(command) && !route_to_stash;
  if (stage_route && !nav_stage_guard_valid) {
    nav_stage_start_path_mm = pose.path_mm;
    nav_stage_limit_mm = (uint32_t)command->target_x_mm +
                         APP_NAV_STAGE_OVERRUN_MARGIN_MM;
    nav_stage_guard_valid = true;
  }
  const uint32_t stage_travelled_mm =
      (nav_stage_guard_valid && (pose.path_mm >= nav_stage_start_path_mm)) ?
          pose.path_mm - nav_stage_start_path_mm : 0U;
  const bool stage_encoder_arrived = stage_route && nav_stage_guard_valid &&
      (command->target_x_mm <=
       (int16_t)APP_NAV_STAGE_NEAR_DISTANCE_MM) &&
      (stage_travelled_mm >= nav_stage_limit_mm);
  if (stage_route &&
      (distance_command_done ||
       (command->target_x_mm <=
        (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM) ||
       stage_encoder_arrived)) {
    if (!distance_command_done) {
      /* The staging point is where the upper computer starts observing the
       * safe-zone entrance. Raise the camera before advertising arrival so
       * the following ALIGN handshake cannot wait forever on a low view. */
      Camera_SetAngle(APP_SAFE_ALIGN_CAMERA_ANGLE);
      camera_angle = (float)APP_SAFE_ALIGN_CAMERA_ANGLE;
    }
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    nav_stop_output();
    return REMOTE_ROUTE_REACHED;
  }
  if (stage_route && nav_stage_guard_valid &&
      (stage_travelled_mm >= nav_stage_limit_mm)) {
    /* The chassis has already exceeded the first advertised stage distance by
     * the configured margin, but the host still reports D > 30 mm.  Stop and
     * wait for a corrected D instead of driving indefinitely. */
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
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
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      !route_to_stash && !stage_route;
  const bool terminal_timeout = delivery_route &&
      nav_terminal_timeout(command->target_x_mm, now_ms);

  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float command_heading_deg = (float)command->heading_cdeg * 0.01f;
  const bool return_route =
      expected_command == VISION_CMD_RETURN_CENTER;
  const bool lock_allowed =
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      !route_to_stash && !stage_route;

  if (nav_heading_locked &&
      (!lock_allowed ||
       (!nav_final_push_active &&
        (command->target_x_mm >
         (int16_t)APP_NAV_HEADING_UNLOCK_DISTANCE_MM)))) {
    nav_heading_locked = false;
    task_status.nav_heading_locked = false;
    nav_realign_pending = false;
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
    nav_realign_pending = false;
    nav_stop_output();
  }

  const float desired_heading_deg = nav_heading_locked ?
      nav_locked_heading_deg : command_heading_deg;
  const float heading_error_deg = task_wrap_angle(
      desired_heading_deg - current_heading_deg);
  const float route_realign_deg = return_route ?
      APP_RETURN_CENTER_REALIGN_DEG : APP_NAV_REALIGN_DEG;
  bool realign_due = false;
  if (nav_ready && nav_heading_locked) {
    realign_due = nav_realign_due(
        heading_error_deg, APP_NAV_FINAL_REALIGN_DEG,
        APP_NAV_FINAL_REALIGN_HOLD_MS, now_ms);
  } else {
    nav_realign_pending = false;
    realign_due = nav_ready &&
        (task_abs(heading_error_deg) >= route_realign_deg);
  }
  if (realign_due) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
  }
  if (!nav_ready) {
    nav_realign_pending = false;
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

static void task_process_safe_align(uint32_t now_ms)
{
  if (safe_align_done) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }

  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    return;
  }
  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float error_deg = task_wrap_angle(
      safe_align_target_deg - current_heading_deg);

  /* Enter the aligned window at 1.5 degrees, but use a wider hold window
   * during the 300 ms camera/IMU settling period. Without this hysteresis a
   * small IMU oscillation repeatedly restarted ALIGN and left both computers
   * waiting at the staging point. */
  if (nav_ready &&
      (task_abs(error_deg) > APP_SAFE_ALIGN_HOLD_TOLERANCE_DEG)) {
    nav_ready = false;
  }
  if (!nav_ready) {
    if (!turn_to(safe_align_target_deg, current_heading_deg,
                 APP_SAFE_ALIGN_TOLERANCE_DEG, now_ms)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }
  const uint32_t settle_ms =
      (APP_SAFE_ALIGN_CAMERA_SETTLE_MS > APP_NAV_TURN_SETTLE_MS) ?
      APP_SAFE_ALIGN_CAMERA_SETTLE_MS : APP_NAV_TURN_SETTLE_MS;
  if ((uint32_t)(now_ms - step_started_ms) < settle_ms) {
    return;
  }

  Motor_Stop();
  task_status.motors_active = false;
  safe_align_done = true;
  task_status.nav_heading_locked = true;
  task_status.nav_locked_heading_deg =
      (uint16_t)(safe_align_target_deg + 0.5f) % 360U;
}

static float safe_enter_speed(int16_t fence_distance_mm)
{
  if (fence_distance_mm >= (int16_t)APP_SAFE_ENTER_SLOWDOWN_MM) {
    return APP_SAFE_ENTER_CRUISE_SPEED_MM_S;
  }
  const float span_mm = APP_SAFE_ENTER_SLOWDOWN_MM -
                        (float)APP_SAFE_ENTER_CONTACT_DISTANCE_MM;
  float ratio = ((float)fence_distance_mm -
                 (float)APP_SAFE_ENTER_CONTACT_DISTANCE_MM) / span_mm;
  if (ratio < 0.0f) {
    ratio = 0.0f;
  } else if (ratio > 1.0f) {
    ratio = 1.0f;
  }
  return APP_SAFE_ENTER_MIN_SPEED_MM_S +
      (APP_SAFE_ENTER_CRUISE_SPEED_MM_S -
       APP_SAFE_ENTER_MIN_SPEED_MM_S) * ratio;
}

static RemoteRouteStatus safe_enter_follow(
    const VisionMissionCommand *command, uint32_t now_ms)
{
  if (!delivery_enter_command_ok(command) || !nav_heading_locked) {
    return REMOTE_ROUTE_COMMAND_INVALID;
  }

  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    task_pause_final_push(now_ms);
    nav_stop_output();
    return REMOTE_ROUTE_WAITING;
  }
  if (nav_final_push_done) {
    Motor_Stop();
    task_status.motors_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    task_status.nav_final_push = false;
    nav_stop_output();
    return REMOTE_ROUTE_REACHED;
  }

  if (!safe_enter_path_valid) {
    /* ENTER is repeated with new SEQs, but this encoder origin belongs to the
     * physical action and is latched exactly once on its first valid pose. */
    safe_enter_start_path_mm = pose.path_mm;
    safe_enter_path_valid = true;
  }

  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float heading_error_deg = task_wrap_angle(
      nav_locked_heading_deg - current_heading_deg);
  const float yaw_mm_s = nav_yaw(heading_error_deg);

  const uint32_t contact_start_mm =
      APP_SAFE_ENTER_TOTAL_DISTANCE_MM -
      APP_SAFE_FINAL_PUSH_DISTANCE_MM;
  const uint32_t enter_travelled_mm =
      pose.path_mm - safe_enter_start_path_mm;

  if (!nav_final_push_active &&
      (enter_travelled_mm >= contact_start_mm)) {
    nav_final_push_active = true;
    nav_final_push_paused = false;
    nav_final_push_started_ms = now_ms;
    nav_final_push_start_path_mm = pose.path_mm;
    task_status.nav_final_push = true;
    /* From here onward localization is deliberately ignored. The encoder
     * push absorbs the final fence-distance error without repeated stops. */
  }

  if (nav_final_push_active) {
    if (nav_final_push_paused) {
      nav_final_push_started_ms += now_ms - nav_final_push_paused_ms;
      nav_final_push_paused = false;
    }
    const uint32_t elapsed_ms = now_ms - nav_final_push_started_ms;
    const uint32_t travelled_mm = pose.path_mm - nav_final_push_start_path_mm;
    if ((travelled_mm >= APP_SAFE_FINAL_PUSH_DISTANCE_MM) ||
        (elapsed_ms >= APP_SAFE_FINAL_PUSH_TIMEOUT_MS)) {
      Motor_Stop();
      task_status.motors_active = false;
      nav_final_push_active = false;
      nav_final_push_done = true;
      distance_command_done = true;
      task_status.nav_done = true;
      task_status.nav_final_push = false;
      nav_stop_output();
      return REMOTE_ROUTE_REACHED;
    }
    Motor_Move(APP_SAFE_FINAL_PUSH_SPEED_MM_S, 0.0f, yaw_mm_s);
    task_status.motors_active = true;
    return REMOTE_ROUTE_RUNNING;
  }

  const uint32_t estimated_fence_distance_mm =
      (enter_travelled_mm < contact_start_mm) ?
          APP_SAFE_ENTER_CONTACT_DISTANCE_MM +
              contact_start_mm - enter_travelled_mm :
          APP_SAFE_ENTER_CONTACT_DISTANCE_MM;
  const float target_speed_mm_s = safe_enter_speed(
      (int16_t)estimated_fence_distance_mm);
  const float speed_rate_mm_s2 =
      (target_speed_mm_s >= remote_speed_mm_s) ?
          APP_NAV_SPEED_ACCEL_MM_S2 : APP_NAV_SPEED_DECEL_MM_S2;
  remote_speed_mm_s = task_step_toward(
      remote_speed_mm_s, target_speed_mm_s,
      speed_rate_mm_s2 * APP_TASK_PERIOD_MS * 0.001f);
  Motor_Move(remote_speed_mm_s, 0.0f, yaw_mm_s);
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
    const RemoteRouteStatus result = delivery_enter_active ?
        safe_enter_follow(&vision->mission, now_ms) : nav_push(now_ms);
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
  if (delivery_stage_only != task_stage_command(&vision->mission)) {
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
  const bool visual = (command != NULL) &&
      ((command->flags & VISION_CMD_VISUAL_CORRECTION_VALID) != 0U);
  const bool red = (command != NULL) &&
      ((command->flags & VISION_CMD_RED_SIDE) != 0U);
  const uint16_t fallback_heading = red ? 9000U : 27000U;
  return task_mission_valid(command) &&
         (command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
         ((command->flags & VISION_CMD_DRIVE_STRAIGHT) != 0U) &&
         ((command->flags & VISION_CMD_DISTANCE_VALID) == 0U) &&
         task_side_flag_valid(command->flags) &&
         (command->target_x_mm == 0) &&
         (command->target_y_mm == 0) &&
         (visual ?
              (((command->flags & VISION_CMD_USE_FINAL_HEADING) == 0U) &&
               (command->heading_cdeg == 0U)) :
              (((command->flags & VISION_CMD_USE_FINAL_HEADING) != 0U) &&
               (command->heading_cdeg == fallback_heading)));
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
  /* mode35/30 re-audit is performed from the physical 140-degree claw view.
   * Advertise it only after the curve/turn and camera settling are complete. */
  if (cargo_recheck_pending) {
    task_status.claw_visible = true;
  }
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

static bool task_remote_separation_curve(uint32_t distance_mm,
                                         uint32_t now_ms)
{
  if ((distance_mm == 0U) || (separation_keep_side == 0)) {
    task_remote_action_advance(now_ms);
    return true;
  }

  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    Motor_Stop();
    task_status.motors_active = false;
    return false;
  }
  if ((pose.path_mm - remote_action.phase_path_mm) >= distance_mm) {
    task_remote_action_advance(now_ms);
    return true;
  }

  /* Peel the open claw away on the outside of one continuous arc.  The
   * retained side moves diagonally backward while the nose slowly turns
   * toward the released side, instead of first reversing in a straight line.
   * Motor_Move inputs are body-frame mm/s; signs mirror for left/right. */
  const float side_sign = (separation_keep_side > 0) ? 1.0f : -1.0f;
  Motor_Move(-APP_CARGO_SEPARATE_CURVE_BACK_MM_S,
             side_sign * APP_CARGO_SEPARATE_CURVE_SIDE_MM_S,
             -side_sign * APP_CARGO_SEPARATE_CURVE_YAW_MM_S);
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

static void task_safe_sweep_advance(uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  safe_sweep_phase = (SafeSweepPhase)((uint8_t)safe_sweep_phase + 1U);
  safe_sweep_phase_path_mm = 0U;
  safe_sweep_phase_path_valid = false;
  step_started_ms = now_ms;
}

static bool task_safe_sweep_move(float forward_mm_s, float lateral_mm_s,
                                 uint32_t distance_mm, uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    Motor_Stop();
    task_status.motors_active = false;
    return false;
  }
  if (!safe_sweep_phase_path_valid) {
    safe_sweep_phase_path_mm = pose.path_mm;
    safe_sweep_phase_path_valid = true;
  }
  const uint32_t travelled_mm =
      (pose.path_mm >= safe_sweep_phase_path_mm) ?
          pose.path_mm - safe_sweep_phase_path_mm : 0U;
  if (travelled_mm >= distance_mm) {
    task_safe_sweep_advance(now_ms);
    return true;
  }
  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float heading_error = task_wrap_angle(
      safe_sweep_heading_deg - current_heading_deg);
  Motor_Move(forward_mm_s, lateral_mm_s, nav_yaw(heading_error));
  task_status.motors_active = true;
  return false;
}

static void task_begin_safe_sweep(const VisionMissionCommand *command,
                                  uint32_t now_ms)
{
  const float locked_heading = safe_align_target_deg;
  const LocationPose pose = Location_GetPose();
  sweep_original_injury = audit_destination_injury;
  sweep_original_left = audit_last_left_class;
  sweep_original_right = audit_last_right_class;
  sweep_original_total = audit_last_total_count;
  task_status.sweep_original_left_class = sweep_original_left;
  task_status.sweep_original_right_class = sweep_original_right;
  task_status.sweep_original_total_count = sweep_original_total;
  task_enter(TASK_SAFE_SWEEP, now_ms);
  safe_sweep_forward_mm = command->target_x_mm;
  safe_sweep_lateral_mm = command->target_y_mm;
  safe_sweep_heading_deg = task_heading_360(locked_heading);
  safe_sweep_visual_pickup = command->target_x_mm == 0;
  safe_sweep_center_x_mm = pose.x_mm;
  safe_sweep_center_y_mm = pose.y_mm;
  safe_sweep_recheck_pending = true;
  visual_sweep_phase = VS_TURN_PARK;
  sweep_trace_recording = false;
  sweep_retrieving = false;
  sweep_forward_commanded = false;
  task_clear_audit_result();
  task_status.gripper_closed = true;
  task_status.nav_heading_locked = true;
  task_status.nav_locked_heading_deg =
      (uint16_t)(safe_sweep_heading_deg + 0.5f) % 360U;
}


static float task_sweep_side_heading(bool original)
{
  const float side = (safe_sweep_lateral_mm > 0) ? -90.0f : 90.0f;
  return task_heading_360(safe_sweep_heading_deg + (original ? side : -side));
}

static void task_sweep_next(VisualSweepPhase next, uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  sweep_forward_commanded = false;
  visual_sweep_phase = next;
  if ((next == VS_BACK_OBSTACLE) || (next == VS_BACK_LOAD) ||
      (next == VS_FAILED_BACK)) {
    const LocationPose pose = Location_GetPose();
    /* Freeze once at the physical backoff entry, never on CLEAR/HOLD. */
    sweep_trace_recording = false;
    sweep_reverse = (SweepReverse){
      .phase = SWEEP_REVERSE_SELECT,
      .cursor = {pose.x_mm, pose.y_mm, pose.heading_mdeg * 0.001f},
      .initial_count = sweep_trace_count
    };
  }
  safe_sweep_phase_path_valid = false;
  step_started_ms = now_ms;
}

static bool task_sweep_turn(float heading, uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) { return false; }
  const MotorTurnStatus result = Motor_TurnAngle(task_wrap_angle(
      heading - (float)pose.heading_mdeg * 0.001f));
  task_status.sweep_turn_status = (uint8_t)result;
  task_status.motors_active = result == MOTOR_TURN_RUNNING;
  if (result == MOTOR_TURN_DONE) {
    Motor_Stop();
    task_status.motors_active = false;
    return true;
  }
  if ((result == MOTOR_TURN_FAULT) || (result == MOTOR_TURN_INVALID)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  return false;
}

static bool task_sweep_stroke(float heading, bool reverse, uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) { return false; }
  if (!safe_sweep_phase_path_valid) {
    safe_sweep_phase_path_mm = pose.path_mm;
    safe_sweep_phase_path_valid = true;
  }
  uint32_t used = pose.path_mm - safe_sweep_phase_path_mm;
  if (used >= APP_SAFE_SWEEP_PLACEMENT_MM) {
    Motor_Stop();
    task_status.motors_active = false;
    return true;
  }
  float speed = fminf(APP_SAFE_SWEEP_FORWARD_SPEED_MM_S,
      fmaxf(30.0f, (APP_SAFE_SWEEP_PLACEMENT_MM - used) * 4.0f));
  Motor_Move(reverse ? -speed : speed, 0.0f, nav_yaw(task_wrap_angle(
      heading - (float)pose.heading_mdeg * 0.001f)));
  task_status.motors_active = true;
  return false;
}

static void task_sweep_trace_begin(void)
{
  const LocationPose pose = Location_GetPose();
  sweep_trace_count = 1U;
  sweep_trace[0] = (SweepTracePoint){safe_sweep_center_x_mm, safe_sweep_center_y_mm,
                                  (float)pose.heading_mdeg * 0.001f};
  sweep_trace_recording = true;
  sweep_forward_commanded = false;
  sweep_budget_path_mm = pose.path_mm;
  sweep_budget_pose = pose;
}

/* Sample before processing any new command, including HOLD/GRAB.
 * Positive measured translation consumes budget; backing never refunds it. */
static void task_sweep_sample(uint32_t now_ms)
{
  if (!sweep_trace_recording) { return; }
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) { return; }
  if (sweep_retrieving) {
    const float middle_yaw = (sweep_budget_pose.heading_mdeg * 0.001f +
        task_wrap_angle((pose.heading_mdeg - sweep_budget_pose.heading_mdeg) *
                        0.001f) * 0.5f) * (3.14159265358979323846f / 180.0f);
    const float forward = (pose.x_mm - sweep_budget_pose.x_mm) * cosf(middle_yaw) +
                          (pose.y_mm - sweep_budget_pose.y_mm) * sinf(middle_yaw);
    /* Measure real displacement, including coasting after stop/HOLD. Use
     * path length conservatively for forward/rounded-zero displacement;
     * a backward segment never subtracts from the consumed allowance. */
    if (forward >= 0.0f) {
      sweep_forward_used_mm += pose.path_mm - sweep_budget_path_mm;
    }
  }
  sweep_budget_path_mm = pose.path_mm;
  sweep_budget_pose = pose;
  task_status.sweep_forward_used_mm = (uint16_t)sweep_forward_used_mm;
  if (sweep_retrieving && !sweep_budget_exhausted &&
      (sweep_forward_used_mm >= APP_SAFE_SWEEP_RETRIEVE_BUDGET_MM)) {
    sweep_budget_exhausted = true;
    sweep_budget_exhausted_ms = now_ms;
    Motor_Stop();
    sweep_forward_commanded = false;
  }
  const SweepTracePoint last = sweep_trace[sweep_trace_count - 1U];
  const float dx = (float)(pose.x_mm - last.x);
  const float dy = (float)(pose.y_mm - last.y);
  const float yaw = (float)pose.heading_mdeg * 0.001f;
  if ((dx * dx + dy * dy >= APP_SAFE_SWEEP_TRACE_STEP_MM *
                                APP_SAFE_SWEEP_TRACE_STEP_MM) ||
      (task_abs(task_wrap_angle(yaw - last.heading)) >= 5.0f)) {
    if (sweep_trace_count >= APP_SAFE_SWEEP_TRACE_CAPACITY) {
      /* Stop while the entire recorded route is still recoverable. */
      task_sweep_fail(now_ms);
      return;
    }
    sweep_trace[sweep_trace_count++] =
        (SweepTracePoint){pose.x_mm, pose.y_mm, yaw};
  }
}

/* Select/merge from the frozen route. The cursor is the last CONSUMED
 * endpoint, not the robot's jittering position. Count changes only in DONE. */
static void task_sweep_select_segment(const LocationPose *pose)
{
  uint16_t end = sweep_trace_count - 1U;
  const SweepTracePoint origin = sweep_reverse.cursor;
  SweepTracePoint target = sweep_trace[end];
  float dx = origin.x - target.x, dy = origin.y - target.y;
  float length = sqrtf(dx * dx + dy * dy);
  const bool rotation = length <= APP_SAFE_SWEEP_SEGMENT_MERGE_MM;
  if (rotation) {
    /* Coalesce one genuine in-place turn, but not a reversal of turn sense. */
    float sense = task_wrap_angle(target.heading - origin.heading);
    while (end > 0U) {
      const SweepTracePoint candidate = sweep_trace[end - 1U];
      dx = origin.x - candidate.x;
      dy = origin.y - candidate.y;
      const float delta = task_wrap_angle(candidate.heading - target.heading);
      if ((dx * dx + dy * dy > APP_SAFE_SWEEP_SEGMENT_MERGE_MM *
                                    APP_SAFE_SWEEP_SEGMENT_MERGE_MM) ||
          ((sense * delta < 0.0f) && task_abs(delta) > 0.5f)) {
        break;
      }
      if (task_abs(sense) < 0.5f) { sense = delta; }
      target = candidate;
      --end;
    }
  } else {
    const float ux = dx / length, uy = dy / length;
    while (end > 0U) {
      const SweepTracePoint candidate = sweep_trace[end - 1U];
      const float ex = target.x - candidate.x, ey = target.y - candidate.y;
      const float cx = origin.x - candidate.x, cy = origin.y - candidate.y;
      /* Keep corners, in-place rotations and reversed translation separate.
       * Every skipped point must remain near the SAME locked line/yaw. */
      if ((ex * ux + ey * uy <= 0.0f) ||
          task_abs(cx * uy - cy * ux) > APP_SAFE_SWEEP_SEGMENT_MERGE_MM ||
          task_abs(task_wrap_angle(candidate.heading - origin.heading)) >
              APP_SAFE_SWEEP_SEGMENT_MERGE_DEG ||
          task_abs(task_wrap_angle(candidate.heading - target.heading)) >
              APP_SAFE_SWEEP_SEGMENT_MERGE_DEG) {
        break;
      }
      target = candidate;
      --end;
    }
    dx = origin.x - target.x;
    dy = origin.y - target.y;
    length = sqrtf(dx * dx + dy * dy);
  }
  sweep_reverse.origin = origin;
  sweep_reverse.target = target;
  sweep_reverse.consume_to = end;
  sweep_reverse.rotation_only = rotation;
  sweep_reverse.length = rotation ? 0.0f : length;
  sweep_reverse.ux = rotation ? 0.0f : dx / length;
  sweep_reverse.uy = rotation ? 0.0f : dy / length;
  /* atan2 is evaluated ONCE per nonzero segment, never near its endpoint. */
  sweep_reverse.heading = rotation ? target.heading :
      task_heading_360(atan2f(dy, dx) * (180.0f / 3.14159265358979323846f));
  const float error = task_wrap_angle(sweep_reverse.heading -
                                      pose->heading_mdeg * 0.001f);
  sweep_reverse.phase = task_abs(error) > APP_SAFE_SWEEP_SEGMENT_ALIGN_DEG ?
      SWEEP_REVERSE_ALIGN : rotation ? SWEEP_REVERSE_DONE : SWEEP_REVERSE_DRIVE;
}

static bool task_sweep_reverse_trace(uint32_t now_ms)
{
  if (sweep_reverse.phase == SWEEP_REVERSE_FINISHED) { return true; }
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) { return false; }
  task_status.sweep_current_x_mm = pose.x_mm;
  task_status.sweep_current_y_mm = pose.y_mm;
  task_status.sweep_current_yaw_mdeg = pose.heading_mdeg;
  task_status.sweep_heading_error_deg = task_wrap_angle(sweep_reverse.heading -
                                                       pose.heading_mdeg * 0.001f);
  switch (sweep_reverse.phase) {
    case SWEEP_REVERSE_SELECT:
      if (sweep_trace_count == 0U) {
        sweep_reverse.phase = SWEEP_REVERSE_FINISHED;
        return true;
      }
      task_sweep_select_segment(&pose);
      task_status.sweep_target_x_mm = sweep_reverse.target.x;
      task_status.sweep_target_y_mm = sweep_reverse.target.y;
      task_status.sweep_segment_length_mm = sweep_reverse.length;
      task_status.sweep_segment_heading_deg = sweep_reverse.heading;
      task_status.sweep_segment_remaining_mm = sweep_reverse.length;
      task_status.sweep_reverse_forward_mm_s = 0.0f;
      task_status.sweep_reverse_yaw_mm_s = 0.0f;
      return false;

    case SWEEP_REVERSE_ALIGN:
      /* No Motor_Move or distance branch may interrupt this latched turn. */
      if (task_sweep_turn(sweep_reverse.heading, now_ms)) {
        sweep_reverse.phase = sweep_reverse.rotation_only ?
            SWEEP_REVERSE_DONE : SWEEP_REVERSE_DRIVE;
      }
      return false;

    case SWEEP_REVERSE_DRIVE: {
      const float dx = pose.x_mm - sweep_reverse.origin.x;
      const float dy = pose.y_mm - sweep_reverse.origin.y;
      const float progress = -(dx * sweep_reverse.ux + dy * sweep_reverse.uy);
      const float cross = -dx * sweep_reverse.uy + dy * sweep_reverse.ux;
      const float remaining = sweep_reverse.length - progress;
      task_status.sweep_segment_remaining_mm = remaining;
      task_status.sweep_segment_cross_mm = cross;
      const float end_tolerance = sweep_reverse.consume_to == 0U ?
          APP_SAFE_SWEEP_RETURN_TOLERANCE_MM : APP_SAFE_SWEEP_SEGMENT_END_MM;
      const float cross_tolerance = sweep_reverse.consume_to == 0U ?
          APP_SAFE_SWEEP_RETURN_TOLERANCE_MM : APP_SAFE_SWEEP_SEGMENT_CROSS_MM;
      /* Reaching or crossing the endpoint plane is permanent. Cross-track
       * error gates completion, but never flips the target heading by 180. */
      if (((remaining <= end_tolerance) || (progress >= sweep_reverse.length)) &&
          (task_abs(cross) <= cross_tolerance)) {
        Motor_Stop();
        task_status.motors_active = false;
        task_status.sweep_reverse_forward_mm_s = 0.0f;
        task_status.sweep_reverse_yaw_mm_s = 0.0f;
        sweep_reverse.phase = SWEEP_REVERSE_DONE;
        return false;
      }
      float correction = atanf(cross / APP_SAFE_SWEEP_SEGMENT_LOOKAHEAD_MM) *
                         (180.0f / 3.14159265358979323846f);
      correction = fmaxf(-15.0f, fminf(15.0f, correction));
      const float error = task_wrap_angle(sweep_reverse.heading + correction -
                                          pose.heading_mdeg * 0.001f);
      /* Positive cross requires a positive body yaw when travelling BACK. */
      task_status.sweep_reverse_forward_mm_s =
          -fminf(120.0f, fmaxf(20.0f, remaining * 4.0f));
      task_status.sweep_reverse_yaw_mm_s = nav_yaw(error);
      Motor_Move(task_status.sweep_reverse_forward_mm_s, 0.0f,
                 task_status.sweep_reverse_yaw_mm_s);
      task_status.motors_active = true;
      return false;
    }

    case SWEEP_REVERSE_DONE:
      /* Consume once. Position noise can never reactivate this segment. */
      sweep_trace_count = sweep_reverse.consume_to;
      sweep_reverse.cursor = sweep_reverse.target;
      sweep_reverse.phase = SWEEP_REVERSE_SELECT;
      return false;

    case SWEEP_REVERSE_FINISHED:
      return true;
  }
  return false;
}

static void task_sweep_fail(uint32_t now_ms)
{
  sweep_trace_recording = false;
  sweep_forward_commanded = false;
  task_clear_audit_result();
  task_enter(TASK_SAFE_SWEEP, now_ms);
  task_sweep_next(VS_FAILED_OPEN, now_ms);
}

static void task_process_visual_sweep(uint32_t now_ms)
{
  const float park = task_sweep_side_heading(true);
  const float drop = task_sweep_side_heading(false);
  switch (visual_sweep_phase) {
    case VS_TURN_PARK:
      if (task_sweep_turn(park, now_ms)) task_sweep_next(VS_PARK, now_ms);
      break;
    case VS_PARK:
      if (task_sweep_stroke(park, false, now_ms)) task_sweep_next(VS_RELEASE_LOAD, now_ms);
      break;
    case VS_RELEASE_LOAD:
    case VS_RELEASE_OBSTACLE:
    case VS_FAILED_OPEN:
      if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_sweep_next(visual_sweep_phase == VS_RELEASE_LOAD ? VS_BACK_PARK :
            visual_sweep_phase == VS_RELEASE_OBSTACLE ? VS_BACK_DROP : VS_FAILED_BACK, now_ms);
      }
      break;
    case VS_BACK_PARK:
      if (task_sweep_stroke(park, true, now_ms)) task_sweep_next(VS_FACE_OBSTACLE, now_ms);
      break;
    case VS_FACE_OBSTACLE:
      if (task_sweep_turn(safe_sweep_heading_deg, now_ms)) {
        task_sweep_trace_begin();
        task_enter(TASK_SAFE_SWEEP_APPROACH, now_ms);
      }
      break;
    case VS_GRAB_OBSTACLE:
    case VS_GRAB_LOAD:
      if (Claw_Touch(now_ms)) {
        task_status.gripper_closed = true;
        sweep_trace_recording = false;
        task_sweep_next(visual_sweep_phase == VS_GRAB_LOAD ? VS_BACK_LOAD : VS_BACK_OBSTACLE, now_ms);
      }
      break;
    case VS_BACK_OBSTACLE:
    case VS_BACK_LOAD:
    case VS_FAILED_BACK:
      if (task_sweep_reverse_trace(now_ms)) {
        task_sweep_next(visual_sweep_phase == VS_BACK_OBSTACLE ? VS_FACE_H :
            visual_sweep_phase == VS_BACK_LOAD ? VS_FINAL_H : VS_FAILED_H, now_ms);
      }
      break;
    case VS_FACE_H:
      if (task_sweep_turn(safe_sweep_heading_deg, now_ms)) task_sweep_next(VS_TURN_DROP, now_ms);
      break;
    case VS_TURN_DROP:
      if (task_sweep_turn(drop, now_ms)) task_sweep_next(VS_DROP, now_ms);
      break;
    case VS_DROP:
      if (task_sweep_stroke(drop, false, now_ms)) task_sweep_next(VS_RELEASE_OBSTACLE, now_ms);
      break;
    case VS_BACK_DROP:
      if (task_sweep_stroke(drop, true, now_ms)) task_sweep_next(VS_FACE_LOAD, now_ms);
      break;
    case VS_FACE_LOAD:
      /* Absolute original-load heading: from drop this is a full 180 deg. */
      if (task_sweep_turn(park, now_ms)) {
        sweep_retrieving = true;
        sweep_forward_used_mm = 0U;
        sweep_budget_exhausted = false;
        sweep_retrieve_started_ms = now_ms;
        task_sweep_trace_begin();
        task_enter(TASK_SAFE_SWEEP_RETRIEVE, now_ms);
      }
      break;
    case VS_FINAL_H:
    case VS_FAILED_H:
      if (task_sweep_turn(safe_sweep_heading_deg, now_ms)) {
        if (visual_sweep_phase == VS_FAILED_H) {
          safe_sweep_recheck_pending = false;
          sweep_retrieving = false;
          task_clear_audit_result();
          task_enter(TASK_SAFE_SWEEP_RETRIEVE_FAILED, now_ms);
        } else {
          task_sweep_next(VS_POST_AUDIT, now_ms);
        }
      }
      break;
    case VS_POST_AUDIT:
      if (now_ms - step_started_ms >= APP_SAFE_SWEEP_MODE39_HOLD_MS) {
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        task_reset_sweep_audit(now_ms);
        task_clear_audit_result();
        task_enter(TASK_POST_GRAB_AUDIT, now_ms);
        task_status.gripper_closed = true;
      }
      break;
    default: task_stop(TASK_FAULT_INVALID_STATE, now_ms); break;
  }
}

static void task_process_safe_sweep(uint32_t now_ms)
{
  if (safe_sweep_visual_pickup) {
    task_process_visual_sweep(now_ms);
    return;
  }
  const float side_sign = (safe_sweep_lateral_mm >= 0) ? 1.0f : -1.0f;
  const float lateral_speed = side_sign *
      APP_SAFE_SWEEP_LATERAL_SPEED_MM_S;
  switch (safe_sweep_phase) {
    case SAFE_SWEEP_PARK_LOAD:
      (void)task_safe_sweep_move(
          0.0f, lateral_speed, APP_SAFE_SWEEP_LATERAL_MM, now_ms);
      return;

    case SAFE_SWEEP_RELEASE_LOAD:
      Motor_Stop();
      task_status.motors_active = false;
      if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_safe_sweep_advance(now_ms);
      }
      return;

    case SAFE_SWEEP_RETURN_CENTER_OPEN:
      (void)task_safe_sweep_move(
          0.0f, -lateral_speed, APP_SAFE_SWEEP_LATERAL_MM, now_ms);
      return;

    case SAFE_SWEEP_APPROACH_OBSTACLE:
      (void)task_safe_sweep_move(
          APP_SAFE_SWEEP_FORWARD_SPEED_MM_S, 0.0f,
          (uint32_t)safe_sweep_forward_mm, now_ms);
      return;

    case SAFE_SWEEP_GRAB_OBSTACLE:
      Motor_Stop();
      task_status.motors_active = false;
      if (Claw_Touch(now_ms)) {
        task_status.gripper_closed = true;
        safe_sweep_phase = SAFE_SWEEP_MOVE_OBSTACLE_ASIDE;
        safe_sweep_phase_path_valid = false;
        step_started_ms = now_ms;
      }
      return;

    case SAFE_SWEEP_MOVE_OBSTACLE_ASIDE:
      (void)task_safe_sweep_move(
          0.0f, -lateral_speed, APP_SAFE_SWEEP_LATERAL_MM, now_ms);
      return;

    case SAFE_SWEEP_RELEASE_OBSTACLE:
      Motor_Stop();
      task_status.motors_active = false;
      if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_safe_sweep_advance(now_ms);
      }
      return;

    case SAFE_SWEEP_RETURN_CENTER_AFTER_RELEASE:
      (void)task_safe_sweep_move(
          0.0f, lateral_speed, APP_SAFE_SWEEP_LATERAL_MM, now_ms);
      return;

    case SAFE_SWEEP_REVERSE_TO_STAGE:
      (void)task_safe_sweep_move(
          -APP_SAFE_SWEEP_RETURN_SPEED_MM_S, 0.0f,
          (uint32_t)safe_sweep_forward_mm, now_ms);
      return;

    case SAFE_SWEEP_MOVE_TO_LOAD:
      (void)task_safe_sweep_move(
          0.0f, lateral_speed, APP_SAFE_SWEEP_LATERAL_MM, now_ms);
      return;

    case SAFE_SWEEP_REGRAB_LOAD:
      Motor_Stop();
      task_status.motors_active = false;
      if (Claw_Touch(now_ms)) {
        task_status.gripper_closed = true;
        task_safe_sweep_advance(now_ms);
      }
      return;

    case SAFE_SWEEP_RECENTER_LOAD:
      (void)task_safe_sweep_move(
          0.0f, -lateral_speed, APP_SAFE_SWEEP_LATERAL_MM, now_ms);
      return;

    case SAFE_SWEEP_MODE39_HOLD:
      Motor_Stop();
      task_status.motors_active = false;
      task_status.gripper_closed = true;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_SAFE_SWEEP_MODE39_HOLD_MS) {
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        camera_angle = (float)APP_GRAB_VIEW_ANGLE;
        task_reset_sweep_audit(now_ms);
        task_clear_audit_result();
        task_enter(TASK_POST_GRAB_AUDIT, now_ms);
        task_status.gripper_closed = true;
      }
      return;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      return;
  }
}

static void task_reset_sweep_audit(uint32_t now_ms)
{
  sweep_audit_consistent_count = 0U;
  sweep_audit_last_id = 0U;
  sweep_audit_last_id_valid = false;
  sweep_audit_valid = false;
  sweep_audit_started_ms = now_ms;
  task_status.audit_ready = false;
  task_status.audit_valid = false;
  task_status.audit_left_class = VISION_CARGO_NONE;
  task_status.audit_right_class = VISION_CARGO_NONE;
  task_status.audit_total_count = 0U;
}

static void task_latch_sweep_audit(const VisionMissionCommand *command,
                                   uint32_t now_ms)
{
  (void)now_ms;
  if ((command->audit_flags & VISION_AUDIT_SWEEP_PICKUP) == 0U) {
    return;
  }
  if (command->audit_total_count == 0U) {
    sweep_audit_consistent_count = 0U;
    sweep_audit_last_id_valid = false;
    sweep_audit_valid = false;
    task_status.audit_ready = false;
    task_status.audit_valid = false;
    task_status.audit_left_class = VISION_CARGO_NONE;
    task_status.audit_right_class = VISION_CARGO_NONE;
    task_status.audit_total_count = 0U;
    return;
  }
  bool seen = false;
  for (uint8_t i = 0U; i < sweep_audit_consistent_count && i < 3U; ++i) {
    seen = seen || (sweep_seen_ids[i] == command->audit_id);
  }
  if (!seen && (sweep_audit_consistent_count < 3U)) {
    sweep_seen_ids[sweep_audit_consistent_count++] = command->audit_id;
    sweep_audit_last_id = command->audit_id;
    sweep_audit_last_id_valid = true;
  }
  sweep_audit_valid = sweep_audit_consistent_count >= 3U;
  task_status.audit_ready = sweep_audit_valid;
  task_status.audit_valid = sweep_audit_valid;
  task_status.audit_left_class = command->audit_left_class;
  task_status.audit_right_class = command->audit_right_class;
  task_status.audit_total_count = command->audit_total_count;
}

static void task_sweep_drive(float forward, float yaw)
{
  if (sweep_retrieving) {
    uint32_t remaining = (sweep_forward_used_mm >= APP_SAFE_SWEEP_RETRIEVE_BUDGET_MM) ?
        0U : APP_SAFE_SWEEP_RETRIEVE_BUDGET_MM - sweep_forward_used_mm;
    /* Slow the final stroke before the sampled odometry limit. */
    forward = fminf(forward, (float)remaining * 4.0f);
    if (remaining == 0U) { forward = 0.0f; }
  }
  sweep_forward_commanded = forward > 0.0f;
  Motor_Move(forward, 0.0f, yaw);
  task_status.motors_active = (forward > 0.0f) || (task_abs(yaw) > 0.5f);
}

static bool task_sweep_retrieve_expired(uint32_t now_ms)
{
  return sweep_retrieving &&
      ((now_ms - sweep_retrieve_started_ms >= APP_SAFE_SWEEP_RETRIEVE_TIMEOUT_MS) ||
       (sweep_budget_exhausted && (now_ms - sweep_budget_exhausted_ms >=
                                  APP_SAFE_SWEEP_OBSERVE_TIMEOUT_MS)));
}

static void task_process_safe_sweep_approach(const VisionData *vision,
                                             uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    sweep_forward_commanded = false;
    return;
  }
  if (task_sweep_retrieve_expired(now_ms)) {
    task_sweep_fail(now_ms);
    return;
  }
  sweep_forward_commanded = false;
  task_status.auto_approach = true;
  const bool new_report = !approach_report_generation_valid ||
      (vision->report_generation != approach_report_generation);
  const bool target_visible = sweep_pickup_target_active &&
      task_report_valid(vision);
  if (new_report) {
    approach_report_generation = vision->report_generation;
    approach_report_generation_valid = true;
  }

  if (!sweep_pickup_target_active &&
      (sweep_pickup_phase != SWEEP_PICKUP_CAMERA_TO_140) &&
      (sweep_pickup_phase != SWEEP_PICKUP_SETTLE_140)) {
    sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
  }

  switch (sweep_pickup_phase) {
    case SWEEP_PICKUP_SEARCH:
      task_status.found = false;
      if (target_visible && new_report) {
        task_reset_tracking();
        approach_speed_mm_s = APP_SAFE_SWEEP_APPROACH_SPEED_MM_S;
        sweep_pickup_phase = SWEEP_PICKUP_TRACK;
        (void)task_track_target(vision, true);
        return;
      }
      Camera_SetAngle(APP_SEARCH_HIGH_CAMERA_ANGLE);
      camera_angle = (float)APP_SEARCH_HIGH_CAMERA_ANGLE;
      if (sweep_retrieving) {
        const LocationPose pose = Location_GetPose();
        task_sweep_drive(APP_SAFE_SWEEP_RETRIEVE_SEARCH_MM_S,
            nav_yaw(task_wrap_angle(task_sweep_side_heading(true) -
                                   pose.heading_mdeg * 0.001f)));
      } else {
        task_sweep_drive(0.0f, APP_SAFE_SWEEP_SEARCH_SPEED_MM_S);
      }
      return;

    case SWEEP_PICKUP_TRACK:
      if (!target_visible) {
        sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
        task_reset_tracking();
        return;
      }
      if (new_report) {
        task_status.found = true;
        approach_speed_mm_s = task_approach_speed(vision);
        (void)task_track_target(vision, true);
      }
      if (Camera_GetAngle() >= APP_GRAB_ALIGN_CAMERA_ANGLE) {
        Camera_SetAngle(APP_GRAB_ALIGN_CAMERA_ANGLE);
        camera_angle = (float)APP_GRAB_ALIGN_CAMERA_ANGLE;
        sweep_pickup_phase = SWEEP_PICKUP_ALIGN_125;
        Motor_Stop();
        task_status.motors_active = false;
        return;
      }
      task_sweep_drive(task_approach_aligned_speed(approach_speed_mm_s),
                       task_approach_steering());
      return;

    case SWEEP_PICKUP_ALIGN_125:
      Camera_SetAngle(APP_GRAB_ALIGN_CAMERA_ANGLE);
      camera_angle = (float)APP_GRAB_ALIGN_CAMERA_ANGLE;
      if (!target_visible) {
        sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
        task_reset_tracking();
        return;
      }
      if (new_report) {
        task_status.found = true;
        (void)task_track_target(vision, false);
      }
      if (tracking_filter_valid &&
          (task_abs(filtered_target_x - (float)APP_VISION_TARGET_X) <=
           APP_GRAB_ALIGN_X_ERROR_PX)) {
        Motor_Stop();
        task_status.motors_active = false;
        sweep_pickup_phase = SWEEP_PICKUP_CAMERA_TO_140;
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, task_approach_steering());
      task_status.motors_active = task_abs(steering_mm_s) > 0.5f;
      return;

    case SWEEP_PICKUP_CAMERA_TO_140:
      Motor_Stop();
      task_status.motors_active = false;
      if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
        sweep_pickup_phase = SWEEP_PICKUP_SETTLE_140;
        step_started_ms = now_ms;
      }
      return;

    case SWEEP_PICKUP_SETTLE_140:
      Motor_Stop();
      task_status.motors_active = false;
      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
      camera_angle = (float)APP_GRAB_VIEW_ANGLE;
      if ((uint32_t)(now_ms - step_started_ms) >=
          APP_GRAB_CAMERA_SETTLE_MS) {
        task_enter(sweep_retrieving ? TASK_SAFE_SWEEP_RETRIEVE_AUDIT :
                                     TASK_SAFE_SWEEP_AUDIT, now_ms);
        task_reset_sweep_audit(now_ms);
      }
      return;

    default:
      task_stop(TASK_FAULT_INVALID_STATE, now_ms);
      return;
  }
}

static void task_process_safe_sweep_audit(uint32_t now_ms)
{
  sweep_forward_commanded = false;
  Motor_Stop();
  task_status.motors_active = false;
  task_status.auto_approach = true;
  task_status.claw_visible = true;
  Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
  camera_angle = (float)APP_GRAB_VIEW_ANGLE;
  if (state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT) {
    if (task_sweep_retrieve_expired(now_ms)) {
      task_sweep_fail(now_ms);
    } else if (!audit_valid &&
               (now_ms - sweep_audit_started_ms >= APP_SAFE_SWEEP_AUDIT_TIMEOUT_MS)) {
      task_enter(TASK_SAFE_SWEEP_RETRIEVE, now_ms);
    }
    return;
  }
  if (!sweep_audit_valid &&
      ((uint32_t)(now_ms - sweep_audit_started_ms) >=
       APP_SAFE_SWEEP_AUDIT_TIMEOUT_MS)) {
    task_enter(TASK_SAFE_SWEEP_APPROACH, now_ms);
    sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
    sweep_pickup_target_active = false;
  }
}

static void task_begin_sweep_obstacle_grab(uint32_t now_ms)
{
  const bool original = state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT;
  task_enter(TASK_SAFE_SWEEP, now_ms);
  visual_sweep_phase = original ? VS_GRAB_LOAD : VS_GRAB_OBSTACLE;
  sweep_forward_commanded = false;
  safe_sweep_phase = SAFE_SWEEP_GRAB_OBSTACLE;
  safe_sweep_phase_path_valid = false;
  task_status.gripper_closed = false;
  task_status.claw_visible = false;
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
          APP_REMOTE_DISPERSE_TIMEOUT_MS : APP_REMOTE_ACTION_TIMEOUT_MS;
  if ((uint32_t)(now_ms - remote_action.started_ms) >= action_timeout_ms) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
    return;
  }

  switch (remote_action.type) {
    case REMOTE_ACTION_RELEASE_LEFT:
      if (Claw_OpenLeft(now_ms)) {
        /* Left is fully open; retained right uses the firm 127-degree hold. */
        separation_keep_side = -1;
        task_status.gripper_closed = false;
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_RELEASE_RIGHT:
      if (Claw_OpenRight(now_ms)) {
        /* Right is fully open; retained left uses the firm 53-degree hold. */
        separation_keep_side = 1;
        task_status.gripper_closed = false;
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_RELEASE_BOTH:
      if (Claw_Open(now_ms)) {
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
        const uint32_t magnitude_mm = (uint32_t)(
            (requested_mm < 0) ? -requested_mm : requested_mm);
        if (remote_action.phase == 0U) {
          (void)task_remote_separation_curve(magnitude_mm, now_ms);
        } else if (remote_action.phase == 1U) {
          /* After the curved separation, look back into the claw before
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
      if (remote_action.arg_b == 2) {
        /* First formal green only: create a short run-up, close both claws as
         * a rigid bumper, push through the centre pile, return to the same
         * start line, reopen, and resume the 120-degree SEARCH locally. */
        if (remote_action.phase == 0U) {
          (void)task_remote_distance(
              -APP_FIRST_GREEN_BUMP_BACKOFF_M,
              APP_FIRST_GREEN_BUMP_SPEED_MM_S, now_ms);
        } else if (remote_action.phase == 1U) {
          Motor_Stop();
          task_status.motors_active = false;
          if (Claw_Touch(now_ms)) {
            task_status.gripper_closed = false;
            task_remote_action_advance(now_ms);
          }
        } else if (remote_action.phase == 2U) {
          (void)task_remote_distance(
              APP_FIRST_GREEN_BUMP_FORWARD_M,
              APP_FIRST_GREEN_BUMP_SPEED_MM_S, now_ms);
        } else if (remote_action.phase == 3U) {
          (void)task_remote_distance(
              -APP_FIRST_GREEN_BUMP_BACKOFF_M,
              APP_FIRST_GREEN_BUMP_SPEED_MM_S, now_ms);
        } else {
          Motor_Stop();
          task_status.motors_active = false;
          if (Claw_Open(now_ms)) {
            task_status.gripper_closed = false;
            cargo_recheck_pending = false;
            cluster_target_active = false;
            task_clear_audit_result();
            task_enter(TASK_SEARCH, now_ms);
          }
        }
        return;
      }
      if (remote_action.arg_b == 1) {
        if (remote_action.phase == 0U) {
          /* When the host cannot distinguish left from right, do not ram or
           * guess a claw. Shift the view by a small in-place turn, then ask
           * for a fresh 140-degree claw audit. */
          (void)task_remote_turn(APP_DISPERSE_OBSERVE_TURN_DEG, now_ms);
        } else if (remote_action.phase == 1U) {
          if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
            task_remote_action_advance(now_ms);
          }
        } else if ((uint32_t)(now_ms - step_started_ms) >=
                   APP_CARGO_RECHECK_SETTLE_MS) {
          cargo_recheck_pending = true;
          task_clear_audit_result();
          task_remote_action_finish();
        }
        return;
      }
      if (remote_action.phase == 0U) {
        /* Every SIDE_VALID pile curve uses the same 15-degree hold. Ordinary
         * RELEASE_LEFT/RIGHT remains the separate 25-degree strong-release
         * path and is intentionally not selected here. */
        const bool ready = (separation_keep_side > 0) ?
            Claw_ClusterOpenRight(now_ms) :
            Claw_ClusterOpenLeft(now_ms);
        if (ready) {
          task_status.gripper_closed = false;
          task_remote_action_advance(now_ms);
        }
      } else if (remote_action.phase == 1U) {
        (void)task_remote_separation_curve(
            APP_SELECTIVE_DISPERSE_CURVE_MM, now_ms);
      } else if (remote_action.phase == 2U) {
        if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
          task_remote_action_advance(now_ms);
        }
      } else if ((uint32_t)(now_ms - step_started_ms) >=
                 APP_CARGO_RECHECK_SETTLE_MS) {
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
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
  task_status.command_reject_reason = TASK_COMMAND_REJECT_NONE;

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
    if ((state == TASK_APPROACH) &&
        !task_approach_runs_without_target() &&
        !approach_hold_pending) {
      approach_hold_pending = true;
      approach_hold_started_ms = now_ms;
    }
    if ((state == TASK_BOUNDARY_RECOVER) ||
        (state == TASK_SAFE_SWEEP) ||
        (state == TASK_SAFE_SWEEP_AUDIT) ||
        (state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT) ||
        (state == TASK_SAFE_SWEEP_RETRIEVE_FAILED)) {
      /* ACK/heartbeat only. Keep the common PAUSE-resume epilogue below. */
    } else if ((state == TASK_SAFE_SWEEP_APPROACH) ||
               (state == TASK_SAFE_SWEEP_RETRIEVE)) {
      sweep_pickup_target_active = false;
      if ((sweep_pickup_phase != SWEEP_PICKUP_CAMERA_TO_140) &&
          (sweep_pickup_phase != SWEEP_PICKUP_SETTLE_140)) {
        sweep_pickup_phase = SWEEP_PICKUP_SEARCH;
        task_reset_tracking();
      }
    } else if ((state == TASK_NAVIGATE) && route_to_stash &&
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
               (((remote_action.type == REMOTE_ACTION_DISPERSE) &&
                 !cargo_recheck_pending) ||
                ((remote_action.type == REMOTE_ACTION_RELEASE_BOTH) &&
                 ((remote_action.arg_b == 1) ||
                  !stash_backoff_pending)))) {
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
             ((state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT) ||
              ((state == TASK_POST_GRAB_AUDIT) && safe_sweep_recheck_pending &&
               safe_sweep_visual_pickup)) &&
             (((command->audit_flags & (VISION_AUDIT_SWEEP_PICKUP |
                                        VISION_AUDIT_INITIAL_STASH)) != 0U) ||
              (((command->audit_flags & VISION_AUDIT_DESTINATION_INJURY) != 0U) !=
               sweep_original_injury))) {
    task_clear_audit_result();
    task_status.command_reject_reason = TASK_COMMAND_REJECT_AUDIT;
  } else if ((command->command == VISION_CMD_CARGO_AUDIT) &&
             (state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT)) {
    task_status.acknowledged_sequence = command->sequence;
    task_latch_audit(command);
    /* Remain in mode45 even when legal; only a fresh GRAB closes the claw. */
  } else if ((state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT) &&
             ((command->command == VISION_CMD_RELEASE_LEFT) ||
              (command->command == VISION_CMD_RELEASE_RIGHT) ||
              (command->command == VISION_CMD_RELEASE_BOTH) ||
              (command->command == VISION_CMD_DISPERSE_PILE))) {
    /* Do not enter ordinary separation motions outside the shared retrieval
     * budget. An explicit disposal request abandons this pickup safely. */
    task_status.acknowledged_sequence = command->sequence;
    task_sweep_fail(now_ms);
  } else if ((command->command == VISION_CMD_CARGO_AUDIT) &&
             (state == TASK_SAFE_SWEEP_AUDIT) &&
             ((command->audit_flags &
               VISION_AUDIT_SWEEP_PICKUP) != 0U)) {
    task_status.acknowledged_sequence = command->sequence;
    task_latch_sweep_audit(command, now_ms);
  } else if ((command->command == VISION_CMD_CARGO_AUDIT) &&
             ((command->audit_flags &
               VISION_AUDIT_SWEEP_PICKUP) != 0U)) {
    /* The special permissive audit is valid only inside mode43. Never let it
     * bypass ordinary cargo, initial-stash or post-grab legality rules. */
    task_status.command_reject_reason = TASK_COMMAND_REJECT_STATE;
  } else if ((command->command == VISION_CMD_CARGO_AUDIT) &&
              (((state >= TASK_GRAB_OBSERVE) &&
                (state <= TASK_GRAB_ROTATE)) ||
               (state == TASK_POST_GRAB_AUDIT) ||
               ((state == TASK_APPROACH) &&
                (approach_phase == APPROACH_CLUSTER_CAPTURE)) ||
               ((state == TASK_REMOTE_ACTION) && remote_action.done &&
                cargo_recheck_pending))) {
    task_status.acknowledged_sequence = command->sequence;
    if (state == TASK_REMOTE_ACTION) {
      task_enter(TASK_GRAB_OBSERVE, now_ms);
    }
    task_latch_audit(command);
    if ((state == TASK_POST_GRAB_AUDIT) && audit_received) {
      if (task_audit_is_empty(command)) {
        safe_sweep_recheck_pending = false;
        task_status.gripper_closed = false;
        task_clear_audit_result();
        task_enter(TASK_OPEN_CLAW, now_ms);
      } else if (audit_valid) {
        task_enter(safe_sweep_recheck_pending ?
                   TASK_SAFE_SWEEP_DONE : TASK_WAIT_NAVIGATION, now_ms);
      }
    } else if (cargo_recheck_pending && audit_received &&
        task_audit_is_empty(command)) {
      /* Nothing remained after unilateral separation.  There is no cargo to
       * grab, release or navigate with, so reopen and resume SEARCH locally
       * instead of leaving both controllers waiting in the audit state.  A
       * non-STABLE all-zero packet is only the upper computer's temporary
       * "no observation" placeholder and must not trigger this transition. */
      cargo_recheck_pending = false;
      task_status.gripper_closed = false;
      task_enter(TASK_OPEN_CLAW, now_ms);
    }
  } else if ((command->command == VISION_CMD_APPROACH_TARGET) &&
             ((state == TASK_SAFE_SWEEP_APPROACH) ||
              (state == TASK_SAFE_SWEEP_RETRIEVE))) {
    task_status.acknowledged_sequence = command->sequence;
    sweep_pickup_target_active = true;
    if (sweep_pickup_phase == SWEEP_PICKUP_SEARCH) {
      sweep_pickup_phase = SWEEP_PICKUP_TRACK;
      task_reset_tracking();
    }
  } else if ((command->command == VISION_CMD_APPROACH_TARGET) &&
              !stash_backoff_pending &&
              !cargo_recheck_pending &&
              (((state == TASK_SEARCH) &&
                task_search_accepts_remote_target()) ||
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
    approach_hold_pending = false;
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
             (state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT) &&
             !task_sweep_retrieve_expired(now_ms) &&
             audit_received && audit_valid && (audit_consistent_count >= 3U)) {
    task_status.acknowledged_sequence = command->sequence;
    task_begin_sweep_obstacle_grab(now_ms);
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             (state == TASK_SAFE_SWEEP_AUDIT) &&
             sweep_audit_valid) {
    task_status.acknowledged_sequence = command->sequence;
    task_begin_sweep_obstacle_grab(now_ms);
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             (state == TASK_SAFE_SWEEP) &&
             safe_sweep_visual_pickup &&
             (safe_sweep_phase >= SAFE_SWEEP_GRAB_OBSTACLE)) {
    /* The upper computer repeats GRAB until it sees mode39. ACK without
     * restarting the close, pose-return or obstacle-placement sequence. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             (state == TASK_DISPERSE_READY) &&
             audit_received && audit_valid &&
             (audit_consistent_count >= 3U)) {
    /* A clustered target that proves to contain one legal retained cargo no
     * longer needs separation. Close normally and continue to navigation. */
    task_status.acknowledged_sequence = command->sequence;
    cluster_target_active = false;
    task_begin_close_claw(now_ms);
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             (state >= TASK_GRAB_OBSERVE) &&
             (state <= TASK_GRAB_ROTATE) &&
             (!complete_flow_active ||
              (audit_received && audit_valid &&
               (audit_consistent_count >= 3U)))) {
    task_status.acknowledged_sequence = command->sequence;
    if (cargo_recheck_pending) {
      /* Backoff has physically separated the released cargo.  Close both
       * claws to the ordinary touch angles again so the retained cargo cannot
       * slip out during navigation. */
      cargo_recheck_pending = false;
    }
    task_begin_close_claw(now_ms);
  } else if ((command->command == VISION_CMD_GRAB_CONFIRMED) &&
             ((state == TASK_CLOSE_CLAW) ||
              (state == TASK_WAIT_NAVIGATION))) {
    /* GRAB_CONFIRMED is now repeated until GRIPPER_CLOSED is reported.  ACK
     * every new sequence, but never restart an in-progress/completed motion. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             task_status.gripper_closed &&
             audit_received && audit_valid &&
             ((state == TASK_WAIT_NAVIGATION) ||
              ((state == TASK_REMOTE_ACTION) && remote_action.done))) {
    task_status.acknowledged_sequence = command->sequence;
    route_to_stash = audit_initial_stash;
    task_enter(TASK_NAVIGATE, now_ms);
    route_to_stash = audit_initial_stash;
    delivery_stage_only = !route_to_stash && task_stage_command(command);
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             !task_status.gripper_closed && (state == TASK_SEARCH) &&
             task_distance_command_valid(command,
                                         VISION_CMD_NAVIGATE_WAYPOINT) &&
             !task_stage_command(command)) {
    /* The current upper-computer RETURN_STASH state reuses NAV because the
     * wire protocol has no separate empty-claw waypoint opcode.  Constrain
     * this compatibility entry to SEARCH + open claws + a fully validated
     * distance/heading payload, and mark it as a stash route so safe-zone
     * heading lock and final pushing can never run. */
    task_status.acknowledged_sequence = command->sequence;
    route_to_stash = true;
    task_enter(TASK_NAVIGATE, now_ms);
    route_to_stash = true;
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT) &&
             (state == TASK_NAVIGATE) &&
             (delivery_stage_only == task_stage_command(command))) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             task_status.gripper_closed &&
             (state == TASK_NAVIGATE) && delivery_stage_only &&
             task_align_command_valid(command, false)) {
    /* ALIGN is itself an explicit upper-computer staging-arrival decision.
     * Normally D=0/DISTANCE_DONE is observed first, but accepting ALIGN here
     * also covers the host's map-tolerance branch without a deadlock. */
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_ALIGN_SAFE_ZONE, now_ms);
    Camera_SetAngle(APP_SAFE_ALIGN_CAMERA_ANGLE);
    camera_angle = (float)APP_SAFE_ALIGN_CAMERA_ANGLE;
    safe_align_target_deg = task_heading_360(
        (float)command->heading_cdeg * 0.01f);
    safe_align_done = false;
    safe_align_visual_started = false;
    nav_ready = false;
    task_status.nav_locked_heading_deg =
        (uint16_t)(safe_align_target_deg + 0.5f) % 360U;
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (state == TASK_ALIGN_SAFE_ZONE) &&
             task_align_command_valid(command, false) &&
             !safe_align_visual_started) {
    /* The upper computer repeats the pose ALIGN while collecting three
     * frames. ACK it without restarting an already completed turn. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (state == TASK_ALIGN_SAFE_ZONE) && safe_align_done &&
             task_align_command_valid(command, true)) {
    task_status.acknowledged_sequence = command->sequence;
    if (!safe_align_visual_started) {
      safe_align_target_deg = task_heading_360(
          safe_align_target_deg +
          task_visual_heading_correction(command->target_x_mm));
      safe_align_visual_started = true;
      safe_align_done = false;
      nav_ready = false;
      task_status.nav_heading_locked = false;
      task_status.nav_locked_heading_deg =
          (uint16_t)(safe_align_target_deg + 0.5f) % 360U;
    }
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (state == TASK_ALIGN_SAFE_ZONE) &&
             safe_align_visual_started &&
             task_align_command_valid(command, true)) {
    /* Repeated visual ALIGN frames are idempotent: never add the same frozen
     * pixel correction more than once. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (state == TASK_SAFE_SWEEP_DONE) &&
             task_status.gripper_closed &&
             task_align_command_valid(command, false)) {
    /* Sweeping returns the recovered cargo to the original staging centre.
     * Re-run pose and visual ALIGN before ENTER; never reuse the heading from
     * a long lateral/forward obstacle-clearing motion. */
    task_status.acknowledged_sequence = command->sequence;
    safe_sweep_recheck_pending = false;
    task_enter(TASK_ALIGN_SAFE_ZONE, now_ms);
    Camera_SetAngle(APP_SAFE_ALIGN_CAMERA_ANGLE);
    camera_angle = (float)APP_SAFE_ALIGN_CAMERA_ANGLE;
    safe_align_target_deg = task_heading_360(
        (float)command->heading_cdeg * 0.01f);
    safe_align_done = false;
    safe_align_visual_started = false;
    nav_ready = false;
    task_status.nav_locked_heading_deg =
        (uint16_t)(safe_align_target_deg + 0.5f) % 360U;
  } else if ((command->command == VISION_CMD_CLEAR_SAFE_ZONE) &&
             (state == TASK_ALIGN_SAFE_ZONE) && safe_align_done &&
             safe_align_visual_started &&
             task_status.gripper_closed &&
             ((command->target_x_mm != 0) ||
              ((command->target_y_mm < 0) == audit_destination_injury)) &&
             task_safe_sweep_command_valid(command)) {
    /* The upper computer may request this only after its final visual ALIGN
     * has identified an obstacle outside the safe-zone polygon. */
    task_status.acknowledged_sequence = command->sequence;
    task_begin_safe_sweep(command, now_ms);
  } else if ((command->command == VISION_CMD_CLEAR_SAFE_ZONE) &&
             ((state == TASK_SAFE_SWEEP) ||
              (state == TASK_SAFE_SWEEP_DONE) ||
              (state == TASK_SAFE_SWEEP_APPROACH) ||
              (state == TASK_SAFE_SWEEP_AUDIT) ||
              (state == TASK_SAFE_SWEEP_RETRIEVE) ||
              (state == TASK_SAFE_SWEEP_RETRIEVE_AUDIT) ||
              (state == TASK_SAFE_SWEEP_RETRIEVE_FAILED)) &&
             task_safe_sweep_command_valid(command)) {
    /* Repeated frames are ACKed but cannot restart either the physical sweep
     * or its completed hand-off state. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
             task_status.gripper_closed &&
             (state == TASK_ALIGN_SAFE_ZONE) && safe_align_done &&
             delivery_enter_command_ok(command) &&
             ((((command->flags & VISION_CMD_VISUAL_CORRECTION_VALID) != 0U) &&
               safe_align_visual_started) ||
              (((command->flags & VISION_CMD_VISUAL_CORRECTION_VALID) == 0U) &&
               !safe_align_visual_started))) {
    const float final_heading_deg = safe_align_target_deg;
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_NAVIGATE, now_ms);
    delivery_enter_active = true;
    delivery_stage_only = false;
    nav_locked_heading_deg = task_heading_360(final_heading_deg);
    nav_heading_locked = true;
    nav_ready = true;
    task_status.nav_heading_locked = true;
    task_status.nav_locked_heading_deg =
        (uint16_t)(nav_locked_heading_deg + 0.5f) % 360U;
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
             task_status.gripper_closed &&
             !route_to_stash && (state == TASK_NAVIGATE) &&
             delivery_enter_command_ok(command)) {
    task_status.acknowledged_sequence = command->sequence;
    if (nav_final_push_done) {
      task_enter(TASK_OPEN_FOR_RAM, now_ms);
    }
  } else if ((command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
             ((state == TASK_OPEN_FOR_RAM) ||
              (state == TASK_RAM_VERIFY)) &&
             delivery_enter_command_ok(command)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_TASK_COMPLETE) &&
             !task_status.gripper_closed &&
             (state == TASK_RAM_VERIFY)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_SAFE_SWEEP_RETRIEVE_FAILED) &&
             task_distance_command_valid(command, VISION_CMD_RETURN_CENTER)) {
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_FACE_FIELD_CENTER, now_ms);
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_EXIT_SAFE_ZONE) &&
             task_distance_command_valid(command,
                                         VISION_CMD_RETURN_CENTER)) {
    /* Cache/ACK RETURN while mode16 finishes its local 0.30 m retreat. This
     * also releases a PAUSE latch, but never changes state or restarts the
     * local distance action. mode17 consumes the latest H/D afterwards. */
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_SEARCH) &&
             (search_phase == SEARCH_WAIT_RETURN) &&
             !task_status.gripper_closed &&
             task_distance_command_valid(command,
                                         VISION_CMD_RETURN_CENTER)) {
    /* Two 120/90-degree sweeps found no target. Let the upper computer use
     * the existing H/D return command to place the empty chassis at centre. */
    task_status.acknowledged_sequence = command->sequence;
    task_enter(TASK_FACE_FIELD_CENTER, now_ms);
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_FACE_FIELD_CENTER)) {
    task_status.acknowledged_sequence = command->sequence;
  } else if ((command->command == VISION_CMD_RETURN_CENTER) &&
             (state == TASK_SEARCH) &&
             (return_search_ack_started_ms != 0U) &&
             ((uint32_t)(now_ms - return_search_ack_started_ms) <=
              APP_RETURN_SEARCH_ACK_GRACE_MS)) {
    /* RETURN has already reached D=0 and SEARCH is running.  Keep SEARCH
     * motion unchanged, but ACK the upper computer's repeated RETURN frames
     * for a short hand-off window.  This is a compatibility aid for hosts
     * that otherwise compare only the current 8-bit ACK with their entry
     * value and can be fooled when the sequence wraps. */
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
             (task_disperse_reject_reason(command) ==
              TASK_COMMAND_REJECT_NONE)) {
    const bool first_green_bump =
        (command->flags & VISION_CMD_FIRST_GREEN_BUMP) != 0U;
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_DISPERSE, command, now_ms);
    cargo_recheck_pending = false;
    if (first_green_bump) {
      remote_action.arg_b = 2;
      separation_keep_side = 0;
      first_green_bump_done = true;
    } else if ((command->flags & VISION_CMD_SIDE_VALID) != 0U) {
      remote_action.arg_b = 0;
      separation_keep_side =
          ((command->flags & VISION_CMD_TARGET_RIGHT) != 0U) ? -1 : 1;
    } else {
      /* Marker for the observation-turn variant. It deliberately retains no
       * side until the upper computer has received a new audit. */
      remote_action.arg_b = 1;
      separation_keep_side = 0;
    }
  } else if (command->command == VISION_CMD_DISPERSE_PILE) {
    task_status.command_reject_reason =
        (uint8_t)task_disperse_reject_reason(command);
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
    /* RELEASE_BOTH now has one unambiguous meaning: physically open both
     * claws and finish as mode34. A 12-degree observation turn must use an
     * explicit DISPERSE_PILE without SIDE_VALID and finishes as mode35. */
    task_start_remote_action(action, command, now_ms);
    remote_action.arg_b = 0;
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

static bool task_boundary_guard_exempt(void)
{
  return (state == TASK_WAIT_CONFIG) || (state == TASK_START) ||
         (state == TASK_OPEN_CLAW) || (state == TASK_STOPPED) ||
         (state == TASK_BOUNDARY_RECOVER) ||
         (state == TASK_OPEN_FOR_RAM) || (state == TASK_RAM_VERIFY) ||
         (state == TASK_EXIT_SAFE_ZONE) ||
         (state == TASK_FACE_FIELD_CENTER);
}

static void task_check_boundary_guard(uint32_t now_ms)
{
  if (task_boundary_guard_exempt()) {
    return;
  }
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    return;
  }
  const float x_abs = task_abs((float)pose.x_mm);
  const float y_abs = task_abs((float)pose.y_mm);
  const float edge_distance_mm = APP_LOCATION_FIELD_HALF_MM -
      ((x_abs > y_abs) ? x_abs : y_abs);
  const float margin_mm =
      ((state == TASK_NAVIGATE) && delivery_enter_active) ?
          APP_SAFE_PUSH_EDGE_MARGIN_MM : APP_FIELD_EDGE_ABORT_MARGIN_MM;
  if (edge_distance_mm > margin_mm) {
    return;
  }

  const float heading_to_center = atan2f(-(float)pose.y_mm,
                                         -(float)pose.x_mm) *
      (180.0f / 3.14159265358979323846f);
  task_enter(TASK_BOUNDARY_RECOVER, now_ms);
  boundary_recovery_heading_deg = task_heading_360(heading_to_center);
  task_status.boundary_x_mm = pose.x_mm;
  task_status.boundary_y_mm = pose.y_mm;
  task_status.boundary_yaw_mdeg = pose.heading_mdeg;
  task_status.boundary_edge_mm = (int32_t)edge_distance_mm;
  task_status.boundary_heading_deg = (uint16_t)boundary_recovery_heading_deg;
  task_status.boundary_turn_done = false;
  mission_paused = false;
  task_status.nav_heading_locked = true;
  task_status.nav_locked_heading_deg =
      (uint16_t)(boundary_recovery_heading_deg + 0.5f) % 360U;
}

static void task_process_boundary_recover(uint32_t now_ms)
{
  Lift_SetTravelPosition();
  if (!boundary_claw_opened && Claw_Open(now_ms)) {
    boundary_claw_opened = true;
    task_status.gripper_closed = false;
  }
  if (!boundary_claw_opened) { return; }
  const LocationPose pose = Location_GetPose();
  if (!pose.valid) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  const float heading_deg = (float)pose.heading_mdeg * 0.001f;
  if (!boundary_turn_done) {
    if (!turn_to(boundary_recovery_heading_deg, heading_deg,
                 APP_BOUNDARY_TURN_TOLERANCE_DEG, now_ms)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
      return;
    }
    if (nav_ready) {
      boundary_turn_done = true;
      task_status.boundary_turn_done = true;
      nav_ready = false;
    }
    return;
  }

  const float x_abs = task_abs((float)pose.x_mm);
  const float y_abs = task_abs((float)pose.y_mm);
  const float edge_distance_mm = APP_LOCATION_FIELD_HALF_MM -
      ((x_abs > y_abs) ? x_abs : y_abs);
  if ((edge_distance_mm >= APP_FIELD_EDGE_REARM_MARGIN_MM) &&
      boundary_claw_opened) {
    Motor_Stop();
    task_status.motors_active = false;
    task_clear_audit_result();
    task_enter(TASK_SEARCH, now_ms);
    return;
  }
  const float heading_error = task_wrap_angle(
      boundary_recovery_heading_deg - heading_deg);
  Motor_Move(APP_BOUNDARY_RECOVERY_SPEED_MM_S, 0.0f,
             nav_yaw(heading_error));
  task_status.motors_active = true;
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
  task_sweep_sample(now_ms);
  task_update_match_time(now_ms);
  task_status.camera_angle = Camera_GetAngle();
  task_accept_mission(&vision.mission, now_ms);
  task_apply_complete_target(&vision);
  task_check_boundary_guard(now_ms);
  if (mission_paused &&
      (state != TASK_WAIT_CONFIG) && (state != TASK_START) &&
      (state != TASK_OPEN_CLAW)) {
    task_pause_runtime(now_ms);
    task_publish_status(now_ms);
    return;
  }
  const bool local_approach_autonomous =
      task_approach_runs_without_target();
  if (complete_flow_active && !local_approach_autonomous &&
      task_mission_valid(&vision.mission) &&
      (vision.mission.command == VISION_CMD_HOLD) &&
      (state != TASK_WAIT_CONFIG) && (state != TASK_START) &&
      (state != TASK_OPEN_CLAW) && (state != TASK_SEARCH) &&
      (state != TASK_APPROACH_RECOVER) &&
      (state != TASK_SAFE_SWEEP) &&
      (state != TASK_SAFE_SWEEP_APPROACH) &&
      (state != TASK_SAFE_SWEEP_AUDIT) &&
      (state != TASK_SAFE_SWEEP_RETRIEVE) &&
      (state != TASK_SAFE_SWEEP_RETRIEVE_AUDIT) &&
      (state != TASK_SAFE_SWEEP_RETRIEVE_FAILED) &&
      (state != TASK_BOUNDARY_RECOVER)) {
    if ((state == TASK_REMOTE_ACTION) && !remote_action.done) {
      task_pause_action(now_ms, true);
    } else {
      Motor_Stop();
      task_status.motors_active = false;
    }
    if (state == TASK_APPROACH) {
      /* Normal tracking/alignment HOLD is a stationary upper-level decision.
       * After 500 ms without a new APP frame, leave mode20 through the
       * existing mode24 recovery instead of waiting forever. Camera-to-140
       * and claw-audit phases bypass this branch because they no longer need
       * target coordinates. */
      task_status.auto_approach = true;
      task_status.found = false;
      if (approach_hold_pending &&
          ((uint32_t)(now_ms - approach_hold_started_ms) >=
           APP_APPROACH_LOSS_HOLD_MS)) {
        task_enter(TASK_APPROACH_RECOVER, now_ms);
        task_status.auto_approach = true;
        task_status.found = false;
      }
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
      if (grab_core_advance_required) {
        LocationPose pose;
        if (!task_get_location_pose(&pose, now_ms)) {
          break;
        }
        if (!grab_core_advance_started) {
          grab_core_advance_started = true;
          grab_core_advance_start_path_mm = pose.path_mm;
          grab_core_advance_heading_deg =
              (float)pose.heading_mdeg * 0.001f;
        }
        const uint32_t travelled_mm =
            pose.path_mm - grab_core_advance_start_path_mm;
        if (travelled_mm < APP_CORE_GRAB_ADVANCE_DISTANCE_MM) {
          const float heading_deg = (float)pose.heading_mdeg * 0.001f;
          const float heading_error = task_wrap_angle(
              grab_core_advance_heading_deg - heading_deg);
          Motor_Move(APP_GRAB_WATCH_CRAWL_SPEED_MM_S, 0.0f,
                     nav_yaw(heading_error));
          task_status.motors_active = true;
          break;
        }
        Motor_Stop();
        task_status.motors_active = false;
        grab_core_advance_required = false;
      }
      if (Claw_Touch(now_ms)) {
        task_status.gripper_closed = true;
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        camera_angle = (float)APP_GRAB_VIEW_ANGLE;
        task_clear_audit_result();
        task_enter(TASK_POST_GRAB_AUDIT, now_ms);
      }
      break;

    case TASK_POST_GRAB_AUDIT:
      task_process_post_grab_audit(now_ms);
      break;

    case TASK_WAIT_NAVIGATION:
      Motor_Stop();
      break;

    case TASK_NAVIGATE:
      task_process_navigation(&vision, now_ms);
      break;

    case TASK_ALIGN_SAFE_ZONE:
      task_process_safe_align(now_ms);
      break;

    case TASK_SAFE_SWEEP:
      task_process_safe_sweep(now_ms);
      break;

    case TASK_SAFE_SWEEP_APPROACH:
    case TASK_SAFE_SWEEP_RETRIEVE:
      task_process_safe_sweep_approach(&vision, now_ms);
      break;

    case TASK_SAFE_SWEEP_AUDIT:
    case TASK_SAFE_SWEEP_RETRIEVE_AUDIT:
      task_process_safe_sweep_audit(now_ms);
      break;

    case TASK_SAFE_SWEEP_DONE:
      Motor_Stop();
      task_status.motors_active = false;
      task_status.gripper_closed = true;
      break;

    case TASK_SAFE_SWEEP_RETRIEVE_FAILED:
      Motor_Stop();
      task_status.motors_active = false;
      task_status.gripper_closed = false;
      break;

    case TASK_BOUNDARY_RECOVER:
      task_process_boundary_recover(now_ms);
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
        const LocationPose pose = Location_GetPose();
        if (!delivery_exit_path_valid) {
          delivery_exit_start_path_mm = pose.path_mm;
          delivery_exit_path_valid = true;
        }
        const uint32_t target_mm = (uint32_t)(
            APP_DELIVERY_EXIT_DISTANCE_M * 1000.0f + 0.5f);
        const uint32_t travelled_mm =
            pose.path_mm - delivery_exit_start_path_mm;
        if (travelled_mm >= target_mm) {
          task_enter(TASK_FACE_FIELD_CENTER, now_ms);
          break;
        }
        const float remaining_m =
            (float)(target_mm - travelled_mm) * 0.001f;
        const MotorDistanceStatus result = Motor_MoveDistance(
            -remaining_m,
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
      if ((task_status.fault == TASK_FAULT_REMOTE_STOP) &&
          vision.config_ready) {
        /* Vision_RearmConfig() made this readiness possible only after a new
         * confirmed operator configuration. Rebuild task-local state and
         * repeat the normal safe claw retraction before consuming it. */
        task_initialize(now_ms);
      }
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
  snapshot.sweep_phase = (uint8_t)visual_sweep_phase;
  snapshot.sweep_reverse_phase = (uint8_t)sweep_reverse.phase;
  snapshot.sweep_trace_remaining = sweep_trace_count;
  snapshot.sweep_trace_initial = sweep_reverse.initial_count;
  snapshot.audit_recheck_pending = cargo_recheck_pending;
  snapshot.action_command = remote_action.command;
  snapshot.action_phase = remote_action.phase;
  snapshot.action_done = remote_action.done;
  snapshot.action_disambiguate =
      (remote_action.type == REMOTE_ACTION_DISPERSE) &&
      (remote_action.arg_b == 1);
  snapshot.action_green_bump =
      (remote_action.type == REMOTE_ACTION_DISPERSE) &&
      (remote_action.arg_b == 2);
  if (primask == 0U) {
    __enable_irq();
  }
  return snapshot;
}
