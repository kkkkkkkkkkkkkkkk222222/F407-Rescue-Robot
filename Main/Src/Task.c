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
  SEARCH_RETURN_ALIGN,
  SEARCH_RETURN_CENTER
} SearchPhase;

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

static volatile TaskStatus task_status;
static Pid_t steering_pid;
static Pid_t camera_pid;
static TaskState state;
static TurnTracker turn_tracker;
static SearchPhase search_phase;
static uint32_t state_started_ms;
static uint32_t step_started_ms;
static uint32_t status_sent_ms;
static uint32_t pose_invalid_started_ms;
static uint32_t approach_last_target_ms;
static uint32_t approach_prestop_started_ms;
static uint32_t nav_payload_change_ms;
static uint32_t nav_final_push_started_ms;
static uint32_t nav_final_push_paused_ms;
static float camera_angle;
static float steering_mm_s;
static float steering_target_mm_s;
static float approach_speed_mm_s;
static bool approach_close_hold;
static bool approach_prestop_active;
static float filtered_target_x;
static float filtered_target_y;
static float remote_speed_mm_s;
static float remote_yaw_mm_s;
static float nav_locked_heading_deg;
static uint32_t tracking_tick_ms;
static uint32_t approach_report_generation;
static uint32_t search_counted_report_generation;
static int16_t nav_last_distance_mm;
static uint8_t tracking_sequence;
static uint8_t mission_sequence;
static uint32_t start_reverse_path_mm;
static float start_target_heading_deg;
static float reposition_heading_deg;
static float reposition_distance_m;
static uint32_t scan_entry_report_generation;
static uint32_t nav_final_push_start_path_mm;
static uint8_t search_phase_report_count;
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
static bool nav_payload_valid;
static bool nav_payload_stale;
static bool nav_forward_active;
static bool nav_heading_locked;
static bool nav_final_push_active;
static bool nav_final_push_done;
static bool nav_final_push_paused;
static bool first_delivery_done;
static bool complete_flow_active;
static bool audit_received;
static bool audit_valid;
static bool audit_initial_stash;
static bool audit_destination_injury;
static bool cargo_recheck_pending;
static bool route_to_initial_stash;
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

static void task_enter(TaskState next, uint32_t now_ms);
static bool distance_failed(MotorDistanceStatus result);
static bool delivery_enter_command_ok(const VisionMissionCommand *command,
                                      uint32_t now_ms);

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
  steering_target_mm_s = 0.0f;
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

static uint8_t task_protocol_mode(void)
{
  if (complete_flow_active) {
    if (state == TASK_APPROACH) {
      return 20U;
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

  if ((total == 0U) || (total > 3U) ||
      ((uint8_t)(left_count + right_count) != total) ||
      dangerous || unknown || injury_mixed) {
    return false;
  }
  if (audit_initial_stash) {
    return true;
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
  return !injury &&
         ((left == VISION_CARGO_NONE) ||
          (left == VISION_CARGO_GREEN) ||
          (left == VISION_CARGO_CORE) ||
          (left == VISION_CARGO_MIXED_MATERIAL)) &&
         ((right == VISION_CARGO_NONE) ||
          (right == VISION_CARGO_GREEN) ||
          (right == VISION_CARGO_CORE) ||
          (right == VISION_CARGO_MIXED_MATERIAL));
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

static void task_apply_complete_target(VisionData *vision, uint32_t now_ms)
{
  if (!complete_flow_active) {
    return;
  }
  const VisionMissionCommand *command = &vision->mission;
  const bool fresh = Vision_MissionIsFresh(
      command, now_ms, APP_MISSION_COMMAND_TIMEOUT_MS);
  if (fresh && (command->command == VISION_CMD_APPROACH_TARGET)) {
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
    search_counted_report_generation = vision.report_generation;
    search_phase_report_count = 0U;
    scan_report_gate_open = false;
    reposition_distance_m = 0.0f;
    audit_received = false;
    audit_valid = false;
    audit_initial_stash = false;
    audit_destination_injury = false;
    cargo_recheck_pending = false;
    route_to_initial_stash = false;
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
    approach_close_hold = false;
    approach_prestop_active = false;
    approach_prestop_started_ms = now_ms;
    approach_last_target_ms = now_ms;
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
  approach_prestop_started_ms = now_ms;
  nav_payload_change_ms = now_ms;
  nav_final_push_started_ms = now_ms;
  nav_final_push_paused_ms = now_ms;
  camera_angle = (float)Camera_GetAngle();
  steering_mm_s = 0.0f;
  steering_target_mm_s = 0.0f;
  approach_speed_mm_s = APP_APPROACH_SPEED_MM_S;
  approach_close_hold = false;
  approach_prestop_active = false;
  remote_speed_mm_s = 0.0f;
  remote_yaw_mm_s = 0.0f;
  nav_locked_heading_deg = 0.0f;
  tracking_sequence = 0U;
  tracking_tick_ms = 0U;
  approach_report_generation = 0U;
  search_counted_report_generation = 0U;
  nav_last_distance_mm = 0;
  mission_sequence = 0U;
  start_reverse_path_mm = 0U;
  start_target_heading_deg = 0.0f;
  reposition_heading_deg = 0.0f;
  reposition_distance_m = 0.0f;
  scan_entry_report_generation = 0U;
  nav_final_push_start_path_mm = 0U;
  search_phase_report_count = 0U;
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
  nav_payload_valid = false;
  nav_payload_stale = false;
  nav_forward_active = false;
  nav_heading_locked = false;
  nav_final_push_active = false;
  nav_final_push_done = false;
  nav_final_push_paused = false;
  first_delivery_done = false;
  complete_flow_active = false;
  audit_received = false;
  audit_valid = false;
  audit_initial_stash = false;
  audit_destination_injury = false;
  cargo_recheck_pending = false;
  route_to_initial_stash = false;
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
    return steering_target_mm_s;
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
  task_count_search_report(vision, now_ms);
  task_status.found = task_scan_target_found(vision, now_ms);
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
        task_reset_search_report_count(vision);
      }
      return;

    case SEARCH_SWEEP_90:
      if (task_full_turn_reached()) {
        Motor_Stop();
        task_status.motors_active = false;
        if (search_phase_report_count <
            APP_SEARCH_MIN_REPORTS_PER_SWEEP) {
          task_reset_turn_tracker();
          task_reset_search_report_count(vision);
          return;
        }
        LocationPose pose;
        if (!task_get_location_pose(&pose, now_ms)) {
          return;
        }
        const float center_distance_mm = sqrtf(
            (float)pose.x_mm * (float)pose.x_mm +
            (float)pose.y_mm * (float)pose.y_mm);
        if (center_distance_mm <= APP_SEARCH_CENTER_TOLERANCE_MM) {
          task_reset_turn_tracker();
          task_reset_search_report_count(vision);
          return;
        }
        reposition_heading_deg = atan2f(-(float)pose.y_mm,
                                        -(float)pose.x_mm) * 57.2957795f;
        reposition_distance_m = center_distance_mm * 0.001f;
        search_phase = SEARCH_RETURN_ALIGN;
        step_started_ms = now_ms;
        return;
      }
      Motor_Move(0.0f, 0.0f, APP_SEARCH_ROTATE_SPEED_MM_S);
      task_status.motors_active = true;
      return;

    case SEARCH_RETURN_ALIGN: {
      const MotorTurnStatus result =
          task_turn_to_heading(reposition_heading_deg, now_ms);
      task_status.motors_active = result == MOTOR_TURN_RUNNING;
      if (result == MOTOR_TURN_DONE) {
        Motor_Stop();
        search_phase = SEARCH_RETURN_CENTER;
        step_started_ms = now_ms;
      } else if ((result == MOTOR_TURN_FAULT) ||
                 (result == MOTOR_TURN_INVALID)) {
        task_stop(TASK_FAULT_MOTOR, now_ms);
      }
      return;
    }

    case SEARCH_RETURN_CENTER: {
      if ((uint32_t)(now_ms - step_started_ms) <
          APP_NAV_TURN_SETTLE_MS) {
        return;
      }
      const MotorDistanceStatus result = Motor_MoveDistance(
          reposition_distance_m,
          APP_SEARCH_RETURN_CENTER_SPEED_MM_S);
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

static float task_steering_freshness_scale(uint32_t age_ms)
{
  if (age_ms <= APP_STEERING_FRAME_HOLD_MS) {
    return 1.0f;
  }
  if (age_ms >= APP_STEERING_FRAME_STOP_MS) {
    return 0.0f;
  }
  return (float)(APP_STEERING_FRAME_STOP_MS - age_ms) /
         (float)(APP_STEERING_FRAME_STOP_MS -
                 APP_STEERING_FRAME_HOLD_MS);
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

static float task_approach_steering(uint32_t report_age_ms)
{
  const float desired_mm_s = steering_target_mm_s *
      task_steering_freshness_scale(report_age_ms);
  steering_mm_s = task_step_toward(
      steering_mm_s, desired_mm_s,
      APP_STEERING_RATE_MM_S2 * APP_TASK_PERIOD_MS * 0.001f);
  return steering_mm_s;
}

static bool task_approach_close_enough(const VisionData *vision)
{
  if (!tracking_filter_valid ||
      (task_abs(filtered_target_x - (float)APP_VISION_TARGET_X) >
       APP_GRAB_HOLD_X_ERROR_PX)) {
    return false;
  }
  return vision->near ||
      (vision->distance_valid &&
       (vision->distance_mm <= APP_GRAB_HOLD_DISTANCE_MM));
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
      if (!approach_close_hold) {
        (void)task_track_target(vision);
        if (task_approach_close_enough(vision)) {
          approach_close_hold = true;
          step_started_ms = now_ms;
        }
      }
      if (Camera_GetAngle() >= APP_GRAB_VIEW_ANGLE) {
        Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
        camera_angle = (float)APP_GRAB_VIEW_ANGLE;
        task_enter(TASK_GRAB_OBSERVE, now_ms);
        return;
      }
      const uint8_t camera_command = Camera_GetAngle();
      if ((camera_command >= APP_GRAB_PRESTOP_CAMERA_ANGLE) &&
          (camera_command < APP_GRAB_VIEW_ANGLE)) {
        if (!approach_prestop_active) {
          approach_prestop_active = true;
          approach_prestop_started_ms = now_ms;
        }
        if (!approach_close_hold && tracking_filter_valid &&
            (task_abs(filtered_target_x - (float)APP_VISION_TARGET_X) <=
             APP_GRAB_HOLD_X_ERROR_PX) &&
            ((uint32_t)(now_ms - approach_prestop_started_ms) >=
             APP_GRAB_PRESTOP_FALLBACK_MS)) {
          approach_close_hold = true;
          step_started_ms = now_ms;
        }
      } else {
        approach_prestop_active = false;
      }
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

  if (approach_close_hold) {
    Motor_Stop();
    task_status.motors_active = false;
    if (task_scan_camera_to(APP_GRAB_VIEW_ANGLE, now_ms)) {
      task_enter(TASK_GRAB_OBSERVE, now_ms);
    }
    return;
  }

  const float freshness_scale =
      task_approach_freshness_scale(report_age_ms);
  const float turn_mm_s = task_approach_steering(report_age_ms);
  task_status.found = freshness_scale > 0.0f;
  if (freshness_scale <= 0.0f) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }

  float forward_mm_s = task_approach_aligned_speed(
      approach_speed_mm_s) * freshness_scale;
  if ((Camera_GetAngle() >= APP_GRAB_PRESTOP_CAMERA_ANGLE) &&
      (forward_mm_s > APP_GRAB_PRESTOP_SPEED_MM_S)) {
    forward_mm_s = APP_GRAB_PRESTOP_SPEED_MM_S;
  }
  Motor_Move(forward_mm_s, 0.0f, turn_mm_s);
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
  if (complete_flow_active) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
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
  if (complete_flow_active) {
    task_enter(TASK_GRAB_OBSERVE, now_ms);
    return;
  }
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
  float speed_mm_s = end_speed_mm_s +
      (cruise_speed_mm_s - end_speed_mm_s) * ratio;
  if ((cruise_speed_mm_s > APP_NAV_FINAL_APPROACH_SPEED_MM_S) &&
      (remaining_mm <= (int16_t)APP_NAV_FINAL_APPROACH_DISTANCE_MM) &&
      (speed_mm_s > APP_NAV_FINAL_APPROACH_SPEED_MM_S)) {
    speed_mm_s = APP_NAV_FINAL_APPROACH_SPEED_MM_S;
  }
  return speed_mm_s;
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

static RemoteRouteStatus task_run_final_push(uint32_t now_ms)
{
  LocationPose pose;
  if (!task_get_location_pose(&pose, now_ms)) {
    task_pause_final_push(now_ms);
    nav_forward_active = false;
    task_reset_remote_targets();
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
    task_reset_remote_targets();
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
    task_reset_remote_targets();
    return REMOTE_ROUTE_REACHED;
  }

  const float yaw_mm_s = task_remote_heading_correction(heading_error_deg);
  Motor_Move(APP_NAV_FINAL_PUSH_SPEED_MM_S, 0.0f, yaw_mm_s);
  task_status.motors_active = true;
  return REMOTE_ROUTE_RUNNING;
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
    task_pause_final_push(now_ms);
    nav_forward_active = false;
    task_reset_remote_targets();
    return REMOTE_ROUTE_WAITING;
  }
  (void)task_nav_payload_changed(command, now_ms);

  if (nav_final_push_done) {
    Motor_Stop();
    task_status.motors_active = false;
    nav_forward_active = false;
    distance_command_done = true;
    task_status.nav_done = true;
    task_reset_remote_targets();
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
    task_reset_remote_targets();
    return REMOTE_ROUTE_REACHED;
  }
  if (route_to_initial_stash &&
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      (command->target_x_mm <=
       (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM)) {
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

  const float current_heading_deg = (float)pose.heading_mdeg * 0.001f;
  const float command_heading_deg = (float)command->heading_cdeg * 0.01f;
  const bool reverse_route =
      expected_command == VISION_CMD_RETURN_CENTER;
  const float route_body_heading_deg = reverse_route ?
      task_wrap_angle(command_heading_deg + 180.0f) : command_heading_deg;
  const bool lock_allowed =
      (expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      !route_to_initial_stash;

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
    task_reset_remote_targets();
  }

  const float desired_heading_deg = nav_heading_locked ?
      nav_locked_heading_deg : route_body_heading_deg;
  const float heading_error_deg = task_wrap_angle(
      desired_heading_deg - current_heading_deg);
  if (nav_ready &&
      ((nav_heading_locked &&
        (task_abs(heading_error_deg) >
         APP_NAV_FINAL_TURN_TOLERANCE_DEG)) ||
       (!nav_heading_locked && !reverse_route &&
        (task_abs(heading_error_deg) >= APP_NAV_REALIGN_DEG)))) {
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
    if (reverse_route) {
      /* RETURN heading is the field travel direction. Keep the body facing
       * opposite that direction and start backing up without a stationary
       * 180-degree turn. Small yaw corrections remain active while moving. */
      nav_ready = true;
      step_started_ms = now_ms;
      return REMOTE_ROUTE_WAITING;
    }
    const float turn_tolerance_deg = nav_heading_locked ?
        APP_NAV_FINAL_TURN_TOLERANCE_DEG :
        APP_NAV_HEADING_TOLERANCE_DEG;
    if (!turn_to(desired_heading_deg, current_heading_deg,
                 turn_tolerance_deg, now_ms)) {
      return REMOTE_ROUTE_MOTOR_FAULT;
    }
    return REMOTE_ROUTE_WAITING;
  }
  if ((uint32_t)(now_ms - step_started_ms) < APP_NAV_TURN_SETTLE_MS) {
    return REMOTE_ROUTE_WAITING;
  }

  if ((expected_command == VISION_CMD_NAVIGATE_WAYPOINT) &&
      !route_to_initial_stash &&
      ((command->target_x_mm <=
        (int16_t)APP_NAV_REMOTE_STOP_DISTANCE_MM) ||
       nav_final_push_active)) {
    return task_run_final_push(now_ms);
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

  const float route_speed_mm_s = task_remote_route_speed(
      command->target_x_mm, cruise_speed_mm_s);
  const float target_speed_mm_s = reverse_route ?
      -route_speed_mm_s : route_speed_mm_s;
  float target_yaw_mm_s = task_remote_heading_correction(heading_error_deg);
  if (reverse_route) {
    if (target_yaw_mm_s > APP_RETURN_CENTER_HEADING_MAX_MM_S) {
      target_yaw_mm_s = APP_RETURN_CENTER_HEADING_MAX_MM_S;
    } else if (target_yaw_mm_s < -APP_RETURN_CENTER_HEADING_MAX_MM_S) {
      target_yaw_mm_s = -APP_RETURN_CENTER_HEADING_MAX_MM_S;
    }
  }
  const float speed_rate_mm_s2 =
      (task_abs(target_speed_mm_s) >= task_abs(remote_speed_mm_s)) ?
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
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return;
  }

  if (!command_fresh) {
    /* Never move on an expired externally closed-loop target. */
    task_pause_final_push(now_ms);
    Motor_Stop();
    task_status.motors_active = false;
    nav_ready = false;
    nav_forward_active = false;
    task_reset_remote_targets();
    return;
  }
  if (vision->mission.command == VISION_CMD_ENTER_SAFE_ZONE) {
    if (!delivery_enter_command_ok(&vision->mission, now_ms)) {
      task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
      return;
    }
    const RemoteRouteStatus result = task_run_final_push(now_ms);
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
  const uint8_t required_flags = VISION_CMD_DRIVE_STRAIGHT |
                                 VISION_CMD_USE_FINAL_HEADING;
  return Vision_MissionIsFresh(command, now_ms,
                               APP_MISSION_COMMAND_TIMEOUT_MS) &&
         (command->command == VISION_CMD_ENTER_SAFE_ZONE) &&
         ((command->flags & required_flags) == required_flags) &&
         task_side_flag_valid(command->flags);
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
    first_delivery_done = true;
    task_enter(TASK_FACE_FIELD_CENTER, now_ms);
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
    task_enter(TASK_SEARCH, now_ms);
  } else if (result == REMOTE_ROUTE_COMMAND_INVALID) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
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
}

static void task_remote_action_finish(void)
{
  Motor_Stop();
  task_status.motors_active = false;
  remote_action.done = true;
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

static void task_process_remote_action(const VisionMissionCommand *command,
                                       uint32_t now_ms)
{
  if (remote_action.done) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  if ((uint32_t)(now_ms - remote_action.started_ms) >=
      APP_REMOTE_ACTION_TIMEOUT_MS) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
    return;
  }
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS) ||
      (command->command != remote_action.command)) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }

  switch (remote_action.type) {
    case REMOTE_ACTION_RELEASE_LEFT:
      if (Claw_OpenLeft(now_ms)) {
        /* Left is fully open and right remains at its normal 100-degree touch
         * position.  The complete two-claw grip is not restored yet. */
        task_status.gripper_closed = false;
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_RELEASE_RIGHT:
      if (Claw_OpenRight(now_ms)) {
        /* Right is fully open and left remains at its normal 80-degree touch
         * position.  Report closed only after both claws are secured again. */
        task_status.gripper_closed = false;
        cargo_recheck_pending = true;
        task_clear_audit_result();
        task_remote_action_finish();
      }
      return;

    case REMOTE_ACTION_RELEASE_BOTH:
      if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        cargo_recheck_pending = false;
        task_clear_audit_result();
        route_to_initial_stash = false;
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
        if (Claw_Open(now_ms)) {
          task_remote_action_advance(now_ms);
        }
      } else if (remote_action.phase == 1U) {
        (void)task_remote_distance(APP_REMOTE_DISPERSE_FORWARD_M,
                                   APP_REMOTE_DISPERSE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 2U) {
        (void)task_remote_distance(-APP_REMOTE_DISPERSE_BACK_M,
                                   APP_REMOTE_DISPERSE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 3U) {
        (void)task_remote_turn(APP_REMOTE_DISPERSE_TURN_DEG, now_ms);
      } else if (remote_action.phase == 4U) {
        (void)task_remote_distance(APP_REMOTE_DISPERSE_FORWARD_M,
                                   APP_REMOTE_DISPERSE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 5U) {
        (void)task_remote_distance(-APP_REMOTE_DISPERSE_FORWARD_M,
                                   APP_REMOTE_DISPERSE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 6U) {
        (void)task_remote_turn(-2.0f * APP_REMOTE_DISPERSE_TURN_DEG, now_ms);
      } else if (remote_action.phase == 7U) {
        (void)task_remote_distance(APP_REMOTE_DISPERSE_FORWARD_M,
                                   APP_REMOTE_DISPERSE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 8U) {
        (void)task_remote_distance(-APP_REMOTE_DISPERSE_FORWARD_M,
                                   APP_REMOTE_DISPERSE_SPEED_MM_S, now_ms);
      } else if (remote_action.phase == 9U) {
        if (task_remote_turn(APP_REMOTE_DISPERSE_TURN_DEG, now_ms)) {
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
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS) ||
      (mission_sequence_valid &&
       (command->sequence == mission_sequence))) {
    return;
  }
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
  } else if (command->command == VISION_CMD_HOLD) {
    task_status.acknowledged_sequence = command->sequence;
    if ((state == TASK_FACE_FIELD_CENTER) && nav_payload_valid &&
        (nav_last_distance_mm <= APP_COMPLETE_RETURN_HOLD_ACCEPT_MM)) {
      /* The complete-flow branch currently changes to SEARCH/HOLD at its
       * configured 0.60 m centre radius instead of transmitting RETURN D=0.
       * Accept that hand-off only when the last genuine RETURN remainder was
       * already within a narrow 650 mm gate; never treat a distant HOLD as
       * arrival. */
      task_enter(TASK_SEARCH, now_ms);
    } else if ((state == TASK_REMOTE_ACTION) && remote_action.done &&
               (remote_action.type == REMOTE_ACTION_DISPERSE)) {
      task_enter(TASK_SEARCH, now_ms);
    } else if (state != TASK_SEARCH) {
      Motor_Stop();
      task_status.motors_active = false;
      nav_ready = false;
      nav_forward_active = false;
      if ((state == TASK_REMOTE_ACTION) && !remote_action.done &&
          !remote_action.paused) {
        remote_action.paused = true;
        remote_action.paused_ms = now_ms;
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
             !cargo_recheck_pending &&
             ((state == TASK_SEARCH) ||
              (state == TASK_APPROACH_RECOVER) ||
              ((state == TASK_REMOTE_ACTION) && remote_action.done))) {
    task_status.acknowledged_sequence = command->sequence;
    locked_cargo_counts = 1U;
    task_enter(TASK_APPROACH, now_ms);
  } else if ((command->command == VISION_CMD_APPROACH_TARGET) &&
             (state == TASK_APPROACH)) {
    task_status.acknowledged_sequence = command->sequence;
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
    route_to_initial_stash = audit_initial_stash;
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
               !route_to_initial_stash && (state == TASK_NAVIGATE)) {
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
             (state != TASK_STOPPED) &&
             ((state != TASK_REMOTE_ACTION) || remote_action.done)) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_YIELD, command, now_ms);
  } else if ((command->command == VISION_CMD_ESCAPE_MANEUVER) &&
             (state != TASK_STOPPED) &&
             ((state != TASK_REMOTE_ACTION) || remote_action.done)) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_ESCAPE, command, now_ms);
  } else if ((command->command == VISION_CMD_CHANGE_LANE) &&
             (state != TASK_STOPPED) &&
             ((state != TASK_REMOTE_ACTION) || remote_action.done)) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_LANE, command, now_ms);
  } else if ((command->command == VISION_CMD_DISPERSE_PILE) &&
             !cargo_recheck_pending &&
             !task_status.gripper_closed &&
             ((state == TASK_SEARCH) ||
              ((state == TASK_REMOTE_ACTION) && remote_action.done))) {
    task_status.acknowledged_sequence = command->sequence;
    task_start_remote_action(REMOTE_ACTION_DISPERSE, command, now_ms);
  } else if (((command->command == VISION_CMD_RELEASE_LEFT) ||
              (command->command == VISION_CMD_RELEASE_RIGHT) ||
              (command->command == VISION_CMD_RELEASE_BOTH)) &&
             audit_received && (!audit_valid || audit_initial_stash) &&
             !((state == TASK_REMOTE_ACTION) && !remote_action.done)) {
    task_status.acknowledged_sequence = command->sequence;
    /* On the first invalid audit, honour the upper computer's selected side:
     * that side opens while the opposite side only moves to its normal touch
     * angle.  If the retained cargo fails the follow-up audit, release both;
     * never close the previously released side around the same pile again. */
    const RemoteAction action = cargo_recheck_pending ?
        REMOTE_ACTION_RELEASE_BOTH :
        ((command->command == VISION_CMD_RELEASE_LEFT) ?
             REMOTE_ACTION_RELEASE_LEFT :
         (command->command == VISION_CMD_RELEASE_RIGHT) ?
             REMOTE_ACTION_RELEASE_RIGHT : REMOTE_ACTION_RELEASE_BOTH);
    task_start_remote_action(action, command, now_ms);
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
  if (task_motor_fault()) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
  task_accept_mission(&vision.mission, now_ms);
  task_apply_complete_target(&vision, now_ms);
  if (complete_flow_active &&
      Vision_MissionIsFresh(&vision.mission, now_ms,
                            APP_MISSION_COMMAND_TIMEOUT_MS) &&
      (vision.mission.command == VISION_CMD_HOLD) &&
      (state != TASK_WAIT_CONFIG) && (state != TASK_START) &&
      (state != TASK_OPEN_CLAW) && (state != TASK_SEARCH)) {
    Motor_Stop();
    task_status.motors_active = false;
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
      /* Reserved status value: return now starts directly from mode 17. */
      task_enter(TASK_FACE_FIELD_CENTER, now_ms);
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
  if (primask == 0U) {
    __enable_irq();
  }
  return snapshot;
}
