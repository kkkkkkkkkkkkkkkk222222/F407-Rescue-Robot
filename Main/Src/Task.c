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

static volatile TaskStatus task_status;
static Pid_t steering_pid;
static Pid_t camera_pid;
static TaskState state;
static StartStep start_step;
static TurnTracker turn_tracker;
static uint32_t state_started_ms;
static uint32_t step_started_ms;
static uint32_t match_started_ms;
static uint32_t status_sent_ms;
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
static bool navigation_turned;
static bool start_reverse_path_valid;
static bool search_report_gate_open;

static void task_enter(TaskState next, uint32_t now_ms);
static bool task_distance_failed(MotorDistanceStatus result);

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
    start_reverse_path_valid = false;
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
    task_reset_tracking();
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
    navigation_turned = false;
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
  navigation_turned = false;
  start_reverse_path_valid = false;
  search_report_gate_open = false;
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
    if (!start_reverse_path_valid) {
      start_reverse_path_mm = pose.path_mm;
      start_reverse_path_valid = true;
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
      Go_distance(APP_PILE_APPROACH_DISTANCE_M,
                  APP_PILE_APPROACH_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_SCATTER_POSITIVE, now_ms);
  } else if (task_distance_failed(result)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
}

static void task_process_scatter_positive(uint32_t now_ms)
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
    task_enter(TASK_SCATTER_PAUSE, now_ms);
    return;
  }
  Motor_Move(0.0f, 0.0f, APP_SCATTER_ROTATE_SPEED_MM_S);
  task_status.motors_active = true;
}

static void task_process_scatter_pause(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - state_started_ms) >=
      APP_SCATTER_BRAKE_WAIT_MS) {
    task_enter(TASK_SCATTER_NEGATIVE, now_ms);
  }
}

static void task_process_scatter_negative(uint32_t now_ms)
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
    task_enter(TASK_SCATTER_EXIT, now_ms);
    return;
  }
  Motor_Move(0.0f, 0.0f, -APP_SCATTER_ROTATE_SPEED_MM_S);
  task_status.motors_active = true;
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
      Go_distance(-APP_SCATTER_EXIT_DISTANCE_M,
                  APP_SCATTER_EXIT_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_SEARCH, now_ms);
  } else if (task_distance_failed(result)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  }
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
        Go_distance(APP_SEARCH_ADVANCE_DISTANCE_M,
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
    search_advancing = true;
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
    }
    return;
  }

  const float turn = task_track_target(vision);
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

static bool task_pose_ready(const VisionFusedPose *pose, uint32_t now_ms)
{
  return Vision_FusedPoseIsFresh(pose, now_ms,
                                 APP_FUSED_POSE_TIMEOUT_MS) &&
         ((pose->status & VISION_POSE_T265_GOOD) != 0U) &&
         ((pose->status & VISION_POSE_T265_UPDATE_REJECTED) == 0U);
}

static bool task_prepare_navigation(const VisionMissionCommand *command,
                                    const VisionFusedPose *pose,
                                    bool use_target_bearing)
{
  float desired_deg;
  if (use_target_bearing) {
    const float dx = (float)command->target_x_mm - (float)pose->x_mm;
    const float dy = (float)command->target_y_mm - (float)pose->y_mm;
    desired_deg = atan2f(dy, dx) * 57.2957795f;
    if (desired_deg < 0.0f) {
      desired_deg += 360.0f;
    }
  } else {
    desired_deg = (float)command->heading_cdeg * 0.01f;
  }

  const float current_deg = (float)pose->heading_cdeg * 0.01f;
  const float error_deg = task_wrap_angle(desired_deg - current_deg);
  if (task_abs(error_deg) <= APP_NAV_HEADING_TOLERANCE_DEG) {
    Motor_Stop();
    navigation_turned = true;
    return true;
  }

  const MotorTurnStatus result = Motor_TurnAngle(error_deg);
  task_status.motors_active = result == MOTOR_TURN_RUNNING;
  if (result == MOTOR_TURN_DONE) {
    Motor_Stop();
    task_status.motors_active = false;
    navigation_turned = true;
    return true;
  }
  if ((result == MOTOR_TURN_FAULT) || (result == MOTOR_TURN_INVALID)) {
    return false;
  }
  return true;
}

static float task_target_distance(const VisionMissionCommand *command,
                                  const VisionFusedPose *pose)
{
  const float dx = (float)command->target_x_mm - (float)pose->x_mm;
  const float dy = (float)command->target_y_mm - (float)pose->y_mm;
  return sqrtf(dx * dx + dy * dy);
}

static void task_process_navigation(const VisionData *vision,
                                    uint32_t now_ms)
{
  const uint8_t required_command =
      (state == TASK_NAVIGATE) ? VISION_CMD_NAVIGATE_WAYPOINT :
                                 VISION_CMD_ALIGN_SAFE_ZONE;
  if (!Vision_MissionIsFresh(&vision->mission, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS) ||
      (vision->mission.command != required_command)) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
    return;
  }
  if (!task_pose_ready(&vision->fused_pose, now_ms)) {
    task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
    return;
  }

  if (state == TASK_ALIGN_SAFE_ZONE) {
    if (!task_prepare_navigation(&vision->mission, &vision->fused_pose,
                                 false)) {
      task_stop(TASK_FAULT_MOTOR, now_ms);
    }
    return;
  }

  if (!navigation_turned &&
      !task_prepare_navigation(&vision->mission, &vision->fused_pose,
                               state == TASK_NAVIGATE)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
    return;
  }
  if (!navigation_turned) {
    return;
  }

  const float distance = task_target_distance(&vision->mission,
                                              &vision->fused_pose);
  if (distance <= APP_NAV_TARGET_HOLD_MM) {
    Motor_Stop();
    task_status.motors_active = false;
    return;
  }
  float speed = APP_NAV_FAST_SPEED_MM_S;
  if (distance <= APP_NAV_SLOW_DISTANCE_MM) {
    speed = APP_NAV_SLOW_SPEED_MM_S;
  }
  /* Motor_MoveAngle() returns false while its non-blocking acceleration ramp
   * is still converging; faults are checked centrally on every task tick. */
  (void)Motor_MoveAngle(speed, 0.0f);
  task_status.motors_active = true;
}

static bool task_distance_failed(MotorDistanceStatus result)
{
  return (result == MOTOR_DISTANCE_FAULT) ||
         (result == MOTOR_DISTANCE_INVALID);
}

static void task_process_ram_back(uint32_t now_ms)
{
  const MotorDistanceStatus result =
      Go_distance(-APP_RAM_BACK_DISTANCE_M, APP_RAM_BACK_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_RAM_FORWARD, now_ms);
  } else if (task_distance_failed(result)) {
    task_stop(TASK_FAULT_RAM, now_ms);
  }
}

static void task_process_ram_forward(uint32_t now_ms)
{
  const MotorDistanceStatus result =
      Go_distance(APP_RAM_FORWARD_DISTANCE_M, APP_RAM_FORWARD_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_RAM_VERIFY, now_ms);
  } else if (task_distance_failed(result)) {
    task_stop(TASK_FAULT_RAM, now_ms);
  }
}

static void task_process_ram_verify(const VisionMissionCommand *command,
                                    uint32_t now_ms)
{
  Motor_Stop();
  task_status.motors_active = false;
  if ((uint32_t)(now_ms - state_started_ms) < APP_RAM_VERIFY_WAIT_MS) {
    return;
  }
  if (!Vision_MissionIsFresh(command, now_ms,
                             APP_MISSION_COMMAND_TIMEOUT_MS)) {
    task_stop(TASK_FAULT_COMMAND_TIMEOUT, now_ms);
  } else if (command->command == VISION_CMD_ENTER_SAFE_ZONE) {
    task_enter(TASK_RAM_BACK, now_ms);
  }
}

static void task_process_safe_exit(uint32_t now_ms)
{
  const MotorDistanceStatus result =
      Go_distance(-APP_SAFE_EXIT_DISTANCE_M, APP_SAFE_EXIT_SPEED_MM_S);
  task_status.motors_active = result == MOTOR_DISTANCE_RUNNING;
  if (result == MOTOR_DISTANCE_DONE) {
    task_enter(TASK_FACE_FIELD_CENTER, now_ms);
  } else if (task_distance_failed(result)) {
    task_stop(TASK_FAULT_RAM, now_ms);
  }
}

static void task_process_face_center(const VisionFusedPose *pose,
                                     uint32_t now_ms)
{
  if (!task_pose_ready(pose, now_ms)) {
    task_stop(TASK_FAULT_POSE_TIMEOUT, now_ms);
    return;
  }
  const VisionMissionCommand field_center = {
    .target_x_mm = 0,
    .target_y_mm = 0
  };
  if (!task_prepare_navigation(&field_center, pose, true)) {
    task_stop(TASK_FAULT_MOTOR, now_ms);
  } else if (navigation_turned) {
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
             (state == TASK_ALIGN_SAFE_ZONE)) {
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
        task_enter(TASK_PILE_APPROACH, now_ms);
      }
      break;

    case TASK_PILE_APPROACH:
      task_process_pile_approach(now_ms);
      break;

    case TASK_SCATTER_POSITIVE:
      task_process_scatter_positive(now_ms);
      break;

    case TASK_SCATTER_PAUSE:
      task_process_scatter_pause(now_ms);
      break;

    case TASK_SCATTER_NEGATIVE:
      task_process_scatter_negative(now_ms);
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
      if (Claw_Open(now_ms)) {
        task_status.gripper_closed = false;
        task_enter(TASK_RAM_BACK, now_ms);
      }
      break;

    case TASK_RAM_BACK:
      task_process_ram_back(now_ms);
      break;

    case TASK_RAM_FORWARD:
      task_process_ram_forward(now_ms);
      break;

    case TASK_RAM_VERIFY:
      task_process_ram_verify(&vision.mission, now_ms);
      break;

    case TASK_EXIT_SAFE_ZONE:
      task_process_safe_exit(now_ms);
      break;

    case TASK_FACE_FIELD_CENTER:
      task_process_face_center(&vision.fused_pose, now_ms);
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
