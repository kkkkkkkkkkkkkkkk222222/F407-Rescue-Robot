/* Exercise the production Task and mechanism with hardware replaced by mocks. */
#include <assert.h>
#include <stdio.h>
#include "../Main/Src/Task.c"
#define camera_angle mechanism_camera_angle
#include "../Main/Src/mechanism.c"
#undef camera_angle

static uint8_t servo_angles[4];
static float reverse_output;
static LocationPose test_pose = {.valid = true, .heading_mdeg = 90000};
static bool motor_fault;
void Servo_SetAngle(uint8_t id, uint8_t angle) { servo_angles[id - 1U] = angle; }
void Motor_Stop(void) { reverse_output = 0.0f; }
void Motor_Move(float forward, float lateral, float yaw)
{ (void)lateral; (void)yaw; reverse_output = forward; }
MotorStatus Motor_GetStatus(uint8_t id)
{ (void)id; return (MotorStatus){.stall_fault = motor_fault}; }
LocationPose Location_GetPose(void) { return test_pose; }
VisionData Vision_GetSnapshot(void) { return (VisionData){0}; }
void Vision_RearmConfig(void) {}
void Vision_ClearAbort(void) {}
void Vision_RequestConfigAck(void) {}
void Vision_QueueStmStatus(const VisionStmStatus *value) { (void)value; }
IMUData IMU_GetData(void) { return (IMUData){.ready = true}; }
MotorDistanceStatus Motor_MoveDistance(float distance, float speed)
{ (void)distance; (void)speed; return MOTOR_DISTANCE_DONE; }
MotorDistanceStatus Motor_MoveDistanceLinear(float distance, float speed, float slow, float end)
{ (void)distance; (void)speed; (void)slow; (void)end; return MOTOR_DISTANCE_DONE; }
MotorDistanceStatus Motor_MoveDistanceConstant(float distance, float speed)
{ (void)distance; (void)speed; return MOTOR_DISTANCE_DONE; }
MotorTurnStatus Motor_TurnAngle(float angle) { (void)angle; return MOTOR_TURN_DONE; }
MotorTurnStatus Motor_TurnAngleSpeed(float angle, float speed)
{ (void)angle; (void)speed; return MOTOR_TURN_DONE; }
bool Motor_MoveAngle(float speed, float angle) { (void)speed; (void)angle; return true; }
bool Motor_MoveSpin(float speed, float angle, float yaw)
{ (void)speed; (void)angle; (void)yaw; return true; }
void Location_Reset(LocationStart start) { (void)start; }

static VisionMissionCommand injury_audit(uint8_t id)
{
  return (VisionMissionCommand){.received = true, .flags = VISION_CMD_VALID,
    .command = VISION_CMD_CARGO_AUDIT, .sequence = id, .audit_id = id,
    .audit_left_class = VISION_CARGO_INJURED, .audit_right_class = VISION_CARGO_NONE,
    .audit_counts = 1U, .audit_total_count = 1U,
    .audit_flags = VISION_AUDIT_DESTINATION_INJURY | VISION_AUDIT_STABLE,
    .vision_frame = (uint32_t)id + 1U};
}

static void prepare(void)
{
  Mechanism_Init();
  task_initialize(0U);
  accepted_context = (TaskContext){.valid = true, .opcode = VISION_CMD_CARGO_AUDIT};
  first_delivery_done = true;
  test_pose.valid = true;
  motor_fault = false;
  complete_flow_active = true;
  task_status.gripper_closed = true;
  Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
  task_enter(TASK_POST_GRAB_AUDIT, 0U);
}

static void audit_at(uint8_t id, uint32_t now)
{
  VisionMissionCommand command = injury_audit(id);
  command_visual_new = true;
  task_accept_mission_legacy(&command, now);
}

static void tick(uint32_t now)
{
  accepted_context.last_command_rx_ms = now;
  task_process_injury_backoff(now);
}

static void test_complete_injury_sequence(void)
{
  prepare();
  audit_at(1U, 10U); audit_at(1U, 20U); audit_at(2U, 110U);
  assert(state == TASK_POST_GRAB_AUDIT && !audit_received);
  audit_at(3U, 210U);
  assert(state == TASK_INJURY_BACKOFF && !audit_valid);
  VisionMissionCommand grab = {.received = true, .flags = VISION_CMD_VALID,
    .command = VISION_CMD_GRAB_CONFIRMED};
  task_accept_mission_legacy(&grab, 211U);
  assert(state == TASK_INJURY_BACKOFF && injury_backoff_elapsed_ms == 0U);
  tick(250U);
  assert(servo_angles[3] == 81U && servo_angles[1] == 99U);
  assert(reverse_output == 0.0f);
  audit_at(4U, 260U);
  assert(state == TASK_INJURY_BACKOFF && audit_consistent_count == 0U);
  tick(849U); assert(reverse_output == 0.0f);
  tick(850U); assert(reverse_output == -150.0f);
  tick(2849U); assert(state == TASK_INJURY_BACKOFF && reverse_output == -150.0f);
  tick(2850U); assert(reverse_output == 0.0f && Camera_GetAngle() == 140U);
  tick(3349U); assert(state == TASK_INJURY_BACKOFF);
  tick(3350U); assert(state == TASK_POST_GRAB_AUDIT && !audit_valid);
  assert(injury_recheck_pending && !task_status.gripper_closed);
  audit_at(5U, 3360U); audit_at(5U, 3370U); audit_at(6U, 3460U);
  assert(state == TASK_POST_GRAB_AUDIT);
  audit_at(7U, 3560U);
  assert(state == TASK_INJURY_RECLOSE && !task_status.gripper_closed);
  VisionMissionCommand nav = {.received = true, .flags = VISION_CMD_VALID,
    .command = VISION_CMD_NAVIGATE_WAYPOINT};
  command_accepted = false;
  task_accept_mission_legacy(&nav, 3561U);
  assert(!command_accepted && state == TASK_INJURY_RECLOSE);
  assert(!Claw_Touch(3600U));
  assert(servo_angles[3] == 76U && servo_angles[1] == 104U);
  assert(!Claw_Touch(5599U)); assert(Claw_Touch(5600U));
}

static void test_pause_does_not_consume_reverse_time(void)
{
  prepare(); task_enter(TASK_INJURY_BACKOFF, 0U);
  tick(0U); tick(600U); tick(1100U);
  assert(injury_backoff_elapsed_ms == 500U);
  task_pause_runtime(1100U);
  task_pause_runtime(6100U);
  tick(6100U);
  assert(injury_backoff_elapsed_ms == 500U && reverse_output == -150.0f);
  tick(7600U);
  assert(reverse_output == 0.0f && injury_backoff_phase == 2U);
}

static void test_empty_reaudit(void)
{
  prepare(); injury_recheck_pending = true;
  for (uint8_t i = 1U; i <= 3U; ++i) {
    VisionMissionCommand command = injury_audit(i);
    command.audit_left_class = VISION_CARGO_NONE;
    command.audit_counts = command.audit_total_count = 0U;
    command_visual_new = true;
    task_accept_mission_legacy(&command, i * 100U);
  }
  assert(state == TASK_OPEN_CLAW && !task_status.gripper_closed);
}

static void test_invalid_reaudit_cannot_close(void)
{
  prepare(); injury_recheck_pending = true;
  for (uint8_t i = 1U; i <= 3U; ++i) {
    VisionMissionCommand command = injury_audit(i);
    command.audit_counts = command.audit_total_count = 2U;
    command_visual_new = true;
    task_accept_mission_legacy(&command, i * 100U);
  }
  assert(audit_received && !audit_valid && state == TASK_POST_GRAB_AUDIT);
  task_process_post_grab_audit(40U);
  assert(!task_status.gripper_closed);
}

static void test_ordinary_and_stash_skip_backoff(void)
{
  for (uint8_t stash = 0U; stash <= 1U; ++stash) {
    prepare();
    for (uint8_t i = 1U; i <= 3U; ++i) {
      VisionMissionCommand command = injury_audit(i);
      command.audit_left_class = VISION_CARGO_GREEN;
      command.audit_flags = VISION_AUDIT_STABLE |
          (stash ? VISION_AUDIT_INITIAL_STASH : 0U);
      command_visual_new = true;
      task_accept_mission_legacy(&command, i * 100U);
    }
    assert(state == TASK_WAIT_NAVIGATION && !injury_recheck_pending);
  }
}

int main(void)
{
  test_complete_injury_sequence();
  test_pause_does_not_consume_reverse_time();
  test_empty_reaudit();
  test_invalid_reaudit_cannot_close();
  test_ordinary_and_stash_skip_backoff();
  puts("injury firmware sequence: 5 scenarios PASS");
  return 0;
}
