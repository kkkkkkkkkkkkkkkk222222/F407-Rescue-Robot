/* NormalRun regression for the red-1 start and first-green field reports.
 * Link the production UART parser, task and mechanism; mock hardware I/O only. */
#define TEST_WITH_VISION_LINK
#define main injury_fixture_main
#include "test_injury_backoff.c"
#undef main

static unsigned status_packets, start_acks;
static uint8_t sequence;
static uint32_t now, frame_id;

void BoardF405_ClearAlarmOnConfig(void) {}
void BoardF405_RequestBeep(uint8_t count, bool long_beep) { (void)count; (void)long_beep; }
void BoardF405_SetOperatorAlarm(bool enabled) { (void)enabled; }
bool DebugTelemetry_RxByte(uint8_t byte, bool idle) { (void)byte; (void)idle; return false; }
#include "vision.c"
#include <stdlib.h>
#undef assert
#define assert(condition) do { if (!(condition)) { \
  fprintf(stderr, "line %d, state %d, time %lu: %s\n", __LINE__, (int)state, \
          (unsigned long)now, #condition); exit(1); } } while (0)

bool Uart_Send(const uint8_t *bytes, uint16_t size)
{
  if (size == sizeof(config_ack) && memcmp(bytes, config_ack, size) == 0) ++start_acks;
  if (size == 2U * VISION_FRAME_SIZE && bytes[2] == VISION_MSG_STATUS_CONTEXT &&
      bytes[VISION_FRAME_SIZE + 2U] == VISION_MSG_STM_STATUS) ++status_packets;
  return true;
}

static void receive(uint8_t type, uint8_t seq, const uint8_t payload[8])
{
  uint8_t bytes[VISION_FRAME_SIZE];
  vision_build_frame(bytes, type, seq, payload);
  Vision_ParseBytes(bytes, 4U, now);
  Vision_ParseBytes(bytes + 4U, sizeof(bytes) - 4U, now);
}

static const uint8_t red_one[8] = {VISION_COLOR_RED, VISION_ZONE_1};

static void test_delayed_start(void)
{
  Vision_Init();
  Mechanism_Init();
  task_initialize(0U, false);
  for (now = 0U; now < 531000U; now += 20U) {
    Task_Process(now);
    Vision_Process();
  }
  assert(state == TASK_WAIT_CONFIG && status_packets > 0U && start_acks == 0U);
  uint8_t corrupt[VISION_FRAME_SIZE];
  vision_build_frame(corrupt, VISION_MSG_CONFIG, 0, red_one);
  corrupt[FRAME_CRC_LOW_INDEX] ^= 1U;
  Vision_ParseBytes(corrupt, sizeof(corrupt), now);
  Task_Process(now);
  assert(!latest_data.config_ready && state == TASK_WAIT_CONFIG && start_acks == 0U);
  for (uint8_t seq = 0; seq < 3U; ++seq) {
    receive(VISION_MSG_CONFIG, seq, red_one);
    Task_Process(now);
    Vision_Process();
    now += 40U;
  }
  assert(state == TASK_START && configured_color == VISION_COLOR_RED && start_acks > 0U);
  assert(latest_data.session_generation == 1U && latest_data.config_sequence == 0U);

  uint8_t abort_payload[8] = {VISION_CMD_ABORT, VISION_CMD_VALID};
  receive(VISION_MSG_MISSION, 31U, abort_payload);
  receive(VISION_MSG_CONFIG, 0U, red_one);
  assert(latest_data.session_generation == 1U);
  Task_Process(now += 20U);
  assert(state == TASK_STOPPED && task_status.fault == TASK_FAULT_REMOTE_STOP && !latest_data.config_ready);

  uint8_t resume[8] = {VISION_COLOR_RED, VISION_ZONE_1, VISION_CONFIG_RESUME,
                       VISION_RESUME_FIRST_GREEN, 1U, 30U};
  receive(VISION_MSG_CONFIG, 0U, resume);
  assert(!latest_data.config_ready);
  resume[5] = 31U;
  receive(VISION_MSG_CONFIG, 1U, resume);
  assert(latest_data.config_ready && latest_data.session_generation == 2U);
  for (unsigned i = 0; i < 200 && state != TASK_START; ++i) Task_Process(now += 20U);
  assert(state == TASK_START && first_delivery_done);
}

static void test_config_in_recovery(void)
{
  Vision_Init();
  task_initialize(0U, true);
  task_stop(TASK_FAULT_POSE_TIMEOUT, 100U);
  now = 531000U;
  receive(VISION_MSG_CONFIG, 0U, red_one);
  Task_Process(now);
  assert(latest_data.config_ready && state == TASK_RECOVERY_WAIT && ack_remaining == 0U);
}

static void prepare_batch(TaskState entry)
{
  Vision_Init();
  prepare();
  now = 100U;
  frame_id = 10U;
  sequence = 4U;
  receive(VISION_MSG_CONFIG, 0U, red_one);
  VisionData session = Vision_GetSnapshot();
  task_accept_mission(&session, now);
  first_delivery_done = false;
  configured_color = VISION_COLOR_RED;
  task_enter(entry, now);
  accepted_context = (TaskContext){.valid = true, .task_id = 7U, .action_id = 10U,
      .opcode = VISION_CMD_APPROACH_TARGET};
}

static bool send_action(uint16_t action_id, uint32_t frame, const uint8_t payload[8])
{
  uint8_t context[8] = {0, 7, action_id >> 8, (uint8_t)action_id, frame >> 24,
                        frame >> 16, frame >> 8, (uint8_t)frame};
  now += 100U;
  ++sequence;
  receive(VISION_MSG_COMMAND_CONTEXT, sequence, context);
  receive(VISION_MSG_MISSION, sequence, payload);
  VisionData data = Vision_GetSnapshot();
  assert(data.mission.context_valid && data.mission.action_id == action_id);
  command_accepted = false;
  task_accept_mission(&data, now);
  return command_accepted;
}

static void finish_curve(void)
{
  for (unsigned i = 0; i < 200U && !remote_action.done; ++i) {
    if (separation_motion.active) test_pose.path_mm += 20U;
    task_process_remote_action(&active_mission, now += 20U);
  }
  assert(remote_action.done && task_protocol_mode() == 35U && cargo_recheck_pending);
}

static void test_gate(TaskState entry)
{
  prepare_batch(entry);
  Camera_SetAngle(120U);
  task_status.gripper_closed = true;
  audit_received = audit_host_stable = audit_valid = true;
  confirmed_load.valid = nav_reuse_post_grab_audit = true;
  uint8_t conflict[8] = {VISION_CMD_CARGO_AUDIT, VISION_CMD_VALID, 0, 0, 0,
                         VISION_AUDIT_FIRST_GREEN_CORE_CONFLICT, 1U, 0};
  /* A reserved count nibble remains illegal, even alongside the new flag. */
  conflict[4] = 0x10;
  assert(!send_action(11U, 0U, conflict) && !first_green_core_conflict);
  conflict[4] = 0;
  if (entry == TASK_ALIGN_SAFE_ZONE) {
    first_delivery_done = true;
    (void)send_action(11U, 0U, conflict);
    assert(!first_green_core_conflict && confirmed_load.valid);
    first_delivery_done = false;
    audit_initial_stash = true;
    (void)send_action(11U, 0U, conflict);
    assert(!first_green_core_conflict && confirmed_load.valid);
    audit_initial_stash = false;
  }
  assert(send_action(11U, 0U, conflict));
  assert(state == (entry == TASK_WAIT_NAVIGATION ? TASK_POST_GRAB_AUDIT : TASK_TRANSPORT_AUDIT));
  if (entry == TASK_WAIT_NAVIGATION) task_process_post_grab_audit(now += 20U);
  assert(Camera_GetAngle() == 140U && task_status.gripper_closed && first_green_core_conflict);
  assert(!confirmed_load.valid && !nav_reuse_post_grab_audit && !audit_valid && !audit_received);
  assert(audit_consistent_count == 0U && send_action(11U, 0U, conflict) && audit_consistent_count == 0U);
  if (state != TASK_TRANSPORT_AUDIT) return;
  task_process_transport_audit(now += 500U);
  conflict[2] = VISION_CARGO_GREEN;
  conflict[4] = conflict[7] = 1U;
  for (uint8_t id = 2; id <= 3; ++id) {
    conflict[6] = id;
    if (id == 3) conflict[5] |= VISION_AUDIT_STABLE;
    assert(send_action(11U, ++frame_id, conflict));
  }
  assert(audit_received && !audit_valid && audit_last_total_count == 1U && audit_last_counts == 1U);
  const uint8_t keep_left[8] = {VISION_CMD_DISPERSE_PILE, VISION_CMD_VALID | VISION_CMD_SIDE_VALID};
  assert(send_action(12U, 0U, keep_left) && first_green_core_conflict);
  assert(send_action(12U, 0U, keep_left) && separation_total_attempts == 1U);
  finish_curve();
  assert(!first_green_core_conflict);
  conflict[5] = 0;
  ++conflict[6];
  assert(send_action(13U, ++frame_id, conflict) && !audit_valid);
  conflict[5] = VISION_AUDIT_STABLE;
  ++conflict[6];
  assert(send_action(13U, ++frame_id, conflict) && audit_valid);
}

static void run_five_separations(bool keep_single)
{
  prepare_batch(TASK_DISPERSE_READY);
  task_status.gripper_closed = false;
  task_status.claw_visible = true;
  Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
  uint16_t action = 11U;
  uint8_t audit[8] = {VISION_CMD_CARGO_AUDIT, VISION_CMD_VALID, VISION_CARGO_GREEN,
                      VISION_CARGO_DANGER, 5U, VISION_AUDIT_DANGER_PRESENT, 0, 2U};
  uint8_t separate[8] = {VISION_CMD_DISPERSE_PILE, VISION_CMD_VALID | VISION_CMD_SIDE_VALID};
  for (unsigned round = 0; round <= 5U; ++round) {
    if (round == 5U && keep_single) {
      audit[3] = VISION_CARGO_NONE;
      audit[4] = audit[7] = 1U;
      audit[5] = 0U;
    }
    audit[5] &= (uint8_t)~VISION_AUDIT_STABLE;
    ++audit[6];
    assert(send_action(action, ++frame_id, audit));
    ++audit[6];
    audit[5] |= VISION_AUDIT_STABLE;
    assert(send_action(action++, ++frame_id, audit));
    assert(audit_received && audit_valid == (round == 5U && keep_single));
    if (round == 5U) break;
    assert(send_action(action, 0U, separate) && state == TASK_REMOTE_ACTION);
    assert(send_action(action++, 0U, separate) && separation_total_attempts == round + 1U);
    finish_curve();
  }
  uint8_t decision[8] = {keep_single ? VISION_CMD_GRAB_CONFIRMED : VISION_CMD_RELEASE_BOTH,
                         VISION_CMD_VALID | (keep_single ? 0U : VISION_CMD_RED_SIDE)};
  assert(send_action(action, 0U, decision));
  if (keep_single) assert(state == TASK_CLOSE_CLAW);
  else assert(state == TASK_APPROACH_RECOVER || remote_action.type == REMOTE_ACTION_RELEASE_BOTH);
}

/* Optional upper-computer capture: three CONFIG frames and the first paired
 * mode10 withdrawal (task 7/action 11) from the same protocol revision. */
static void replay_upper_frames(const char *path)
{
  uint8_t bytes[5U * VISION_FRAME_SIZE];
  FILE *file = fopen(path, "rb");
  assert(file && fread(bytes, 1, sizeof(bytes), file) == sizeof(bytes));
  assert(fgetc(file) == EOF);
  fclose(file);
  prepare_batch(TASK_ALIGN_SAFE_ZONE);
  Vision_Init();
  for (size_t i = 0; i < 3U * VISION_FRAME_SIZE; ++i) Vision_ParseBytes(bytes + i, 1U, now);
  assert(latest_data.config_ready && latest_data.session_generation == 1U &&
         latest_data.config_sequence == 0U && latest_data.color == VISION_COLOR_RED);
  task_status.gripper_closed = true;
  Camera_SetAngle(120U);
  Vision_ParseBytes(bytes + 3U * VISION_FRAME_SIZE, 2U * VISION_FRAME_SIZE, now += 100U);
  VisionData data = Vision_GetSnapshot();
  assert(data.mission.context_valid && data.mission.payload_valid && data.mission.audit_flags == 0x80U);
  task_accept_mission(&data, now);
  assert(command_accepted && first_green_core_conflict && task_protocol_mode() == 48U &&
         Camera_GetAngle() == 140U && !audit_received && audit_consistent_count == 0U);
}

int main(int argc, char **argv)
{
  test_delayed_start();
  test_config_in_recovery();
  test_gate(TASK_ALIGN_SAFE_ZONE);
  test_gate(TASK_NAVIGATE);
  test_gate(TASK_WAIT_NAVIGATION);
  run_five_separations(true);
  run_five_separations(false);
  if (argc == 2) replay_upper_frames(argv[1]);
  puts("PASS delayed CONFIG, STOP/restart, first-green withdrawal and five physical action decisions");
  return 0;
}
