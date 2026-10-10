#define main injury_sequence_main
#include "test_injury_backoff.c"
#undef main
#include <stdlib.h>
#undef assert
#define assert(test) ((test) ? (void)0 : \
    (fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #test), exit(1)))

static void test_receipt_cadence_cannot_outrun_host(void)
{
  prepare(); injury_recheck_pending = true;
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.audit_flags &= (uint8_t)~VISION_AUDIT_STABLE;
    command_visual_new = true;
    task_accept_mission_legacy(&c, 1500U + id * 100U);
  }
  assert(audit_consistent_count == 3U && !audit_received && !audit_valid && !audit_host_stable);
  assert(state == TASK_POST_GRAB_AUDIT && !task_context_complete());
  audit_at(4U, 1900U);
  assert(state == TASK_INJURY_RECLOSE && audit_valid);
}

static void test_stable_does_not_skip_three_real_frames(void)
{
  prepare();
  for (uint8_t id = 1U; id <= 3U; ++id) { audit_at(id, id * 10U); }
  assert(!audit_received && !audit_valid && state == TASK_POST_GRAB_AUDIT);
}

static void test_same_frame_confirmation_has_no_new_sample_or_freshness(void)
{
  prepare(); injury_recheck_pending = true;
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.audit_flags &= (uint8_t)~VISION_AUDIT_STABLE;
    command_visual_new = true; task_accept_mission_legacy(&c, id * 100U);
  }
  const uint32_t evidence_ms = audit_evidence_ms;
  accepted_context.visual_valid = true;
  accepted_context.vision_frame = 4U;
  VisionMissionCommand c = injury_audit(3U);
  command_visual_new = false;
  task_accept_mission_legacy(&c, 320U);
  assert(state == TASK_INJURY_RECLOSE && audit_consistent_count == 3U);
  assert(audit_evidence_ms == evidence_ms);
}

static void test_invalid_histogram_and_side_jitter(void)
{
  prepare();
  VisionMissionCommand c = injury_audit(1U);
  c.audit_left_class = VISION_CARGO_GREEN;
  c.audit_counts = c.audit_total_count = 2U;
  c.audit_flags = VISION_AUDIT_STABLE;
  for (uint8_t id = 1U; id <= 3U; ++id) {
    c.audit_id = id;
    c.audit_left_class = id % 2U ? VISION_CARGO_GREEN : VISION_CARGO_CORE;
    command_visual_new = true; task_latch_audit(&c, id * 100U);
    assert(audit_consistent_count == 1U && !audit_received);
  }
  c.audit_id = 4U;
  c.audit_left_class = VISION_CARGO_NONE; c.audit_right_class = VISION_CARGO_GREEN;
  c.audit_counts = 2U << 2;
  task_latch_audit(&c, 400U);
  assert(audit_consistent_count == 2U);
  c.audit_id = 5U;
  c.audit_left_class = VISION_CARGO_GREEN; c.audit_right_class = VISION_CARGO_NONE;
  c.audit_counts = 2U;
  task_latch_audit(&c, 500U);
  assert(audit_received && audit_host_stable && !audit_valid);
}

static void wire_prepare(TaskState target, uint32_t now)
{
  prepare();
  task_enter(target, now);
  accepted_context = (TaskContext){.valid = true, .visual_valid = true,
      .task_id = 7U, .action_id = 10U, .opcode = VISION_CMD_APPROACH_TARGET,
      .flags = VISION_CMD_VALID, .vision_frame = 8U};
  command_generation_seen = 0U;
}

static void wire_send(VisionMissionCommand command, uint32_t now, uint32_t generation)
{
  command.received = command.context_valid = command.payload_valid = true;
  command.task_id = 7U; command.action_id = 11U;
  command.tick_ms = now; command.generation = generation; command.sequence = generation;
  VisionData vision = {.config_ready = true, .mission = command};
  command_accepted = false;
  task_accept_mission(&vision, now);
}

static VisionMissionCommand placeholder(void)
{
  return (VisionMissionCommand){.command = VISION_CMD_CARGO_AUDIT,
                               .flags = VISION_CMD_VALID, .received = true};
}

static void test_sweep_placeholder_ack_and_real_role_validation(void)
{
  wire_prepare(TASK_SAFE_SWEEP_AUDIT, 1000U);
  task_reset_sweep_audit(1000U);
  wire_send(placeholder(), 1050U, 1U);
  assert(command_accepted && accepted_context.action_id == 11U);
  assert(!sweep_audit_ready && sweep_audit_started_ms == 1000U);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.vision_frame = 8U + id;
    c.audit_flags = VISION_AUDIT_SWEEP_PICKUP | VISION_AUDIT_STABLE;
    wire_send(c, 1100U + id * 100U, id + 1U);
  }
  assert(sweep_audit_ready && sweep_audit_valid && state == TASK_SAFE_SWEEP_AUDIT);
  VisionMissionCommand wrong = injury_audit(4U);
  wrong.vision_frame = 12U;
  wrong.audit_flags = VISION_AUDIT_STABLE; /* a real mode43 frame must have SWEEP */
  wire_send(wrong, 1500U, 5U);
  assert(!command_accepted && accepted_context.rejection_valid);
  assert(sweep_audit_valid);
}

static void test_injury_retrieve_placeholder_does_not_change_destination(void)
{
  wire_prepare(TASK_SAFE_SWEEP_RETRIEVE_AUDIT, 1000U);
  sweep_original_injury = true;
  wire_send(placeholder(), 1050U, 1U);
  assert(command_accepted && sweep_original_injury && !audit_received);
  VisionMissionCommand wrong = injury_audit(1U);
  wrong.vision_frame = 9U; wrong.audit_flags = VISION_AUDIT_STABLE;
  wire_send(wrong, 1100U, 2U);
  assert(!command_accepted && accepted_context.rejection_valid);
}

static void test_placeholder_never_means_empty_or_resets_deadline(void)
{
  prepare(); injury_recheck_pending = true;
  task_audit_watchdog(0U);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    command_visual_new = true; task_latch_audit(&c, id * 100U);
  }
  assert(audit_valid);
  VisionMissionCommand empty = placeholder();
  command_visual_new = false;
  task_accept_mission_legacy(&empty, 400U);
  assert(state == TASK_POST_GRAB_AUDIT && !audit_valid && !audit_received);
  task_audit_watchdog(400U);
  assert(audit_watch_started_ms == 0U);
  task_audit_watchdog(15000U);
  assert(state == TASK_APPROACH_RECOVER);
}

static void test_final_wait_keeps_snapshot_and_expires(void)
{
  for (uint8_t sweep = 0U; sweep <= 1U; ++sweep) {
    prepare();
    for (uint8_t id = 1U; id <= 3U; ++id) {
      VisionMissionCommand c = injury_audit(id);
      command_visual_new = true; task_latch_audit(&c, id * 100U);
    }
    task_enter(sweep ? TASK_SAFE_SWEEP_DONE : TASK_WAIT_NAVIGATION, 400U);
    task_audit_watchdog(400U);
    VisionMissionCommand empty = placeholder();
    task_accept_mission_legacy(&empty, 500U);
    assert(audit_valid && audit_received && audit_host_stable);
    task_audit_watchdog(15400U);
    assert(state == TASK_APPROACH_RECOVER);
    assert(capture_escape_return_center == (sweep != 0U));
  }
}

static void test_zero_frame_stable_is_rejected(void)
{
  wire_prepare(TASK_POST_GRAB_AUDIT, 1000U);
  VisionMissionCommand bad = placeholder();
  bad.audit_flags = VISION_AUDIT_STABLE;
  wire_send(bad, 1050U, 1U);
  assert(!command_accepted && !audit_received && state == TASK_POST_GRAB_AUDIT);
}

int main(void)
{
  test_receipt_cadence_cannot_outrun_host();
  test_stable_does_not_skip_three_real_frames();
  test_same_frame_confirmation_has_no_new_sample_or_freshness();
  test_invalid_histogram_and_side_jitter();
  test_sweep_placeholder_ack_and_real_role_validation();
  test_injury_retrieve_placeholder_does_not_change_destination();
  test_placeholder_never_means_empty_or_resets_deadline();
  test_final_wait_keeps_snapshot_and_expires();
  test_zero_frame_stable_is_rejected();
  puts("audit handshake: 9 scenarios PASS");
  return 0;
}
