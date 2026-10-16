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

static void separation_audit_at(bool single, bool swap_sides, uint32_t start_ms)
{
  task_clear_audit_result();
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.audit_flags = VISION_AUDIT_STABLE | VISION_AUDIT_DANGER_PRESENT;
    c.audit_left_class = VISION_CARGO_GREEN;
    c.audit_right_class = single ? VISION_CARGO_DANGER : VISION_CARGO_NONE;
    c.audit_counts = single ? 5U : 2U;
    c.audit_total_count = 2U;
    if (swap_sides && id == 2U) {
      c.audit_left_class = VISION_CARGO_DANGER;
      c.audit_right_class = VISION_CARGO_GREEN;
    }
    command_visual_new = true;
    task_latch_audit(&c, start_ms + id * 100U);
  }
  assert(audit_received && !audit_valid);
}

static void separation_audit(bool single, bool swap_sides)
{
  separation_audit_at(single, swap_sides, 0U);
}

static void test_quantity_grip_and_retry_budget(void)
{
  for (uint8_t single = 0U; single <= 1U; ++single) {
    prepare(); separation_audit(single, false);
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
      assert(task_prepare_separation(1, 400U + attempt));
      assert(separation_extra_close_deg == (single ? 20U : 10U) - 5U * attempt);
      (void)Claw_ClusterOpenRight(500U + attempt);
      assert(servo_angles[3] == 76U - separation_extra_close_deg && servo_angles[1] == 72U);
    }
    assert(!task_prepare_separation(-1, 410U) && separation_total_attempts == 3U);
    assert(capture_escape_return_center && capture_escape_reclose);
  }
  prepare(); separation_audit(false, false);
  assert(task_prepare_separation(1, 400U) && separation_extra_close_deg == 10U);
  separation_audit_at(true, false, 400U);
  assert(task_prepare_separation(1, 800U) && separation_extra_close_deg == 20U);
  assert(separation_total_attempts == 2U && separation_side_attempts == 1U);
  assert(task_prepare_separation(1, 900U) && separation_extra_close_deg == 15U);
  prepare(); separation_audit(true, true);
  assert(audit_single_side_mask == 0U);
  assert(task_prepare_separation(1, 400U) && separation_extra_close_deg == 10U);
  prepare();
  task_enter(TASK_GRAB_OBSERVE, 0U);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.audit_left_class = VISION_CARGO_UNKNOWN;
    c.audit_counts = c.audit_total_count = 2U;
    c.audit_flags = VISION_AUDIT_STABLE | VISION_AUDIT_UNKNOWN_PRESENT;
    command_visual_new = true; task_latch_audit(&c, id * 100U);
  }
  (void)task_capture_observe(400U);
  assert(state == TASK_GRAB_OBSERVE && !capture_escape_active);
  assert(task_prepare_separation(1, 410U) && separation_extra_close_deg == 10U);
  VisionMissionCommand c = injury_audit(1U);
  assert(task_single_side_mask(&c) == 1U);
  c.audit_left_class = VISION_CARGO_GREEN;
  assert(task_single_side_mask(&c) == 0U);
  c.audit_flags = VISION_AUDIT_STABLE;
  assert(task_single_side_mask(&c) == 1U);
  first_delivery_done = false;
  c.audit_left_class = VISION_CARGO_CORE;
  assert(task_single_side_mask(&c) == 0U);
}

static void test_ready_mode_reaudit_and_stale_grip_rejection(void)
{
  wire_prepare(TASK_DISPERSE_READY, 0U);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.vision_frame = 8U + id;
    c.audit_left_class = VISION_CARGO_GREEN;
    c.audit_right_class = VISION_CARGO_DANGER;
    c.audit_counts = 5U; c.audit_total_count = 2U;
    c.audit_flags = VISION_AUDIT_STABLE | VISION_AUDIT_DANGER_PRESENT;
    wire_send(c, id * 100U, id);
  }
  assert(state == TASK_DISPERSE_READY && task_status.claw_visible && task_audit_state());
  assert(audit_received && !audit_valid && audit_single_side_mask == 1U);
  assert(!task_prepare_separation(1, 900U));
  assert(task_status.command_reject_reason == TASK_COMMAND_REJECT_AUDIT && separation_total_attempts == 0U);
}

static void test_curve_mirrors_limits_and_pause(void)
{
  for (int8_t side = -1; side <= 1; side += 2) {
    prepare();
    separation_keep_side = side; separation_total_attempts = 1U;
    const VisionMissionCommand c = {.received = true, .flags = VISION_CMD_VALID,
                                    .command = VISION_CMD_DISPERSE_PILE};
    task_start_remote_action(REMOTE_ACTION_DISPERSE, &c, 0U);
    remote_action.phase = 1U;
    assert(!task_remote_separation_curve(300U, 0U));
    assert(!task_remote_separation_curve(300U, 20U));
    assert(reverse_output < 0.0f && side * lateral_output > 0.0f);
    assert(side * yaw_output > 0.0f); /* field yaw is converted once to wheel yaw */
    test_pose.path_mm = 65U; test_pose.y_mm = -65;
    test_pose.heading_mdeg = 90000 - side * 12000;
    (void)task_remote_separation_curve(300U, 40U);
    assert(separation_motion.stage == SEPARATION_PULL);
    assert(fabsf(separation_motion.pull_heading_deg - test_pose.heading_mdeg * 0.001f) < 0.01f);
    const float locked = separation_motion.pull_heading_deg;
    task_pause_action(60U, false);
    assert(reverse_output == 0.0f && separation_motion.back_mm_s == 0.0f);
    task_process_remote_action(&c, 5060U);
    assert(separation_motion.start_path_mm == 0U && separation_motion.pull_heading_deg == locked);
    assert(reverse_output < 0.0f && fabsf(reverse_output) <= 36.01f);
    test_pose.path_mm = 300U;
    assert(task_remote_separation_curve(300U, 5080U));
    assert(remote_action.phase == 2U && reverse_output == 0.0f);
  }
  prepare(); separation_keep_side = 1; separation_total_attempts = 1U;
  (void)task_remote_separation_curve(300U, 0U);
  (void)task_remote_separation_curve(300U, 20U);
  test_pose.x_mm = -70; test_pose.path_mm = 80U;
  assert(task_remote_separation_curve(300U, 40U));
  assert(lateral_output == 0.0f && reverse_output == 0.0f);
  prepare(); separation_keep_side = 1; separation_total_attempts = 1U;
  (void)task_remote_separation_curve(300U, 0U);
  (void)task_remote_separation_curve(300U, 20U);
  test_pose.heading_mdeg = 64000; test_pose.path_mm = 70U;
  (void)task_remote_separation_curve(300U, 40U);
  assert(yaw_output == 0.0f && separation_motion.stage == SEPARATION_PULL);
  prepare(); separation_keep_side = 1; separation_total_attempts = 1U;
  test_pose.heading_mdeg = 30000;
  (void)task_remote_separation_curve(300U, 0U);
  for (uint32_t mm = 1U; mm <= 100U; ++mm) {
    test_pose.x_mm = (int32_t)lroundf(-0.8660254f * mm);
    test_pose.y_mm = (int32_t)lroundf(-0.5f * mm);
    test_pose.path_mm = mm;
    assert(!task_remote_separation_curve(300U, mm * 20U));
  }
  assert(separation_motion.lateral_peak_mm < 1.0f);
  prepare(); separation_keep_side = 1; separation_total_attempts = 1U;
  (void)task_remote_separation_curve(300U, 0U);
  separation_motion.stage = SEPARATION_PULL;
  separation_motion.pull_heading_deg = 70.0f;
  separation_motion.back_mm_s = 500.0f;
  test_pose.heading_mdeg = 70000; test_pose.x_mm = -50; test_pose.path_mm = 200U;
  assert(!task_remote_separation_curve(300U, 20U));
  assert(reverse_output < 0.0f && fabsf(reverse_output) < 150.0f && yaw_output == 0.0f);
}

static void test_separation_retransmission_is_one_round(void)
{
  wire_prepare(TASK_GRAB_OBSERVE, 0U);
  separation_audit(true, false);
  task_enter(TASK_DISPERSE_READY, 350U);
  const VisionMissionCommand c = {.command = VISION_CMD_DISPERSE_PILE,
      .flags = VISION_CMD_VALID | VISION_CMD_SIDE_VALID};
  wire_send(c, 400U, 1U);
  assert(command_accepted && state == TASK_REMOTE_ACTION && separation_total_attempts == 1U);
  wire_send(c, 420U, 2U);
  assert(command_accepted && separation_total_attempts == 1U && remote_action.phase == 0U);
  wire_send((VisionMissionCommand){.command = VISION_CMD_PAUSE, .flags = VISION_CMD_VALID},
            440U, 3U);
  assert(mission_paused && separation_total_attempts == 1U);
  wire_send(c, 460U, 4U);
  assert(!mission_paused && separation_total_attempts == 1U && separation_extra_close_deg == 20U);
}

static void test_third_failure_recloses_and_reopens_at_center(void)
{
  prepare(); separation_total_attempts = 3U; audit_received = true; audit_valid = false;
  task_begin_capture_escape(0U);
  task_process_capture_escape(0U);
  task_process_capture_escape(1000U);
  assert(capture_escape_phase == 1U && !task_status.gripper_closed);
  test_pose.path_mm = 300U;
  task_process_capture_escape(1100U);
  task_process_capture_escape(1200U);
  assert(state == TASK_APPROACH_RECOVER && servo_angles[3] == 76U && servo_angles[1] == 104U);
  task_process_capture_escape(3199U);
  assert(state == TASK_APPROACH_RECOVER);
  task_process_capture_escape(3200U);
  assert(state == TASK_CAPTURE_RETURN_WAIT && capture_return_closed && task_status.gripper_closed);
  initial_claw_ready = true;
  Task_Process(3220U);
  assert(state == TASK_CAPTURE_RETURN_WAIT && task_status.gripper_closed && !audit_valid);
  configured_color = VISION_COLOR_BLUE;
  VisionData v = {.mission = {.received = true, .command = VISION_CMD_RETURN_CENTER,
      .flags = VISION_CMD_VALID | VISION_CMD_DISTANCE_VALID | VISION_CMD_DRIVE_STRAIGHT |
               VISION_CMD_USE_FINAL_HEADING, .heading_cdeg = 9000U}};
  task_accept_mission_legacy(&v.mission, 3240U);
  assert(state == TASK_FACE_FIELD_CENTER);
  task_process_face_center(&v, 3260U);
  assert(capture_return_opening && !distance_command_done);
  task_process_face_center(&v, 4259U);
  assert(state == TASK_FACE_FIELD_CENTER);
  task_process_face_center(&v, 4260U);
  assert(state == TASK_SEARCH && !task_status.gripper_closed && !capture_return_closed);
}

static void test_recoverable_wait(void)
{
  prepare();
  separation_total_attempts = 2;
  task_stop(TASK_FAULT_MOTOR, 100U);
  assert(state == TASK_RECOVERY_WAIT && task_status.fault == TASK_FAULT_NONE);
  assert(task_status.gripper_closed && separation_total_attempts == 2);
  assert(!task_status.claw_visible);
  VisionData v = {.fused_pose={.received=true, .status=VISION_POSE_VALID | VISION_POSE_T265_GOOD,
      .tracker_confidence=3, .heading_cdeg=9000, .x_mm=100, .y_mm=200}};
  for (unsigned t=700;t<=740;t+=20) {
    v.fused_pose.tick_ms=t;
    task_recover_reference(&v,t);
  }
  assert(recovery_reference_ready && task_status.claw_visible && state == TASK_RECOVERY_WAIT);
  audit_at(1,800); audit_at(2,900); audit_at(3,1000);
  assert(audit_received && audit_valid && state == TASK_RECOVERY_WAIT);
  VisionMissionCommand grab={.received=true,.flags=VISION_CMD_VALID,.command=VISION_CMD_GRAB_CONFIRMED};
  task_accept_mission_legacy(&grab,1010);
  assert(state == TASK_CLOSE_CLAW && !grab_core_advance_required);
}

int main(void)
{
#ifdef TWO_FRAME_REVIEW_CHECK
  prepare();
  task_enter(TASK_GRAB_OBSERVE, 0U);
  audit_at(1, 100);
  assert(!audit_received && audit_consistent_count == 1);
  audit_at(1, 150);
  assert(!audit_received && audit_consistent_count == 1);
  audit_at(2, 200);
  assert(audit_received && audit_valid && audit_consistent_count == 2);
  prepare(); task_enter(TASK_GRAB_OBSERVE, 0U);
  VisionMissionCommand c=injury_audit(1);
  c.audit_flags &= (uint8_t)~VISION_AUDIT_STABLE;
  command_visual_new=true; task_accept_mission_legacy(&c,100);
  c.audit_id=2; c.vision_frame=3; task_accept_mission_legacy(&c,200);
  assert(audit_consistent_count == 2 && !audit_received);
  assert(APP_SEPARATION_MAX_ATTEMPTS == 2 && !task_escape_allowed());
  prepare();
  separation_total_attempts=1; audit_received=true; audit_valid=false;
  audit_consistent_count=APP_AUDIT_REQUIRED_SAMPLES;
  audit_last_left_class=VISION_CARGO_GREEN; audit_last_counts=1;
  audit_last_total_count=2;
  capture_wait_active=true; capture_wait_started_ms=0;
  assert(task_capture_observe(5000));
  assert(state == TASK_POST_GRAB_AUDIT);
  separation_total_attempts=2;
  task_capture_observe(5100);
  assert(state == TASK_APPROACH_RECOVER && capture_escape_return_center);
  puts("two-frame cadence/STABLE, disabled escape, two separation attempts PASS");
  return 0;
#endif
  test_recoverable_wait();
  test_receipt_cadence_cannot_outrun_host();
  test_stable_does_not_skip_three_real_frames();
  test_same_frame_confirmation_has_no_new_sample_or_freshness();
  test_invalid_histogram_and_side_jitter();
  test_sweep_placeholder_ack_and_real_role_validation();
  test_injury_retrieve_placeholder_does_not_change_destination();
  test_placeholder_never_means_empty_or_resets_deadline();
  test_final_wait_keeps_snapshot_and_expires();
  test_zero_frame_stable_is_rejected();
  test_quantity_grip_and_retry_budget();
  test_ready_mode_reaudit_and_stale_grip_rejection();
  test_curve_mirrors_limits_and_pause();
  test_separation_retransmission_is_one_round();
  test_third_failure_recloses_and_reopens_at_center();
  puts("audit/separation handshake: 15 scenario groups PASS");
  return 0;
}
