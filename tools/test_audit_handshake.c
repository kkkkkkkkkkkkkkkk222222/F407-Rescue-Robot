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

static void test_initial_empty_capture_and_heading(void)
{
  for (unsigned cluster = 0; cluster < 2; ++cluster) {
    prepare();
    task_enter(cluster ? TASK_APPROACH : TASK_GRAB_OBSERVE, 0U);
    if (cluster) { approach_phase = APPROACH_CLUSTER_CAPTURE; }
    Camera_SetAngle(APP_GRAB_VIEW_ANGLE);
    task_status.claw_visible = true;
    approach_locked_heading_deg = 90.0f;
    task_capture_budget_begin();
    VisionMissionCommand empty = {.received=true, .flags=VISION_CMD_VALID,
        .command=VISION_CMD_CARGO_AUDIT, .audit_flags=VISION_AUDIT_STABLE};
    for (unsigned id=1; id<=2; ++id) {
      empty.audit_id=id; empty.vision_frame=id;
      command_visual_new=true;
      task_accept_mission_legacy(&empty, id*100U);
    }
    assert(audit_received && !audit_valid);
    assert(!task_capture_observe(220U));
    assert(!capture_wait_active && capture_forward_used_mm == 0U);
    test_vision.mission=empty;
    if (cluster) { task_process_approach(&test_vision, 240U); }
    else { task_process_grab_observe(&test_vision, 240U); }
    assert(heading_calls == 1U && heading_output == 90.0f && reverse_output > 0.0f);
    task_audit_watchdog(240U);
    assert(!task_capture_observe(6000U)); /* Empty reports do not renew the total watchdog. */
    task_audit_watchdog(240U+APP_AUDIT_STATE_TIMEOUT_MS);
    assert(state == TASK_APPROACH_RECOVER && capture_escape_active);
  }
  prepare(); task_enter(TASK_GRAB_OBSERVE, 0U);
  task_capture_budget_begin();
  capture_forward_used_mm=APP_CAPTURE_FORWARD_BUDGET_MM;
  assert(task_capture_observe(100U) && capture_wait_active);
  task_capture_observe(100U+APP_CAPTURE_DECISION_WAIT_MS);
  assert(state == TASK_APPROACH_RECOVER);
  for (unsigned post=0; post<2; ++post) {
    prepare();
    if (!post) { task_enter(TASK_GRAB_OBSERVE, 0U); cargo_recheck_pending=true; }
    VisionMissionCommand empty = {.received=true, .flags=VISION_CMD_VALID,
        .command=VISION_CMD_CARGO_AUDIT, .audit_flags=VISION_AUDIT_STABLE};
    for (unsigned id=1; id<=2; ++id) {
      empty.audit_id=id; empty.vision_frame=id;
      command_visual_new=true; task_accept_mission_legacy(&empty,id*100U);
    }
    assert(state == TASK_OPEN_CLAW && reverse_output == 0.0f);
  }
  prepare(); task_enter(TASK_GRAB_OBSERVE,0U); task_capture_budget_begin();
  audit_at(1,100); audit_at(2,200);
  assert(task_capture_observe(220U) && reverse_output == 0.0f);
  prepare();
  assert(task_move_heading(-150.0f,270.0f,100U));
  assert(reverse_output == -150.0f && heading_output == 270.0f);
  heading_rejected=true;
  assert(!task_move_heading(200.0f,90.0f,120U));
  assert(state == TASK_RECOVERY_WAIT && task_status.fault == TASK_FAULT_NONE);
  assert(!task_status.motors_active && reverse_output == 0.0f);
  prepare();
  task_enter(TASK_INJURY_BACKOFF, 0U);
  tick(0U); tick(APP_INJURY_CLAW_SETTLE_MS);
  assert(reverse_output == -APP_INJURY_BACKOFF_SPEED_MM_S && fabsf(heading_output - 90.0f) < 0.01f);
  tick(APP_INJURY_CLAW_SETTLE_MS + APP_INJURY_BACKOFF_TIME_MS);
  assert(reverse_output == 0.0f && injury_backoff_phase == 2U);
  tick(APP_INJURY_CLAW_SETTLE_MS + APP_INJURY_BACKOFF_TIME_MS + APP_GRAB_CAMERA_SETTLE_MS);
  assert(state == TASK_POST_GRAB_AUDIT && injury_recheck_pending);
  audit_at(1, 3000U); audit_at(2, 3100U);
  assert(state == TASK_INJURY_RECLOSE);
}

static void test_distance_search_accumulation_and_transport_sort(void)
{
  assert(APP_SEARCH_LOW_CAMERA_ANGLE == 105U && APP_CAPTURE_FORWARD_BUDGET_MM == 200U);
  prepare(); task_enter(TASK_APPROACH,1000U);locked_cargo_counts=1U;
  VisionData stale={.x=640,.y=512,.found=true,.classification_valid=true,
      .cargo_counts=1,.valid=true,.report_generation=1,.tick_ms=900};
  task_process_approach(&stale,1300U);
  assert(approach_phase==APPROACH_TRACK && !tracking_filter_valid);
  for (unsigned cluster=0; cluster<2; ++cluster) {
    wire_prepare(TASK_APPROACH,0U);
    Camera_SetAngle(cluster ? 120U : 105U); task_enter(TASK_APPROACH,0U);
    cluster_target_active=cluster; cluster_claws_ready=true; locked_cargo_counts=1U;
    VisionData v={.x=640,.y=512,.found=true,.classification_valid=true,
        .cargo_counts=1,.valid=true,.report_generation=9,.tick_ms=300};
    task_process_approach(&v,300);
    assert(approach_phase == APPROACH_RANGE_WAIT && task_protocol_mode() == 52);
    const uint8_t pitch=Camera_GetAngle();
    VisionMissionCommand range={.command=VISION_CMD_APPROACH_TARGET,
        .flags=VISION_CMD_VALID|VISION_CMD_DISTANCE_VALID|VISION_CMD_APPROACH_RANGE|
               (cluster ? VISION_CMD_CLUSTER_TARGET : 0),
        .target_x_mm=700,.target_y_mm=pitch*100,.vision_frame=10};
    wire_send(range,350,1);
    assert(command_accepted && approach_phase == APPROACH_RANGE_MOVE && task_protocol_mode() == 53);
    const uint32_t deadline=approach_deadline_ms;
    wire_send(range,380,2);
    assert(command_accepted && approach_deadline_ms == deadline);
    range.target_x_mm=900; wire_send(range,400,3);
    assert(!command_accepted && approach_range_mm == 700);
    VisionMissionCommand old_pixel={.command=VISION_CMD_APPROACH_TARGET,.flags=VISION_CMD_VALID,
        .target_x_mm=640,.target_y_mm=512,.vision_frame=11};
    wire_send(old_pixel,410,4);
    assert(!command_accepted && approach_range_mm==700 && approach_phase==APPROACH_RANGE_MOVE);
    distance_result=MOTOR_DISTANCE_RUNNING;
    task_process_approach(&v,420);
    assert(distance_calls == 1 && fabsf(distance_request-.7f)<.001f);
    mission_paused=true; task_pause_runtime(450);
    assert(finite_pauses == 1 && approach_range_mm == 700);
    mission_paused=false;
    task_process_approach(&v,480);
    assert(distance_calls == 2 && finite_resumes >= 2);
    distance_result=MOTOR_DISTANCE_DONE;
    task_process_approach(&v,500);
    assert(approach_phase == (cluster ? APPROACH_CLUSTER_CAMERA_TO_140 : APPROACH_CAMERA_TO_140));
    for (uint32_t t=520;t<=1600;t+=20) task_process_approach(&v,t);
    assert(Camera_GetAngle()==140 && capture_budget_active);
    assert(cluster ? task_protocol_mode()==38 : state==TASK_GRAB_OBSERVE);
  }
  prepare(); task_enter(TASK_APPROACH,0U);
  task_approach_watchdog(APP_APPROACH_ALIGN_TIMEOUT_MS);
  assert(state==TASK_APPROACH_RECOVER && task_status.fault==TASK_FAULT_NONE);
  prepare(); task_enter(TASK_APPROACH,0U); approach_phase=APPROACH_RANGE_WAIT;
  task_approach_watchdog(APP_APPROACH_RANGE_WAIT_MS);
  assert(state==TASK_APPROACH_RECOVER);
  prepare();task_enter(TASK_APPROACH,0U);approach_phase=APPROACH_RANGE_MOVE;
  accepted_context.last_command_rx_ms=APP_APPROACH_RANGE_TIMEOUT_MS;
  task_approach_watchdog(APP_APPROACH_RANGE_TIMEOUT_MS);
  assert(state==TASK_APPROACH_RECOVER); /* Fresh heartbeats cannot extend the travel deadline. */
  prepare(); task_enter(TASK_APPROACH,0U); approach_phase=APPROACH_RANGE_MOVE;
  accepted_context.last_command_rx_ms=0;
  task_process_approach(&test_vision,600);
  assert(finite_pauses==1 && distance_calls==0);
  task_approach_watchdog(APP_APPROACH_LINK_RECOVER_MS);
  assert(state==TASK_APPROACH_RECOVER);

  prepare(); injury_recheck_pending=true;
  VisionMissionCommand c=injury_audit(1);
  command_visual_new=true; task_latch_audit(&c,100);
  const uint32_t evidence=audit_evidence_ms;
  VisionMissionCommand gap=placeholder(); command_visual_new=false;
  task_accept_mission_legacy(&gap,180);
  assert(audit_consistent_count==1 && !audit_received && audit_evidence_ms==evidence);
  c.audit_id=2;c.vision_frame=3;command_visual_new=true;task_latch_audit(&c,250);
  assert(audit_valid && audit_consistent_count==2);
  task_accept_mission_legacy(&gap,270);
  command_visual_new=false; accepted_context.visual_valid=true;accepted_context.vision_frame=3;
  task_latch_audit(&c,290); assert(!audit_received); /* Cached STABLE cannot bridge the gap. */
  c.audit_id=3;c.vision_frame=4;command_visual_new=true;task_latch_audit(&c,310);
  assert(audit_valid);
  c.audit_id=4;c.audit_total_count=2;c.audit_counts=2;task_latch_audit(&c,350);
  assert(audit_consistent_count==1 && !audit_valid);
  prepare();c=injury_audit(1);command_visual_new=true;task_latch_audit(&c,100);
  c.audit_id=2;c.vision_frame=3;task_latch_audit(&c,601);
  assert(audit_consistent_count==1);

  prepare();task_enter(TASK_GRAB_OBSERVE,0);task_capture_budget_begin();
  capture_forward_used_mm=200; audit_received=true;
  task_capture_observe(300);
  assert(state==TASK_SEARCH && !capture_budget_active);

  prepare();task_enter(TASK_TRANSPORT_AUDIT,0);task_process_transport_audit(250);
  c=injury_audit(1);c.audit_left_class=VISION_CARGO_GREEN;c.audit_flags=VISION_AUDIT_STABLE;
  c.audit_counts=c.audit_total_count=2;
  command_visual_new=true;task_latch_audit(&c,300);
  c.audit_id=2;c.vision_frame=3;task_latch_audit(&c,400);
  VisionMissionCommand split={.received=true,.command=VISION_CMD_DISPERSE_PILE,
      .flags=VISION_CMD_VALID|VISION_CMD_SIDE_VALID};
  task_accept_mission_legacy(&split,420);
  assert(command_accepted && state==TASK_REMOTE_ACTION && transport_sort_active);
  assert(separation_total_attempts==1 && separation_keep_side==1 && !task_status.gripper_closed);
  remote_action.done=true;cargo_recheck_pending=true;task_status.claw_visible=true;
  c=injury_audit(3);c.audit_left_class=VISION_CARGO_GREEN;c.audit_flags=VISION_AUDIT_STABLE;
  command_visual_new=true;task_accept_mission_legacy(&c,600);
  c.audit_id=4;c.vision_frame=5;task_accept_mission_legacy(&c,700);
  assert(state==TASK_GRAB_OBSERVE && audit_valid);
  VisionMissionCommand grab={.received=true,.command=VISION_CMD_GRAB_CONFIRMED,.flags=VISION_CMD_VALID};
  task_accept_mission_legacy(&grab,720);
  assert(command_accepted && state==TASK_CLOSE_CLAW);
  initial_claw_ready=true;
  Task_Process(740);Task_Process(2740);
  assert(state==TASK_POST_GRAB_AUDIT);
  c.audit_id=5;c.vision_frame=6;command_visual_new=true;task_accept_mission_legacy(&c,2800);
  c.audit_id=6;c.vision_frame=7;task_accept_mission_legacy(&c,2900);
  assert(state==TASK_WAIT_NAVIGATION && task_status.gripper_closed);
}

static void test_new_curve_targets_and_bounded_exit(void)
{
  for (int side=-1;side<=1;side+=2) for(unsigned retry=1;retry<=2;++retry) {
    prepare(); separation_keep_side=side;separation_total_attempts=retry;
    task_start_remote_action(REMOTE_ACTION_DISPERSE,&(VisionMissionCommand){.command=VISION_CMD_DISPERSE_PILE},0);
    remote_action.phase=1;
    assert(!task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,0));
    assert(separation_motion.stage==SEPARATION_PREPULL);
    assert(!task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,20));
    assert(reverse_output<0 && lateral_output==0 && yaw_output==0);
    test_pose.path_mm=60;test_pose.y_mm=-60;
    assert(!task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,200));
    assert(side*yaw_output*APP_OMNI_FIELD_YAW_TO_DRIVE_SIGN<0 && lateral_output==0);
    const float wanted=retry==1?30.0f:45.0f;
    assert(fabsf(task_wrap_angle(separation_motion.target_heading_deg-90.0f)+side*wanted)<.01f);
    test_pose.path_mm=150;test_pose.heading_mdeg=90000-side*(int)(wanted*1000);
    for(unsigned t=220;t<700;t+=20) (void)task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,t);
    assert(separation_motion.stage==SEPARATION_PULL);
    test_pose.path_mm=220;
    assert(task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,720));
    assert(remote_action.phase==2 && reverse_output==0);
  }
  prepare();separation_keep_side=1;separation_total_attempts=1;
  remote_action.phase=1;
  task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,0);
  assert(task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,APP_CARGO_SEPARATE_NO_PROGRESS_MS));
  assert(remote_action.phase==2);
  prepare();separation_keep_side=1;separation_total_attempts=1;
  task_start_remote_action(REMOTE_ACTION_DISPERSE,&(VisionMissionCommand){.command=VISION_CMD_DISPERSE_PILE},0);
  remote_action.phase=1;
  task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,0);
  task_pause_action(100,false);
  VisionMissionCommand repeat={.received=true,.command=VISION_CMD_DISPERSE_PILE,.flags=VISION_CMD_VALID};
  task_process_remote_action(&repeat,5100);
  assert(separation_motion.active && separation_motion.started_ms==5000 && remote_action.phase==1);
  assert(separation_total_attempts==1);
  for(int side=-1;side<=1;side+=2) for(unsigned retry=1;retry<=2;++retry) {
    prepare(); separation_keep_side=side;separation_total_attempts=retry;remote_action.phase=1;
    float x=0,y=0,heading=90,path=0;
    bool done=false;
    for(unsigned t=0;t<=8000 && !done;t+=20) {
      test_pose.x_mm=(int32_t)lroundf(x);test_pose.y_mm=(int32_t)lroundf(y);
      test_pose.heading_mdeg=(int32_t)lroundf(heading*1000);test_pose.path_mm=(uint32_t)path;
      done=task_remote_separation_curve(APP_SELECTIVE_DISPERSE_CURVE_MM,t);
      const float rad=heading*3.14159265f/180;
      x+=reverse_output*cosf(rad)*.02f;y+=reverse_output*sinf(rad)*.02f;
      path+=fabsf(reverse_output)*.02f;
      heading+=yaw_output*APP_OMNI_FIELD_YAW_TO_DRIVE_SIGN/APP_MOTION_DEBUG_TURN_RADIUS_MM*.02f*180/3.14159265f;
    }
    assert(done && reverse_output==0 && path<=APP_SELECTIVE_DISPERSE_CURVE_MM+5);
    assert(side*(90-heading)>=(retry==1?26.0f:41.0f));
    assert(fabsf(x)<=APP_CARGO_SEPARATE_SIDE_BUDGET_MM+5);
  }
}

int main(void)
{
#ifdef FLOW_20261019_CHECK
  test_distance_search_accumulation_and_transport_sort();
  test_new_curve_targets_and_bounded_exit();
  puts("105/120 range handoff, cumulative audits, mode48 sort/resume, mirrored 30/45 curves and finite exits PASS");
  return 0;
#endif
#ifdef CAPTURE_HEADING_REVIEW_CHECK
  test_initial_empty_capture_and_heading();
  puts("initial empty ordinary/cluster crawl, budgets, nonempty stop, loss and heading recovery PASS");
  return 0;
#endif
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
  assert(APP_SEPARATION_MAX_ATTEMPTS == 2);
  command_accepted=false;
  VisionMissionCommand retired={.received=true,.flags=VISION_CMD_VALID,.command=VISION_CMD_ESCAPE_MANEUVER};
  task_accept_mission_legacy(&retired,300);
  assert(!command_accepted);
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
