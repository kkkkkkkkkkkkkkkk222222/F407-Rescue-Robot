#define main injury_sequence_main
#include "test_injury_backoff.c"
#undef main

static void test_fast_frames_and_gap(void)
{
  prepare();
  injury_recheck_pending = true;
  audit_at(1U, 0U); audit_at(2U, 10U); audit_at(3U, 20U);
  assert(!audit_received && audit_consistent_count == 1U);
  audit_at(4U, 100U); assert(audit_consistent_count == 2U);
  audit_at(5U, 200U); assert(state == TASK_INJURY_RECLOSE);
  prepare();
  audit_at(1U, 0U); audit_at(2U, 100U); audit_at(3U, 601U);
  assert(!audit_received && audit_consistent_count == 1U);
}

static void test_class_and_danger_changes_reset_streak(void)
{
  prepare();
  for (uint8_t id = 1U; id <= 2U; ++id) {
    VisionMissionCommand c = injury_audit(id);
    c.audit_flags = 0U; c.audit_left_class = VISION_CARGO_GREEN;
    command_visual_new = true; task_latch_audit(&c, id * 100U);
  }
  VisionMissionCommand c = injury_audit(3U);
  c.audit_flags = 0U; c.audit_left_class = VISION_CARGO_CORE;
  command_visual_new = true; task_latch_audit(&c, 300U);
  assert(audit_consistent_count == 1U && !audit_valid);
  c.audit_id = 4U; c.audit_flags = VISION_AUDIT_DANGER_PRESENT;
  task_latch_audit(&c, 310U);
  assert(audit_consistent_count == 1U && !audit_valid);
}

static void test_transport_retries_and_fixed_deadline(void)
{
  prepare(); task_enter(TASK_TRANSPORT_AUDIT, 0U);
  task_audit_watchdog(0U);
  task_process_transport_audit(500U);
  assert(transport_audit_camera_ready && task_status.claw_visible);
  task_process_transport_audit(4000U);
  assert(Camera_GetAngle() == 138U && !task_status.claw_visible);
  task_audit_watchdog(4000U);
  task_process_transport_audit(4400U); assert(Camera_GetAngle() == 140U);
  task_process_transport_audit(4900U); assert(task_status.claw_visible);
  task_process_transport_audit(8400U); assert(Camera_GetAngle() == 142U);
  task_process_transport_audit(8800U); task_process_transport_audit(9300U);
  task_audit_watchdog(14999U); assert(state == TASK_TRANSPORT_AUDIT);
  task_clear_audit_result();
  task_audit_watchdog(15000U);
  assert(state == TASK_APPROACH_RECOVER && capture_escape_active && capture_escape_return_center);
  assert(!audit_received && !audit_valid);
}

static void test_third_separation_no_evidence_cannot_wait_forever(void)
{
  prepare(); task_enter(TASK_GRAB_OBSERVE, 0U);
  separation_total_attempts = APP_SEPARATION_MAX_ATTEMPTS;
  cargo_recheck_pending = true;
  task_audit_watchdog(0U); task_audit_watchdog(15000U);
  assert(state == TASK_APPROACH_RECOVER && capture_escape_return_center);
}

static void test_post_nudge_settles_before_collecting(void)
{
  prepare();
  task_process_post_grab_audit(3500U);
  assert(Camera_GetAngle() == 138U && !task_status.claw_visible);
  task_process_post_grab_audit(3900U);
  assert(Camera_GetAngle() == 140U && !task_status.claw_visible);
  audit_at(1U, 3910U);
  assert(audit_consistent_count == 0U);
  task_process_post_grab_audit(4399U); assert(!task_status.claw_visible);
  task_process_post_grab_audit(4400U); assert(task_status.claw_visible);
}

int main(void)
{
  test_fast_frames_and_gap();
  test_class_and_danger_changes_reset_streak();
  test_transport_retries_and_fixed_deadline();
  test_third_separation_no_evidence_cannot_wait_forever();
  test_post_nudge_settles_before_collecting();
  puts("audit stability: 5 scenarios PASS");
  return 0;
}
