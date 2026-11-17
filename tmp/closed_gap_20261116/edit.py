from pathlib import Path
r=Path.cwd()
def update(path,changes):
 p=r/path;s=p.read_text(encoding='utf-8')
 for old,new in changes:
  assert s.count(old)==1,(path,old[:90],s.count(old))
  s=s.replace(old,new)
 p.write_text(s,encoding='utf-8',newline='\n')
update('Main/Inc/vision.h',[
('#define VISION_CMD_REUSE_POST_GRAB_AUDIT 0x20U','#define VISION_CMD_REUSE_POST_GRAB_AUDIT 0x20U\n/* Pose ALIGN / STAGE NAV: reuse a genuinely confirmed closed load, not a new audit. */\n#define VISION_CMD_REUSE_CONFIRMED_LOAD 0x20U')])
update('Main/Src/vision.c',[
('((code == VISION_CMD_NAVIGATE_WAYPOINT) ? VISION_CMD_REUSE_POST_GRAB_AUDIT : 0U)',
 '(((code == VISION_CMD_NAVIGATE_WAYPOINT) || (code == VISION_CMD_ALIGN_SAFE_ZONE)) ?\n          VISION_CMD_REUSE_CONFIRMED_LOAD : 0U)'),
('  if (code == VISION_CMD_ALIGN_SAFE_ZONE) {\n','''  if (code == VISION_CMD_ALIGN_SAFE_ZONE) {
    if ((flags & VISION_CMD_REUSE_CONFIRMED_LOAD) != 0U) {
      const uint8_t permitted = VISION_CMD_VALID | VISION_CMD_RED_SIDE |
          VISION_CMD_USE_FINAL_HEADING | VISION_CMD_REUSE_CONFIRMED_LOAD;
      if ((flags & (uint8_t)~permitted) != 0U ||
          heading != ((flags & VISION_CMD_RED_SIDE) != 0U ? 9000U : 27000U)) {
        goto rejected_payload;
      }
    }
''')])
update('tools/vision_protocol.py',[
('CMD_CLUSTER_TARGET = 0x20','CMD_CLUSTER_TARGET = 0x20\nCMD_REUSE_POST_GRAB_AUDIT = 0x20\nCMD_REUSE_CONFIRMED_LOAD = 0x20'),
('    if command == CMD_DISPERSE_PILE:\n        allowed_flags |= (','    if command in (CMD_NAVIGATE_WAYPOINT, CMD_ALIGN_SAFE_ZONE):\n        allowed_flags |= CMD_REUSE_CONFIRMED_LOAD\n    if command == CMD_DISPERSE_PILE:\n        allowed_flags |= ('),
('    if command == CMD_ALIGN_SAFE_ZONE:\n        if target_y_mm', '''    if command == CMD_NAVIGATE_WAYPOINT and flags & CMD_REUSE_CONFIRMED_LOAD:
        direction = flags & (CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING)
        if (not flags & CMD_STAGE_ONLY or not flags & CMD_DISTANCE_VALID or
                direction not in (0, CMD_DRIVE_STRAIGHT | CMD_USE_FINAL_HEADING) or
                target_x_mm < 0 or target_y_mm != 0):
            raise ValueError("confirmed load reuse requires STAGE NAV")
    if command == CMD_ALIGN_SAFE_ZONE:
        if flags & CMD_REUSE_CONFIRMED_LOAD:
            permitted = CMD_VALID | CMD_RED_SIDE | CMD_USE_FINAL_HEADING | CMD_REUSE_CONFIRMED_LOAD
            if (flags & ~permitted or
                    heading_cdeg != (9000 if flags & CMD_RED_SIDE else 27000)):
                raise ValueError("confirmed load reuse requires own-zone pose ALIGN")
        if target_y_mm''')])
update('Main/Src/Task.c',[
('static bool nav_reuse_post_grab_audit;','''static bool nav_reuse_post_grab_audit;
/* Prior real closed-load permission is separate from the current audit.
 * No source frame, sample, READY or VALID is synthesized when it is reused. */
static struct {
  bool valid, active, injury;
  uint8_t total;
  uint8_t classes[VISION_CARGO_MIXED_MATERIAL + 1U];
} confirmed_load;'''),
('static void task_note_first_green_core(const VisionMissionCommand *command)\n{', '''static void task_clear_confirmed_load(void)
{
  memset(&confirmed_load, 0, sizeof(confirmed_load));
}

static bool task_confirmed_load_available(void)
{
  return confirmed_load.valid && task_status.gripper_closed &&
      !(first_green_core_conflict && !first_delivery_done);
}

static void task_note_first_green_core(const VisionMissionCommand *command)
{'''),
('  first_green_core_conflict = true;\n  audit_valid', '  first_green_core_conflict = true;\n  task_clear_confirmed_load();\n  audit_valid'),
('static TaskCommandReject task_disperse_reject_reason(','''static void task_normalize_load_classes(uint8_t *counts)
{
  const uint8_t mixed = counts[VISION_CARGO_MIXED_MATERIAL] / 2U;
  counts[VISION_CARGO_GREEN] += mixed;
  counts[VISION_CARGO_CORE] += mixed;
  counts[VISION_CARGO_MIXED_MATERIAL] = 0U;
}

static void task_remember_confirmed_load(void)
{
  if (!task_status.gripper_closed || !audit_received || !audit_valid ||
      !audit_host_stable || audit_initial_stash || audit_last_total_count == 0U ||
      (first_green_core_conflict && !first_delivery_done)) { return; }
  confirmed_load.valid = true;
  confirmed_load.active = false;
  confirmed_load.injury = audit_destination_injury;
  confirmed_load.total = audit_last_total_count;
  memcpy(confirmed_load.classes, audit_last_class_counts, sizeof(confirmed_load.classes));
  task_normalize_load_classes(confirmed_load.classes);
}

static void task_observe_confirmed_load(const VisionMissionCommand *command)
{
  if (!confirmed_load.valid || !command_visual_new || command->vision_frame == 0U ||
      task_audit_is_empty(command)) { return; } /* A missing box is not a changed load. */
  uint8_t classes[VISION_CARGO_MIXED_MATERIAL + 1U];
  const uint8_t zero_classes = task_audit_class_counts(command, classes);
  task_normalize_load_classes(classes);
  if (zero_classes != 0U || task_audit_semantic(command) == AUDIT_SEMANTIC_INVALID ||
      command->audit_total_count != confirmed_load.total ||
      ((command->audit_flags & VISION_AUDIT_DESTINATION_INJURY) != 0U) != confirmed_load.injury ||
      (command->audit_flags & VISION_AUDIT_INITIAL_STASH) != 0U ||
      memcmp(classes, confirmed_load.classes, sizeof(classes)) != 0) {
    task_clear_confirmed_load();
  }
}

static TaskCommandReject task_disperse_reject_reason('''),
('  if (!command_visual_new) {\n    /* A same-picture STABLE', '  task_observe_confirmed_load(command);\n  if (!command_visual_new) {\n    /* A same-picture STABLE'),
('      task_status.audit_valid = audit_valid;\n    }\n    TaskDiag_Event("AUDIT_SAME_FRAME"', '      task_status.audit_valid = audit_valid;\n      task_remember_confirmed_load();\n    }\n    TaskDiag_Event("AUDIT_SAME_FRAME"'),
('  task_status.audit_valid = audit_valid;\n  TaskDiag_Event("AUDIT_RESULT"', '  task_status.audit_valid = audit_valid;\n  task_remember_confirmed_load();\n  TaskDiag_Event("AUDIT_RESULT"'),
('static void task_enter(TaskState next, uint32_t now_ms)\n{', '''static void task_enter(TaskState next, uint32_t now_ms)
{
  if (next == TASK_WAIT_NAVIGATION) { task_remember_confirmed_load(); }
  if (next == TASK_SEARCH || next == TASK_STOPPED || next == TASK_RECOVERY_WAIT ||
      next == TASK_OPEN_CLAW || next == TASK_OPEN_FOR_RAM || next == TASK_SAFE_SWEEP ||
      next == TASK_APPROACH || next == TASK_APPROACH_RECOVER ||
      next == TASK_INJURY_BACKOFF || next == TASK_BOUNDARY_RECOVER ||
      next == TASK_REMOTE_ACTION) {
    task_clear_confirmed_load();
  }
  if (next == TASK_TRANSPORT_AUDIT) { confirmed_load.active = false; }'''),
('static void task_initialize(uint32_t now_ms)\n{', 'static void task_initialize(uint32_t now_ms)\n{\n  task_clear_confirmed_load();'),
('    case POST_AUDIT_WAIT_PRIMARY:\n      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);\n      if (', '    case POST_AUDIT_WAIT_PRIMARY:\n      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);\n      if (!empty_load_reobserved && '),
('    case POST_AUDIT_WAIT_LOW:\n      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);\n      if (', '    case POST_AUDIT_WAIT_LOW:\n      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);\n      if (!empty_load_reobserved && '),
('    case POST_AUDIT_WAIT_HIGH:\n      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);\n      if (', '    case POST_AUDIT_WAIT_HIGH:\n      Camera_SetAngle(APP_GRAB_VIEW_ANGLE);\n      if (!empty_load_reobserved && '),
('  } else if (transport_audit_attempt < 2U &&', '  } else if (!empty_load_reobserved && transport_audit_attempt < 2U &&'),
('static bool task_dynamic_opcode(uint8_t opcode)', '''static bool task_capture_cancel_state(void)
{
  return (!task_status.gripper_closed &&
          (state == TASK_APPROACH || state == TASK_GRAB_OBSERVE || state == TASK_DISPERSE_READY)) ||
      (task_status.gripper_closed &&
          (state == TASK_POST_GRAB_AUDIT || state == TASK_TRANSPORT_AUDIT));
}

static bool task_capture_cancel_command(const VisionMissionCommand *command)
{
  return command->command == VISION_CMD_RELEASE_BOTH &&
      command->flags == (VISION_CMD_VALID | (command->flags & VISION_CMD_RED_SIDE)) &&
      task_side_flag_valid(command->flags) && command->vision_frame == 0U &&
      command->target_x_mm == 0 && command->target_y_mm == 0 && command->heading_cdeg == 0U;
}

static bool task_dynamic_opcode(uint8_t opcode)'''),
('    context_session_generation = vision->session_generation;', '    context_session_generation = vision->session_generation;\n    task_clear_confirmed_load();'),
('  if (state == TASK_ALIGN_SAFE_ZONE && safe_align_done) {', '  if (!task_status.gripper_closed) { task_clear_confirmed_load(); }\n  if (state == TASK_ALIGN_SAFE_ZONE && safe_align_done) {'),
('  const bool new_action = new_task || incoming.action_id != accepted_context.action_id;', '''  const bool new_action = new_task || incoming.action_id != accepted_context.action_id;
  const bool cancel_capture = task_capture_cancel_command(&incoming) && task_capture_cancel_state();
  const bool cancel_replay = !new_action && task_capture_cancel_command(&incoming) &&
      accepted_context.opcode == VISION_CMD_RELEASE_BOTH &&
      ((state == TASK_APPROACH_RECOVER && capture_escape_active) || state == TASK_SEARCH);
  if (cancel_capture && (new_target || !new_action)) goto rejected;
  const bool reuse_load = (incoming.command == VISION_CMD_NAVIGATE_WAYPOINT ||
      incoming.command == VISION_CMD_ALIGN_SAFE_ZONE) &&
      (incoming.flags & VISION_CMD_REUSE_CONFIRMED_LOAD) != 0U;
  if (reuse_load && (new_target || incoming.vision_frame != 0U)) goto rejected;
  if (incoming.command == VISION_CMD_ALIGN_SAFE_ZONE && reuse_load &&
      !(task_confirmed_load_available() &&
        (state == TASK_TRANSPORT_AUDIT ||
         (confirmed_load.active && (state == TASK_ALIGN_SAFE_ZONE ||
          (state == TASK_NAVIGATE && delivery_stage_only && !route_to_stash && distance_command_done)))))) {
    goto rejected;
  }'''),
('    if (!new_target && accepted_context.cancelled && !overlay &&','    if (!new_target && accepted_context.cancelled && !overlay && !cancel_replay &&'),
('  if (pre_grab.moving && !overlay && incoming.command != VISION_CMD_STOP &&\n      !pre_grab_request) goto rejected;', '  if (pre_grab.moving && !overlay && incoming.command != VISION_CMD_STOP &&\n      !pre_grab_request && !cancel_capture) goto rejected;'),
('  } else if (!new_action && !overlay && !task_dynamic_opcode(incoming.command)', '  } else if (cancel_capture) {\n    task_status.acknowledged_sequence = incoming.sequence;\n    command_accepted = true;\n    task_begin_capture_escape(now_ms);\n    mission_paused = false;\n  } else if (!new_action && !overlay && !task_dynamic_opcode(incoming.command)'),
('  if (new_target) {\n    memset(&delivery, 0, sizeof(delivery));','  if (new_target) {\n    task_clear_confirmed_load();\n    memset(&delivery, 0, sizeof(delivery));'),
('             audit_received && audit_valid && !audit_initial_stash &&\n             task_stage_command(command)', '''             ((audit_received && audit_valid && !audit_initial_stash) ||
              ((command->flags & VISION_CMD_REUSE_CONFIRMED_LOAD) != 0U &&
               task_confirmed_load_available())) &&
             task_stage_command(command)'''),
('    delivery_stage_only = true;\n  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT)', '''    delivery_stage_only = true;
    if ((command->flags & VISION_CMD_REUSE_CONFIRMED_LOAD) != 0U &&
        task_confirmed_load_available()) {
      confirmed_load.active = true;
      audit_destination_injury = transport_stage_injury = confirmed_load.injury;
    }
  } else if ((command->command == VISION_CMD_NAVIGATE_WAYPOINT)'''),
('    /* Withdrawal is final for this NAV. Re-adding the bit cannot manufacture', '''    if ((command->flags & VISION_CMD_REUSE_CONFIRMED_LOAD) == 0U) {
      confirmed_load.active = false;
    }
    /* Withdrawal is final for this NAV. Re-adding the bit cannot manufacture'''),
('  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&\n             state == TASK_NAVIGATE && delivery_stage_only', '''  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             (command->flags & VISION_CMD_REUSE_CONFIRMED_LOAD) != 0U &&
             task_confirmed_load_available() &&
             (state == TASK_TRANSPORT_AUDIT ||
              (state == TASK_NAVIGATE && delivery_stage_only && !route_to_stash &&
               distance_command_done && confirmed_load.active)) &&
             task_align_command_valid(command, false)) {
    task_status.acknowledged_sequence = command->sequence;
    command_accepted = true;
    confirmed_load.active = true;
    audit_destination_injury = transport_stage_injury = confirmed_load.injury;
    task_begin_safe_align(now_ms);
  } else if ((command->command == VISION_CMD_ALIGN_SAFE_ZONE) &&
             state == TASK_NAVIGATE && delivery_stage_only''')
])

