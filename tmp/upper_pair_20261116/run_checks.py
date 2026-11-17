from pathlib import Path
import subprocess
r=Path.cwd()
p=r/'tools/test_audit_handshake.c'
original=p.read_bytes()
checks=r'''
static void paired_range_checks(void)
{
  for(unsigned red=0;red<2;red++) for(unsigned cluster=0;cluster<2;cluster++)
  for(unsigned pitch_i=0;pitch_i<2;pitch_i++) {
    const uint8_t pitch=pitch_i ? 120 : 105;
    wire_prepare(TASK_APPROACH,0); configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
    Camera_SetAngle(pitch); task_enter(TASK_APPROACH,0);
    cluster_target_active=cluster;cluster_claws_ready=true;locked_cargo_counts=1;
    VisionData v={.x=640,.y=512,.found=true,.classification_valid=true,.cargo_counts=1,
        .valid=true,.report_generation=9,.tick_ms=300};
    task_process_approach(&v,300);assert(approach_phase==APPROACH_RANGE_WAIT);
    VisionMissionCommand c={.command=VISION_CMD_APPROACH_TARGET,
      .flags=VISION_CMD_VALID|VISION_CMD_DISTANCE_VALID|VISION_CMD_APPROACH_RANGE|
        (red?VISION_CMD_RED_SIDE:0)|(cluster?VISION_CMD_CLUSTER_TARGET:0),
      .target_x_mm=460,.target_y_mm=pitch*100,.heading_cdeg=1,.vision_frame=10};
    wire_send(c,350,1);assert(command_accepted && approach_range_reobserve && approach_phase==APPROACH_RANGE_MOVE);
    distance_result=MOTOR_DISTANCE_RUNNING;distance_travelled_mm=459;
    task_process_approach(&v,370);assert(approach_phase==APPROACH_RANGE_MOVE && !accepted_context.completed);
    uint32_t deadline=approach_deadline_ms;
    wire_send(c,380,2);assert(command_accepted && approach_deadline_ms==deadline);
    VisionMissionCommand bad=c;bad.heading_cdeg=0;wire_send(bad,390,3);assert(!command_accepted);
    bad=c;bad.target_x_mm=461;wire_send(bad,400,4);assert(!command_accepted);
    bad=c;bad.target_y_mm=14000;wire_send(bad,410,5);assert(!command_accepted);
    bad=c;bad.heading_cdeg=2;wire_send(bad,420,6);assert(!command_accepted);
    wire_send_action(c,12,425,7);assert(!command_accepted);
    mission_paused=true;task_pause_runtime(430);assert(finite_pauses>0);
    mission_paused=false;distance_result=MOTOR_DISTANCE_DONE;
    task_process_approach(&v,450);
    assert(approach_phase==APPROACH_RANGE_WAIT && task_protocol_mode()==52);
    assert(accepted_context.completed && Camera_GetAngle()==pitch && !task_status.motors_active);
    unsigned calls=distance_calls;deadline=approach_deadline_ms;
    wire_send(c,500,8);assert(command_accepted && accepted_context.completed && approach_phase==APPROACH_RANGE_WAIT);
    task_process_approach(&v,510);assert(distance_calls==calls && approach_deadline_ms==deadline);
    VisionMissionCommand hold={.command=VISION_CMD_HOLD,.flags=VISION_CMD_VALID};
    wire_send(hold,520,9);assert(command_accepted && accepted_context.completed);
    wire_send(c,530,10);assert(command_accepted && accepted_context.completed && distance_calls==calls);
    bad=c;bad.heading_cdeg=0;wire_send(bad,540,11);assert(!command_accepted);
    c.vision_frame=11;c.target_x_mm=200;
    wire_send_action(c,12,550,12);assert(command_accepted && !accepted_context.completed && approach_range_mm==200);
    distance_result=MOTOR_DISTANCE_DONE;task_process_approach(&v,570);assert(accepted_context.completed && task_protocol_mode()==52);
    bad=c;bad.vision_frame=10;wire_send(bad,580,13);assert(!command_accepted && approach_range_mm==200);
    c.vision_frame=12;c.target_x_mm=100;c.heading_cdeg=0;
    wire_send_action(c,13,590,14);assert(command_accepted && !approach_range_reobserve && !accepted_context.completed);
    task_process_approach(&v,610);
    assert(approach_phase==(cluster?APPROACH_CLUSTER_CAMERA_TO_140:APPROACH_CAMERA_TO_140));
  }
  puts("range: red/blue x ordinary/cluster x 105/120, true DONE, repeat/HOLD, new segment, aux0, mismatches PASS");
}
static VisionMissionCommand transport_sample(uint8_t id,uint32_t frame,bool stable)
{
  VisionMissionCommand c=injury_audit(id);c.vision_frame=frame;
  c.audit_flags=stable?VISION_AUDIT_STABLE:0;
  c.audit_left_class=VISION_CARGO_GREEN;return c;
}
static void transport_begin(void)
{
  wire_prepare(TASK_TRANSPORT_AUDIT,0);initial_claw_ready=true;
  task_process_transport_audit(APP_GRAB_CAMERA_SETTLE_MS);
  assert(transport_audit_camera_ready && task_status.gripper_closed);
}
static void paired_audit_checks(void)
{
  transport_begin();VisionMissionCommand c=transport_sample(1,9,false);
  wire_send(c,400,1);assert(audit_consistent_count==1 && !audit_valid);
  wire_send(c,1200,2);assert(audit_consistent_count==1 && !audit_valid);
  test_vision=(VisionData){.config_ready=true,.mission=active_mission};
  Task_Process(1400);assert(audit_consistent_count==1);
  wire_send(placeholder(),1500,3);assert(!audit_valid && audit_consistent_count==1 && audit_observation_gap);
  task_nudge_load_camera(1600);assert(audit_consistent_count==1 && Camera_GetAngle()==138);
  c=transport_sample(2,10,true);wire_send(c,1650,4);assert(audit_consistent_count==1 && !audit_valid);
  task_process_transport_audit(1600+APP_POST_GRAB_CAMERA_NUDGE_MS);
  task_process_transport_audit(1600+APP_POST_GRAB_CAMERA_NUDGE_MS+APP_GRAB_CAMERA_SETTLE_MS);
  assert(audit_consistent_count==1 && transport_audit_camera_ready && Camera_GetAngle()==140);
  c=transport_sample(3,11,true);wire_send(c,3000,5);
  assert(audit_consistent_count==2 && audit_valid && task_context_complete());
  const uint32_t evidence=audit_evidence_ms;
  wire_send(c,5000,6);assert(audit_valid && audit_evidence_ms==evidence);
  test_vision=(VisionData){.config_ready=true,.mission=active_mission};
  Task_Process(5100);assert(audit_valid && state==TASK_TRANSPORT_AUDIT);
  wire_send(placeholder(),5200,7);
  assert(!audit_valid && !audit_received && !accepted_context.completed && audit_consistent_count==2);
  wire_send(c,5300,8);assert(!audit_valid && !accepted_context.completed);
  c=transport_sample(4,12,true);wire_send(c,5400,9);assert(audit_valid);
  c=transport_sample(5,13,true);c.audit_left_class=VISION_CARGO_CORE;
  wire_send(c,5500,10);assert(!audit_valid && !accepted_context.completed && audit_consistent_count==1);
  c.audit_id=6;c.vision_frame=14;wire_send(c,5700,11);assert(audit_valid);
  c.audit_id=7;c.vision_frame=15;c.audit_left_class=VISION_CARGO_DANGER;c.audit_flags|=VISION_AUDIT_DANGER_PRESENT;
  wire_send(c,5800,12);assert(!audit_valid && audit_consistent_count==1 && !task_retain_transport_samples());
  test_vision=(VisionData){.config_ready=true,.mission=active_mission};Task_Process(6500);assert(audit_consistent_count==0);
  transport_begin();c=transport_sample(1,9,false);wire_send(c,400,1);
  c=transport_sample(2,10,true);wire_send(c,450,2);assert(audit_consistent_count==1 && !audit_valid);
  wire_send(c,700,3);assert(audit_consistent_count==1 && !audit_valid);
  c=transport_sample(3,11,true);wire_send(c,800,4);assert(audit_valid);
  c.vision_frame=12;wire_send_action(c,12,1000,5);assert(!audit_valid && audit_consistent_count==1);
  task_enter(TASK_TRANSPORT_AUDIT,1200);assert(audit_consistent_count==0 && !audit_valid);
  transport_begin();c=transport_sample(1,9,false);wire_send(c,400,1);
  task_status.gripper_closed=false;assert(!task_retain_transport_samples());
  task_enter(TASK_STOPPED,500);assert(!audit_valid && audit_consistent_count==0);
  transport_begin();first_delivery_done=false;c=transport_sample(1,9,false);wire_send(c,400,1);
  VisionMissionCommand conflict=placeholder();conflict.audit_flags=VISION_AUDIT_FIRST_GREEN_CORE_CONFLICT;
  wire_send(conflict,500,2);assert(first_green_core_conflict && !task_retain_transport_samples() && !audit_valid);
  c=transport_sample(2,10,true);wire_send(c,600,3);assert(!audit_valid);
  prepare();command_visual_new=true;c=injury_audit(1);task_latch_audit(&c,100);
  c=injury_audit(2);task_latch_audit(&c,1000);assert(audit_consistent_count==1 && !audit_valid);
  transport_begin();c=transport_sample(1,9,false);c.audit_left_class=VISION_CARGO_CORE;
  c.audit_counts=2;c.audit_total_count=2;wire_send(c,400,1);
  c.audit_id=2;c.vision_frame=10;c.audit_flags=VISION_AUDIT_STABLE;wire_send(c,600,2);assert(audit_valid);
  c.audit_id=3;c.vision_frame=11;c.audit_left_class=VISION_CARGO_GREEN;c.audit_right_class=VISION_CARGO_CORE;c.audit_counts=5;
  wire_send(c,800,3);assert(!audit_valid && audit_consistent_count==1);
  c.audit_id=4;c.vision_frame=12;c.audit_left_class=VISION_CARGO_MIXED_MATERIAL;c.audit_right_class=VISION_CARGO_NONE;c.audit_counts=2;
  wire_send(c,1000,4);assert(audit_valid && audit_consistent_count==2);
  puts("audit: real sample across gaps/global expiry/nudge, duplicates/100ms, same-frame STABLE, placeholder revokes, new action/entry/STOP/conflict, mode23 unchanged PASS");
}
static void paired_camera_checks(void)
{
  for(unsigned red=0;red<2;red++) for(unsigned raised=0;raised<2;raised++) {
    wire_prepare(TASK_NAVIGATE,0);configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
    Camera_SetAngle(140);route_to_stash=false;delivery_stage_only=true;
    VisionMissionCommand nav={.received=true,.command=VISION_CMD_NAVIGATE_WAYPOINT,
      .flags=VISION_CMD_VALID|VISION_CMD_STAGE_ONLY|VISION_CMD_DISTANCE_VALID|
             VISION_CMD_DRIVE_STRAIGHT|VISION_CMD_USE_FINAL_HEADING|(red?VISION_CMD_RED_SIDE:0),
      .heading_cdeg=red?9000:27000};
    assert(nav_follow(&nav,nav.command,APP_NAV_FAST_SPEED_MM_S,10)==REMOTE_ROUTE_REACHED);
    assert(Camera_GetAngle()==140 && distance_command_done);
    assert(nav_follow(&nav,nav.command,APP_NAV_FAST_SPEED_MM_S,20)==REMOTE_ROUTE_REACHED && Camera_GetAngle()==140);
    task_enter(TASK_TRANSPORT_AUDIT,30);audit_received=audit_valid=true;task_status.gripper_closed=true;
    VisionMissionCommand align={.command=VISION_CMD_ALIGN_SAFE_ZONE,
      .flags=VISION_CMD_VALID|VISION_CMD_USE_FINAL_HEADING|(red?VISION_CMD_RED_SIDE:0),
      .heading_cdeg=red?9000:27000,.target_y_mm=raised?11500:0,.vision_frame=9};
    wire_send(align,50,1);assert(command_accepted && state==TASK_ALIGN_SAFE_ZONE && Camera_GetAngle()==(raised?115:120));
    wire_prepare(TASK_NAVIGATE,0);configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
    accepted_context.opcode=VISION_CMD_NAVIGATE_WAYPOINT;
    audit_received=audit_valid=audit_host_stable=true;audit_destination_injury=true;
    audit_last_semantic=AUDIT_SEMANTIC_INJURY_SINGLE;task_status.gripper_closed=true;
    nav_reuse_post_grab_audit=delivery_stage_only=true;route_to_stash=false;distance_command_done=true;
    Camera_SetAngle(140);wire_send(align,60,1);
    assert(command_accepted && state==TASK_ALIGN_SAFE_ZONE && Camera_GetAngle()==(raised?115:120));
  }
  puts("camera: repeated STAGE D0 keeps140, red/blue ALIGN120/115 PASS");
}
'''
try:
    s=original.decode('utf-8').replace('\r\n','\n')
    s=s.replace('int main(void)\n{',checks+'\nint main(void)\n{\n  paired_range_checks(); paired_audit_checks(); paired_camera_checks(); return 0;\n',1)
    p.write_text(s,encoding='utf-8',newline='\n')
    exe=r/'tmp/upper_pair_20261116/pair_checks.exe'
    args=[r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)]
    subprocess.run(args,check=True)
    subprocess.run([str(exe)],check=True)
finally:
    p.write_bytes(original)
