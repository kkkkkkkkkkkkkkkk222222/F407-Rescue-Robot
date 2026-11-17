from pathlib import Path
import subprocess
r=Path.cwd();p=r/'tools/test_audit_handshake.c';m=r/'tools/test_injury_backoff.c'
original=p.read_bytes();m_original=m.read_bytes()
checks=r'''
static VisionMissionCommand pg_prepare(unsigned entry, bool red)
{
  wire_prepare(entry==21?TASK_GRAB_OBSERVE:entry==37?TASK_DISPERSE_READY:TASK_APPROACH,0);
  configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
  initial_claw_ready=true;task_status.gripper_closed=false;task_status.claw_visible=true;
  cluster_target_active=entry!=21;opening_claw_probe_active=cargo_recheck_pending=false;
  if(entry==38)approach_phase=APPROACH_CLUSTER_CAPTURE;
  Camera_SetAngle(140);task_capture_budget_begin();
  audit_received=audit_valid=audit_host_stable=true;audit_consistent_count=2;
  audit_last_total_count=1;audit_last_counts=1;audit_last_left_class=VISION_CARGO_GREEN;
  distance_travelled_mm=0;distance_result=MOTOR_DISTANCE_RUNNING;
  return (VisionMissionCommand){.command=VISION_CMD_APPROACH_TARGET,
    .flags=VISION_CMD_VALID|VISION_CMD_DISTANCE_VALID|VISION_CMD_APPROACH_RANGE|
      (red?VISION_CMD_RED_SIDE:0)|(entry!=21?VISION_CMD_CLUSTER_TARGET:0),
    .target_x_mm=50,.target_y_mm=14000,.heading_cdeg=2,.vision_frame=9};
}
static void pg_tick(uint32_t now)
{
  test_vision=(VisionData){.config_ready=true,.mission=active_mission};
  Task_Process(now);
}
static void pg_runs(void)
{
  const unsigned entries[]={21,38,37};
  for(unsigned e=0;e<3;e++)for(unsigned red=0;red<2;red++) {
    unsigned entry=entries[e];VisionMissionCommand c=pg_prepare(entry,red);
    const uint8_t left=Servo_GetAngle(4),right=Servo_GetAngle(2);
    capture_forward_used_mm=40;capture_wait_active=true;capture_wait_started_ms=80;
    wire_send(c,100,1);
    assert(command_accepted && pre_grab.moving && task_protocol_mode()==53 && !task_context_complete());
    assert(!accepted_context.completed && !audit_received && !audit_valid && audit_consistent_count==0);
    assert(capture_forward_used_mm==40 && capture_wait_started_ms==80 && Camera_GetAngle()==140);
    pg_tick(120);assert(distance_calls==1 && fabsf(distance_request-.05f)<.0001f && !accepted_context.completed);
    distance_travelled_mm=15;test_pose.path_mm+=999;test_pose.y_mm+=999;
    pg_tick(140);assert(capture_forward_used_mm==55);
    unsigned calls=distance_calls;wire_send(c,160,2);
    assert(command_accepted && distance_calls==calls && pre_grab.charged_mm==15 && capture_wait_started_ms==80);
    VisionMissionCommand hold={.command=VISION_CMD_HOLD,.flags=VISION_CMD_VALID};
    wire_send(hold,180,3);pg_tick(180);assert(pre_grab.moving && distance_calls==calls+1);
    VisionMissionCommand audit=transport_sample_unused;
  }
}
'''
# Keep the actual fixture helpers and replace only this task's temporary entry point.
checks=checks.replace('    VisionMissionCommand audit=transport_sample_unused;',r'''
    VisionMissionCommand audit=injury_audit(3);audit.vision_frame=10;
    wire_send_action(audit,12,200,4);assert(!command_accepted && pre_grab.moving && audit_consistent_count==0);
    VisionMissionCommand grab={.command=VISION_CMD_GRAB_CONFIRMED,.flags=VISION_CMD_VALID};
    wire_send_action(grab,12,210,5);assert(!command_accepted && pre_grab.moving);
    VisionMissionCommand bad=c;bad.heading_cdeg=1;bad.target_y_mm=12000;
    wire_send(bad,220,6);assert(!command_accepted && pre_grab.moving);
    bad=c;bad.target_x_mm=40;wire_send(bad,230,7);assert(!command_accepted);
    bad=c;bad.vision_frame=10;wire_send(bad,240,8);assert(!command_accepted);
    wire_send_action(bad,12,250,9);assert(!command_accepted);
    VisionMissionCommand pause={.command=VISION_CMD_PAUSE,.flags=VISION_CMD_VALID};
    wire_send(pause,260,10);assert(mission_paused && finite_pauses>0);
    calls=distance_calls;pg_tick(760);assert(distance_calls==calls && capture_wait_started_ms==580);
    wire_send(c,780,11);assert(command_accepted && !mission_paused && pre_grab.moving);
    distance_travelled_mm=49;pg_tick(800);
    assert(pre_grab.moving && !task_context_complete() && capture_forward_used_mm==89);
    distance_result=MOTOR_DISTANCE_DONE;pg_tick(820);
    assert(pre_grab.active && !pre_grab.moving && accepted_context.completed && task_context_complete());
    assert(task_protocol_mode()==entry && capture_forward_used_mm==90 && Camera_GetAngle()==140);
    assert(Servo_GetAngle(4)==left && Servo_GetAngle(2)==right);
    calls=distance_calls;wire_send(c,840,12);pg_tick(840);
    assert(command_accepted && distance_calls==calls && accepted_context.completed && reverse_output==0);
    wire_send(hold,860,13);pg_tick(860);assert(distance_calls==calls && reverse_output==0);
    c.vision_frame=10;wire_send_action(c,12,880,14);assert(!command_accepted);
    c.vision_frame=11;distance_travelled_mm=0;distance_result=MOTOR_DISTANCE_RUNNING;
    wire_send_action(c,12,900,15);assert(command_accepted && pre_grab.moving && !accepted_context.completed);
    assert(capture_forward_used_mm==90 && capture_wait_started_ms==580);
    audit.vision_frame=12;wire_send_action(audit,13,910,16);assert(!command_accepted);
    distance_result=MOTOR_DISTANCE_DONE;distance_travelled_mm=48;pg_tick(920);
    assert(capture_forward_used_mm==140 && task_protocol_mode()==entry);
    // A fresh real audit is still needed; same-source picture cannot count.
    audit=injury_audit(4);audit.audit_flags=VISION_AUDIT_STABLE;
    audit.audit_left_class=VISION_CARGO_GREEN;audit.vision_frame=12;
    wire_send_action(audit,13,930,17);assert(command_accepted && audit_consistent_count==0 && !audit_valid);
    wire_send_action(audit,13,940,18);assert(command_accepted && audit_consistent_count==0 && !audit_valid);
    audit.vision_frame=13;audit.audit_id=5;wire_send_action(audit,13,960,19);
    assert(audit_consistent_count==1 && !audit_valid);
    audit.vision_frame=14;audit.audit_id=6;wire_send_action(audit,13,1080,20);pg_tick(1080);
    assert(audit_valid && reverse_output==0 && pre_grab.active && capture_wait_started_ms==580);
    assert(task_protocol_mode()==(entry==38?37:entry));
    wire_send_action(grab,14,1100,21);assert(command_accepted && state==TASK_CLOSE_CLAW && !pre_grab.active);
''')
checks+=r'''
static void pg_budget_and_rejects(void)
{
  VisionMissionCommand c=pg_prepare(21,true);
  capture_forward_used_mm=151;wire_send(c,100,1);
  assert(command_accepted && state==TASK_APPROACH_RECOVER && !pre_grab.active &&
         capture_escape_active && accepted_context.cancelled && !accepted_context.completed && distance_calls==0);
  c=pg_prepare(21,true);capture_forward_used_mm=150;wire_send(c,100,1);
  assert(pre_grab.moving);distance_result=MOTOR_DISTANCE_DONE;distance_travelled_mm=47;pg_tick(200);
  assert(capture_forward_used_mm==200 && accepted_context.completed && task_protocol_mode()==21);
  c.vision_frame=10;wire_send_action(c,12,220,2);
  assert(command_accepted && state==TASK_APPROACH_RECOVER && !accepted_context.completed);
  c=pg_prepare(38,true);capture_wait_active=true;capture_wait_started_ms=0;
  wire_send(c,3990,1);assert(pre_grab.moving);pg_tick(4000);
  assert(state==TASK_APPROACH_RECOVER && !pre_grab.active && !accepted_context.completed && task_status.fault==TASK_FAULT_NONE);
  c=pg_prepare(37,false);wire_send(c,100,1);distance_result=MOTOR_DISTANCE_FAULT;pg_tick(120);
  assert(state==TASK_APPROACH_RECOVER && !pre_grab.active && task_status.fault==TASK_FAULT_NONE);
  c=pg_prepare(21,false);wire_send(c,100,1);distance_travelled_mm=23;pg_tick(120);
  VisionMissionCommand stop={.command=VISION_CMD_STOP,.flags=VISION_CMD_VALID};
  wire_send_action(stop,12,140,2);assert(command_accepted && state==TASK_STOPPED && !pre_grab.active);
  c=pg_prepare(21,false);wire_send(c,100,1);test_vision.session_generation=1;
  test_vision.config_ready=true;test_vision.mission=(VisionMissionCommand){0};
  task_accept_mission(&test_vision,120);assert(!pre_grab.active && state==TASK_APPROACH_RECOVER && reverse_output==0);
  for(unsigned kind=0;kind<9;kind++) {
    c=pg_prepare(21,false);
    if(kind==0)task_status.gripper_closed=true;
    if(kind==1)cargo_recheck_pending=true;
    if(kind==2)opening_claw_probe_active=true;
    if(kind==3)separation_total_attempts=1;
    if(kind==4)capture_budget_active=false;
    if(kind==5)Camera_SetAngle(120);
    if(kind==6)c.vision_frame=0;
    if(kind==7)c.flags|=VISION_CMD_CLUSTER_TARGET;
    if(kind==8)task_enter(TASK_TRANSPORT_AUDIT,0);
    wire_send(c,100,1);assert(!command_accepted && !pre_grab.active && distance_calls==0);
  }
  c=pg_prepare(21,true);const uint32_t start=100;
  for(unsigned i=0;i<4;i++) {
    c.vision_frame=9+i;distance_travelled_mm=0;distance_result=MOTOR_DISTANCE_RUNNING;
    wire_send_action(c,11+i,start+i*400,1+i*2);assert(command_accepted && pre_grab.moving);
    distance_travelled_mm=47;distance_result=MOTOR_DISTANCE_DONE;pg_tick(start+i*400+200);
    assert(capture_forward_used_mm==(i+1)*50 && capture_wait_started_ms==start);
  }
  c.vision_frame=13;wire_send_action(c,15,1800,9);
  assert(command_accepted && state==TASK_APPROACH_RECOVER && !accepted_context.completed);
  puts("pre-grab: 3 entry modes x red/blue; no early DONE/replay/crawl; audits/GRAB fenced; PAUSE/HOLD/STOP; 200mm and4s shared budget; reject excluded entries PASS");
}
'''
try:
    stub=m_original.decode('utf-8').replace('\r\n','\n')
    stub=stub.replace('{ (void)distance; (void)speed; return MOTOR_DISTANCE_DONE; }',
        '{ (void)speed; distance_request=distance; ++distance_calls; return distance_result; }')
    m.write_text(stub,encoding='utf-8',newline='\n')
    s=original.decode('utf-8').replace('\r\n','\n')
    s=s.replace('int main(void)\n{',checks+'\nint main(void)\n{\n pg_runs();pg_budget_and_rejects();return 0;\n',1)
    p.write_text(s,encoding='utf-8',newline='\n')
    exe=r/'tmp/pregrab_bands_20261116/check.exe'
    subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
finally:
    p.write_bytes(original);m.write_bytes(m_original)
