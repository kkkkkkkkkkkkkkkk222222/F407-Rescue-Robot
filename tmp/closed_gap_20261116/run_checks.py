from pathlib import Path
import subprocess
r=Path.cwd();p=r/'tools/test_audit_handshake.c';original=p.read_bytes()
checks=r'''
static VisionMissionCommand gap_audit(bool red,unsigned kind,unsigned id,unsigned frame)
{
 VisionMissionCommand c=injury_audit(id);(void)red;c.flags=VISION_CMD_VALID;
 c.vision_frame=frame;
 if(kind!=2){c.audit_flags=VISION_AUDIT_STABLE;c.audit_left_class=kind==1?VISION_CARGO_CORE:VISION_CARGO_GREEN;}
 if(kind==3){c.audit_right_class=VISION_CARGO_CORE;c.audit_counts=5;c.audit_total_count=2;}
 return c;
}
static VisionMissionCommand gap_align(bool red)
{
 return (VisionMissionCommand){.command=VISION_CMD_ALIGN_SAFE_ZONE,
 .flags=VISION_CMD_VALID|VISION_CMD_USE_FINAL_HEADING|VISION_CMD_REUSE_CONFIRMED_LOAD|(red?VISION_CMD_RED_SIDE:0),
 .heading_cdeg=red?9000:27000};
}
static VisionMissionCommand gap_nav(bool red,bool reuse)
{
 return (VisionMissionCommand){.command=VISION_CMD_NAVIGATE_WAYPOINT,
 .flags=VISION_CMD_VALID|VISION_CMD_DISTANCE_VALID|VISION_CMD_STAGE_ONLY|VISION_CMD_DRIVE_STRAIGHT|
 VISION_CMD_USE_FINAL_HEADING|(reuse?VISION_CMD_REUSE_CONFIRMED_LOAD:0)|(red?VISION_CMD_RED_SIDE:0),
 .target_x_mm=150,.heading_cdeg=red?9000:27000};
}
static void gap_seed(bool red,unsigned kind)
{
 wire_prepare(TASK_POST_GRAB_AUDIT,0);configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
 first_delivery_done=kind!=0;task_status.claw_visible=true;
 VisionMissionCommand c=gap_audit(red,kind,1,9);wire_send_action(c,11,100,1);
 assert(command_accepted&&!confirmed_load.valid);
 if(audit_consistent_count!=1)fprintf(stderr,"first count%u pitch%u visible%d sample%u frame%u new%d\n",audit_consistent_count,Camera_GetAngle(),task_status.claw_visible,audit_sample_ms,accepted_context.vision_frame,command_visual_new);
 c.audit_id=2;c.vision_frame=10;wire_send_action(c,11,220,2);
 if(!(command_accepted&&audit_received&&audit_valid&&confirmed_load.valid&&state==TASK_WAIT_NAVIGATION))fprintf(stderr,"seed red%d kind%u state%d accept%d audit%d/%d conf%d count%u reject%u visible%d filter%d\n",red,kind,state,command_accepted,audit_received,audit_valid,confirmed_load.valid,audit_consistent_count,diag_reject_stage,task_status.claw_visible,single_filter_armed);
 assert(command_accepted&&audit_received&&audit_valid&&confirmed_load.valid&&state==TASK_WAIT_NAVIGATION);
 c=gap_nav(red,false);wire_send_action(c,12,300,3);assert(command_accepted&&state==TASK_NAVIGATE);
 distance_command_done=true;
 c=placeholder();c.audit_flags=kind==2?VISION_AUDIT_DESTINATION_INJURY:0;
 wire_send_action(c,13,400,4);assert(command_accepted&&state==TASK_TRANSPORT_AUDIT);
 task_process_transport_audit(400+APP_GRAB_CAMERA_SETTLE_MS);
 assert(confirmed_load.valid&&!audit_received&&!audit_valid&&audit_consistent_count==0&&transport_audit_camera_ready);
}
static void gap_reuse_paths(void)
{
 for(unsigned red=0;red<2;red++) for(unsigned kind=0;kind<4;kind++) for(unsigned redirect=0;redirect<2;redirect++){
  gap_seed(red,kind);
  VisionMissionCommand empty=placeholder();empty.audit_id=3;empty.vision_frame=11;
  empty.audit_flags=kind==2?VISION_AUDIT_DESTINATION_INJURY:0;
  wire_send_action(empty,13,1100,5);assert(command_accepted&&empty_load_reobserved&&transport_audit_phase==2);
  task_process_transport_audit(1500);task_process_transport_audit(2100);
  assert(confirmed_load.valid&&!audit_valid&&!audit_received);
  uint32_t phase_started=state_started_ms;
  unsigned gen=6,action=14;
  if(redirect){
   VisionMissionCommand nav=gap_nav(red,true);
   wire_send_action(nav,action,3100,gen++);assert(command_accepted&&state==TASK_NAVIGATE&&confirmed_load.active);
   assert(!audit_received&&!audit_valid&&audit_consistent_count==0);
   VisionMissionCommand a=gap_align(red);wire_send_action(a,action+1,3110,gen++);
   assert(!command_accepted&&state==TASK_NAVIGATE);
   uint32_t started=state_started_ms;nav.target_x_mm=80;
   wire_send_action(nav,action,3120,gen++);assert(command_accepted&&state_started_ms==started&&confirmed_load.active);
   nav.target_x_mm=0;wire_send_action(nav,action,3130,gen++);
   test_pose.heading_mdeg=(red?9000:27000)*10;
   assert(nav_follow(&active_mission,VISION_CMD_NAVIGATE_WAYPOINT,APP_NAV_FAST_SPEED_MM_S,3140)==REMOTE_ROUTE_REACHED);assert(distance_command_done);
   ++action;
  }
  VisionMissionCommand a=gap_align(red);wire_send_action(a,action,3200,gen++);
  assert(command_accepted&&state==TASK_ALIGN_SAFE_ZONE&&confirmed_load.active);
  assert(audit_destination_injury==(kind==2)&&!audit_received&&!audit_valid&&audit_consistent_count==0);
  phase_started=state_started_ms;wire_send_action(a,action,3210,gen++);
  assert(command_accepted&&state_started_ms==phase_started);
  test_pose.heading_mdeg=(red?9000:27000)*10;
  task_process_safe_align(3300);task_process_safe_align(4300);assert(safe_align_done&&task_status.nav_heading_locked);
  VisionMissionCommand visual={.command=VISION_CMD_ALIGN_SAFE_ZONE,
    .flags=VISION_CMD_VALID|VISION_CMD_VISUAL_CORRECTION_VALID|(red?VISION_CMD_RED_SIDE:0),.vision_frame=12};
  wire_send_action(visual,++action,4400,gen++);assert(command_accepted);
  task_process_safe_align(4420);assert(safe_align_done&&safe_align_visual_confirmed);
  VisionMissionCommand enter={.command=VISION_CMD_ENTER_SAFE_ZONE,
   .flags=VISION_CMD_VALID|VISION_CMD_DRIVE_STRAIGHT|VISION_CMD_VISUAL_CORRECTION_VALID|(red?VISION_CMD_RED_SIDE:0)};
  wire_send_action(enter,++action,4500,gen++);
  assert(command_accepted&&delivery_enter_active&&confirmed_load.active&&!audit_received&&!audit_valid);
 }
 puts("16 red/blue x load x direct/redirect paths: genuine audit -> missing -> reuse -> visual ALIGN/ENTER, no fabricated audit PASS");
}
static void gap_revocations(void)
{
 for(unsigned kind=0;kind<4;kind++){
  gap_seed(true,kind);
  VisionMissionCommand c=gap_audit(true,kind,4,12);
  c.audit_flags&=~VISION_AUDIT_STABLE;
  if(kind==0)c.audit_flags|=VISION_AUDIT_DANGER_PRESENT;
  if(kind==1)c.audit_left_class=VISION_CARGO_GREEN;
  if(kind==2){c.audit_counts=c.audit_total_count=2;}
  if(kind==3)c.audit_flags|=VISION_AUDIT_UNKNOWN_PRESENT;
  wire_send_action(c,13,1100,5);
  if(!(command_accepted&&!confirmed_load.valid&&!audit_received))fprintf(stderr,"revoke kind%u accept%d conf%d audit%d state%d ready%d new%d pitch%d\n",kind,command_accepted,confirmed_load.valid,audit_received,state,transport_audit_camera_ready,command_visual_new,Camera_GetAngle());
  assert(command_accepted&&!confirmed_load.valid&&!audit_received);
  VisionMissionCommand a=gap_align(true);wire_send_action(a,14,1200,6);assert(!command_accepted);
  // Two new actual legal samples may establish a new permission.
  c=gap_audit(true,kind,5,13);wire_send_action(c,13,1300,7);
  c.audit_id=6;c.vision_frame=14;wire_send_action(c,13,1420,8);
  assert(command_accepted&&confirmed_load.valid&&audit_valid);
 }
 gap_seed(true,3);
 VisionMissionCommand mixed=gap_audit(true,3,4,12);
 mixed.audit_left_class=VISION_CARGO_MIXED_MATERIAL;mixed.audit_right_class=VISION_CARGO_NONE;mixed.audit_counts=2;
 mixed.audit_flags=0;wire_send_action(mixed,13,1100,5);assert(confirmed_load.valid);
 // A same-view placeholder never revokes historic permission.
 wire_send_action(placeholder(),13,1200,6);assert(confirmed_load.valid&&!audit_valid);
 gap_seed(true,0);
 VisionMissionCommand conflict=placeholder();conflict.audit_flags=VISION_AUDIT_FIRST_GREEN_CORE_CONFLICT;
 wire_send_action(conflict,13,1100,5);assert(!confirmed_load.valid&&first_green_core_conflict);
 for(unsigned i=0;i<7;i++){
  gap_seed(true,2);
  const TaskState exits[]={TASK_OPEN_CLAW,TASK_OPEN_FOR_RAM,TASK_SAFE_SWEEP,TASK_REMOTE_ACTION,TASK_SEARCH,TASK_STOPPED,TASK_RECOVERY_WAIT};
  task_enter(exits[i],1100);assert(!confirmed_load.valid);
 }
 gap_seed(true,2);VisionData session={.session_generation=1};task_accept_mission(&session,1100);assert(!confirmed_load.valid);
 gap_seed(true,2);task_status.gripper_closed=false;VisionData no_command={0};task_accept_mission(&no_command,1100);assert(!confirmed_load.valid);
 // Conflict-driven re-audit must stop a reused redirect even before D=0.
 gap_seed(true,2);VisionMissionCommand nav=gap_nav(true,true);
 wire_send_action(nav,14,1100,5);assert(confirmed_load.active&&!distance_command_done);
 reverse_output=100;VisionMissionCommand review=placeholder();review.audit_flags=VISION_AUDIT_DESTINATION_INJURY;
 wire_send_action(review,15,1200,6);
 assert(command_accepted&&state==TASK_TRANSPORT_AUDIT&&!confirmed_load.active&&reverse_output==0);
 // Withdrawal cannot be re-enabled mid-NAV.
 gap_seed(true,2);wire_send_action(nav,14,1100,5);
 nav.flags&=~VISION_CMD_REUSE_CONFIRMED_LOAD;wire_send_action(nav,15,1200,6);
 assert(command_accepted&&!confirmed_load.active);
 nav.flags|=VISION_CMD_REUSE_CONFIRMED_LOAD;wire_send_action(nav,16,1300,7);
 assert(command_accepted&&!confirmed_load.active);distance_command_done=true;
 wire_send_action(gap_align(true),17,1400,8);assert(!command_accepted);
 // Parking a reused load carries historical identity only into the existing sweep record.
 gap_seed(true,3);wire_send_action(gap_align(true),14,1100,5);
 VisionMissionCommand sweep={.command=VISION_CMD_CLEAR_SAFE_ZONE,.flags=VISION_CMD_VALID|VISION_CMD_RED_SIDE,.target_y_mm=200};
 safe_align_done=safe_align_visual_confirmed=true;test_pose.heading_mdeg=90000;
 wire_send_action(sweep,15,1200,6);
 assert(command_accepted&&state==TASK_SAFE_SWEEP&&sweep_original_total==2&&
   sweep_original_left==VISION_CARGO_GREEN&&sweep_original_right==VISION_CARGO_CORE&&!confirmed_load.valid&&!audit_valid);
 puts("Changed class/count/danger/first-green revoke; normalized side/MIXED; release/separation/session/STOP lifecycle; active NAV re-audit/withdrawal PASS");
}
static void gap_cancel_paths(void)
{
 for(unsigned red=0;red<2;red++)for(unsigned kind=0;kind<9;kind++){
  const TaskState states[]={TASK_APPROACH,TASK_APPROACH,TASK_APPROACH,TASK_GRAB_OBSERVE,TASK_APPROACH,TASK_DISPERSE_READY,TASK_POST_GRAB_AUDIT,TASK_TRANSPORT_AUDIT,TASK_GRAB_OBSERVE};
  wire_prepare(states[kind],0);configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
  task_status.gripper_closed=kind==6||kind==7;first_delivery_done=false;
  audit_received=audit_valid=false;cluster_target_active=kind==4||kind==5;
  if(kind==1)approach_phase=APPROACH_RANGE_WAIT;
  if(kind==2){approach_phase=APPROACH_RANGE_MOVE;approach_range_mm=500;}
  if(kind==4)approach_phase=APPROACH_CLUSTER_CAPTURE;
  if(kind==8){pre_grab.active=pre_grab.moving=true;pre_grab.source_frame=8;capture_budget_active=true;}
  reverse_output=150;distance_result=MOTOR_DISTANCE_RUNNING;
  VisionMissionCommand release={.command=VISION_CMD_RELEASE_BOTH,.flags=VISION_CMD_VALID|(red?VISION_CMD_RED_SIDE:0)};
  wire_send_action(release,11,100,1);
  assert(command_accepted&&state==TASK_APPROACH_RECOVER&&capture_escape_active&&!pre_grab.active&&!pre_grab.moving);
  assert(reverse_output==0&&!accepted_context.completed&&!confirmed_load.valid&&!audit_received);
  uint32_t start=state_started_ms;
  wire_send_action(release,11,200,2);assert(command_accepted&&state_started_ms==start&&capture_escape_phase==0);
  VisionMissionCommand app={.command=VISION_CMD_APPROACH_TARGET,.flags=release.flags,.vision_frame=12,.target_x_mm=640,.target_y_mm=400};
  wire_send_action(app,10,210,3);assert(!command_accepted&&reverse_output==0);
  for(unsigned t=300;t<4000&&capture_escape_phase==0;t+=100)task_process_capture_escape(t);
  assert(capture_escape_phase==1&&!task_status.gripper_closed);
  test_pose.path_mm=capture_escape_path_mm+APP_CAPTURE_ESCAPE_DISTANCE_MM;
  task_process_capture_escape(4100);task_process_capture_escape(4200);
  assert(state==TASK_SEARCH&&accepted_context.completed&&!first_delivery_done);
  wire_send_action(release,11,4300,4);assert(command_accepted&&state==TASK_SEARCH);
 }
 // Wrong side/task/action and single-side release do not gain this permission.
 for(unsigned kind=0;kind<5;kind++){
  wire_prepare(TASK_GRAB_OBSERVE,0);configured_color=VISION_COLOR_RED;task_status.gripper_closed=false;audit_received=false;
  pre_grab.active=pre_grab.moving=true;
  VisionMissionCommand c={.command=VISION_CMD_RELEASE_BOTH,.flags=VISION_CMD_VALID|VISION_CMD_RED_SIDE};
  if(kind==0)c.flags=VISION_CMD_VALID;
  if(kind==1)c.command=VISION_CMD_RELEASE_LEFT;
  if(kind==2)c.target_x_mm=1;
  if(kind==3)c.vision_frame=9;
  if(kind==4)accepted_context.task_id=6;
  wire_send_action(c,11,100,1);assert(!command_accepted&&pre_grab.moving&&state==TASK_GRAB_OBSERVE);
 }
 puts("18 open/closed/range/pre-grab cancel paths: immediate stop, old epoch cancelled, repeat idempotent, actual recovery -> SEARCH; malformed/unowned/single-side rejected PASS");
}
static void gap_camera_and_legacy(void)
{
 for(unsigned transport=0;transport<2;transport++){
  wire_prepare(transport?TASK_TRANSPORT_AUDIT:TASK_POST_GRAB_AUDIT,0);
  configured_color=VISION_COLOR_RED;task_status.gripper_closed=task_status.claw_visible=true;
  transport_audit_camera_ready=true;transport_audit_phase=1;Camera_SetAngle(140);
  VisionMissionCommand e=placeholder();e.vision_frame=9;e.audit_id=1;
  wire_send_action(e,11,100,1);assert(command_accepted&&empty_load_reobserved&&Camera_GetAngle()==138);
  if(transport){task_process_transport_audit(500);task_process_transport_audit(1100);task_process_transport_audit(8000);}
  else{task_process_post_grab_audit(500);task_process_post_grab_audit(1100);task_process_post_grab_audit(8000);}
  assert(Camera_GetAngle()==140&&task_status.gripper_closed);
  assert(state==(transport?TASK_TRANSPORT_AUDIT:TASK_POST_GRAB_AUDIT));
  task_audit_watchdog(100);task_audit_watchdog(100+APP_AUDIT_STATE_TIMEOUT_MS);
  assert(state==TASK_APPROACH_RECOVER); // Original absent-host watchdog remains.
 }
 // Ordinary no-reuse mode48 and recovered single-injury bit5 entry remain legal.
 gap_seed(true,2);VisionMissionCommand a=gap_align(true);a.flags&=~VISION_CMD_REUSE_CONFIRMED_LOAD;
 wire_send_action(a,14,1100,5);assert(!command_accepted);
 VisionMissionCommand c=gap_audit(true,2,4,12);wire_send_action(c,13,1200,6);
 c.audit_id=5;c.vision_frame=13;wire_send_action(c,13,1320,7);assert(audit_valid);
 wire_send_action(a,14,1400,8);assert(command_accepted&&state==TASK_ALIGN_SAFE_ZONE);
 wire_prepare(TASK_POST_GRAB_AUDIT,0);configured_color=VISION_COLOR_RED;task_status.claw_visible=true;
 c=gap_audit(true,2,1,9);wire_send_action(c,11,100,1);
 c.audit_id=2;c.vision_frame=10;wire_send_action(c,11,220,2);
 VisionMissionCommand nav=gap_nav(true,true);wire_send_action(nav,12,300,3);
 assert(command_accepted&&nav_reuse_post_grab_audit);
 distance_command_done=true;wire_send_action(a,13,400,4);assert(command_accepted&&state==TASK_ALIGN_SAFE_ZONE);
 puts("One empty-triggered camera nudge; no second wait/local2s release; original15s watchdog and real-audit/legacy injury NAV paths PASS");
}
'''
checks=checks.replace('wire_prepare(', 'gap_prepare(')
checks='static void gap_prepare(TaskState s,uint32_t now){wire_prepare(s,now);audit_frame_floor_valid=false;}\n'+checks
try:
 s=original.decode('utf-8').replace('\r\n','\n').replace('int main(void)\n{',checks+'\nint main(void)\n{\n gap_reuse_paths();gap_revocations();gap_cancel_paths();gap_camera_and_legacy();return 0;\n',1)
 p.write_text(s,encoding='utf-8',newline='\n');exe=r/'tmp/closed_gap_20261116/check.exe'
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
finally:p.write_bytes(original)

