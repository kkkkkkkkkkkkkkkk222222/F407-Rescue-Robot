from pathlib import Path
import ast, subprocess
r=Path.cwd(); p=r/'tools/test_audit_handshake.c'; original=p.read_bytes()
old=ast.parse((r/'tmp/closed_gap_20261116/run_checks.py').read_text(encoding='utf-8'))
helpers=next(ast.literal_eval(n.value) for n in old.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='checks' for t in n.targets))
helpers=helpers[:helpers.index('static void gap_reuse_paths')]
helpers=helpers.replace('wire_prepare(', 'gap_prepare(')
helpers='static void gap_prepare(TaskState s,uint32_t now){wire_prepare(s,now);audit_frame_floor_valid=false;}\n'+helpers
checks=r'''
static unsigned stage_gen, stage_action, stage_now;
static void stage_finish_pose(void) {
 stage_now+=20;task_process_safe_align(stage_now);
 stage_now+=1000;task_process_safe_align(stage_now);
 assert(safe_align_done);
}
static void stage_send(VisionMissionCommand c, bool next) {
 if(next) ++stage_action;
 stage_now+=120; ++stage_gen;
 wire_send_action(c,stage_action,stage_now,stage_gen);
}
static VisionMissionCommand stage_pose(bool red,bool reuse) {
 VisionMissionCommand c=gap_align(red);
 if(!reuse)c.flags&=~VISION_CMD_REUSE_CONFIRMED_LOAD;
 return c;
}
static VisionMissionCommand stage_nav(bool red,bool reuse) {
 VisionMissionCommand c=gap_nav(red,reuse);
 // Actual upper navigation builder uses compact STAGE, H is translation.
 c.flags&=~(VISION_CMD_DRIVE_STRAIGHT|VISION_CMD_USE_FINAL_HEADING);
 return c;
}
static void stage_seed(bool red,unsigned kind,bool reuse) {
 gap_seed(red,kind); stage_gen=10;stage_action=13;stage_now=2000;
 if(!reuse) {
  VisionMissionCommand c=gap_audit(red,kind,4,12);wire_send_action(c,13,1200,6);
  c.audit_id=5;c.vision_frame=13;wire_send_action(c,13,1320,7);
  assert(audit_received&&audit_valid);
 }
 VisionMissionCommand a=stage_pose(red,reuse);stage_send(a,true);
 assert(command_accepted&&state==TASK_ALIGN_SAFE_ZONE);
 test_pose.heading_mdeg=(red?9000:27000)*10;
 stage_finish_pose();
 VisionMissionCommand v={.command=VISION_CMD_ALIGN_SAFE_ZONE,
 .flags=VISION_CMD_VALID|VISION_CMD_VISUAL_CORRECTION_VALID|(red?VISION_CMD_RED_SIDE:0),
 .vision_frame=20};stage_send(v,true);assert(command_accepted);
 task_process_safe_align(stage_now+20);assert(safe_align_done&&safe_align_visual_confirmed);
 assert(task_context_complete());
}
static void stage_paths(void) {
 unsigned paths=0;
 for(unsigned red=0;red<2;red++)for(unsigned kind=0;kind<4;kind++)for(unsigned reuse=0;reuse<2;reuse++) {
  stage_seed(red,kind,reuse);
  VisionMissionCommand n=stage_nav(red,reuse);stage_send(n,true);
  assert(command_accepted&&state==TASK_NAVIGATE&&delivery_stage_only&&!route_to_stash);
  assert(!safe_align_done&&!safe_align_visual_confirmed&&!distance_command_done);
  assert(!accepted_context.completed&&!task_context_complete());
  assert(task_status.gripper_closed&&audit_destination_injury==(kind==2));
  if(reuse)assert(confirmed_load.active&&!audit_received&&!audit_valid);
  uint32_t started=state_started_ms;
  n.target_x_mm=80;n.heading_cdeg=red?27000:9000;stage_send(n,false);
  assert(command_accepted&&state_started_ms==started&&!task_context_complete());
  VisionMissionCommand pause={.command=VISION_CMD_PAUSE,.flags=VISION_CMD_VALID};stage_send(pause,false);
  assert(command_accepted&&mission_paused);
  stage_send(n,false);assert(command_accepted&&!mission_paused&&state_started_ms==started);
  n.target_x_mm=0;stage_send(n,false);
  assert(nav_follow(&active_mission,VISION_CMD_NAVIGATE_WAYPOINT,APP_NAV_FAST_SPEED_MM_S,stage_now+20)==REMOTE_ROUTE_REACHED);
  assert(distance_command_done&&task_context_complete());
  if(!reuse){
   VisionMissionCommand ph=placeholder();ph.audit_flags=kind==2?VISION_AUDIT_DESTINATION_INJURY:0;
   stage_send(ph,true);assert(command_accepted&&state==TASK_TRANSPORT_AUDIT);
   stage_now+=1000;task_process_transport_audit(stage_now);
   VisionMissionCommand c=gap_audit(red,kind,6,21);stage_send(c,false);
   c.audit_id=7;c.vision_frame=22;stage_gen+=10;stage_send(c,false);
   assert(audit_received&&audit_valid);
  }
  stage_send(stage_pose(red,reuse),true);
  assert(command_accepted&&state==TASK_ALIGN_SAFE_ZONE&&!safe_align_visual_confirmed&&!safe_align_done);
  assert(!accepted_context.completed&&!task_context_complete());
  stage_finish_pose();
  VisionMissionCommand v={.command=VISION_CMD_ALIGN_SAFE_ZONE,
   .flags=VISION_CMD_VALID|VISION_CMD_VISUAL_CORRECTION_VALID|(red?VISION_CMD_RED_SIDE:0),.vision_frame=30};
  stage_send(v,true);task_process_safe_align(stage_now+20);assert(safe_align_visual_confirmed&&safe_align_done);
  VisionMissionCommand enter={.command=VISION_CMD_ENTER_SAFE_ZONE,
   .flags=VISION_CMD_VALID|VISION_CMD_DRIVE_STRAIGHT|VISION_CMD_VISUAL_CORRECTION_VALID|(red?VISION_CMD_RED_SIDE:0)};
  stage_send(enter,true);assert(command_accepted&&delivery_enter_active);
  stage_send(n,true);assert(!command_accepted&&delivery_enter_active);
  paths++;
 }
 printf("STAGE repair/D0/audit or confirmed-load/ALIGN/visual/ENTER: %u paths PASS\n",paths);
}
static void stage_pose_restart(void) {
 for(unsigned red=0;red<2;red++)for(unsigned reuse=0;reuse<2;reuse++) {
  stage_seed(red,2,reuse);VisionMissionCommand a=stage_pose(red,reuse);a.target_y_mm=11500;
  stage_send(a,true);assert(command_accepted&&!safe_align_done&&!safe_align_visual_confirmed&&!nav_ready);
  assert(!accepted_context.completed&&!task_context_complete()&&Camera_GetAngle()==115);
  assert(task_status.gripper_closed&&audit_destination_injury);
  uint32_t started=state_started_ms;
  stage_finish_pose();
  stage_send(a,false);assert(command_accepted&&safe_align_done&&state_started_ms==started);
  assert(!safe_align_visual_confirmed);
  VisionMissionCommand old={.command=VISION_CMD_ALIGN_SAFE_ZONE,
   .flags=VISION_CMD_VALID|VISION_CMD_VISUAL_CORRECTION_VALID|(red?VISION_CMD_RED_SIDE:0),.vision_frame=20};
  wire_send_action(old,stage_action-1,stage_now+20,++stage_gen);assert(!command_accepted&&!safe_align_visual_confirmed);
 }
 puts("New pose action clears old visual DONE; repeated pose is idempotent: 4 paths PASS");
}
static void stage_rejections(void) {
 for(unsigned which=0;which<10;which++) {
  stage_seed(true,2,which==8||which==9);VisionMissionCommand n=stage_nav(true,which==8||which==9);
  if(which==0)n.flags&=~VISION_CMD_STAGE_ONLY;
  if(which==1)n.flags&=~VISION_CMD_RED_SIDE;
  if(which==2)n.target_x_mm=-1;
  if(which==3)n.target_y_mm=1;
  if(which==4)n.heading_cdeg=36000;
  if(which==5)task_status.gripper_closed=false;
  if(which==6)audit_valid=false;
  if(which==7)n.vision_frame=21;
  if(which==8)task_clear_confirmed_load();
  if(which==9)n.flags&=~VISION_CMD_REUSE_CONFIRMED_LOAD;
  stage_send(n,true);assert(!command_accepted&&state==TASK_ALIGN_SAFE_ZONE&&safe_align_visual_confirmed);
 }
 stage_seed(true,2,false);VisionMissionCommand n=stage_nav(true,false);
 stage_send(n,false);assert(!command_accepted&&state==TASK_ALIGN_SAFE_ZONE);
 n.received=n.payload_valid=n.context_valid=true;n.task_id=8;n.action_id=1;n.generation=++stage_gen;
 VisionData vv={.config_ready=true,.mission=n};task_accept_mission(&vv,stage_now+20);
 assert(!command_accepted&&state==TASK_ALIGN_SAFE_ZONE);
 puts("Invalid payload/side/ownership/open claw/revoked or unmarked load: 12 cases PASS");
}
'''
try:
 s=original.decode('utf-8').replace('\r\n','\n').replace('int main(void)\n{',helpers+checks+'\nint main(void)\n{\n stage_paths();stage_pose_restart();stage_rejections();return 0;\n',1)
 p.write_text(s,encoding='utf-8',newline='\n')
 exe=r/'tmp/stage_reentry_20261117/check.exe'
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
finally:p.write_bytes(original)
