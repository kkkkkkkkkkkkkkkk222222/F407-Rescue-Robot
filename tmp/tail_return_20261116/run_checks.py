from pathlib import Path
import subprocess,json
r=Path.cwd();p=r/'tools/test_audit_handshake.c';original=p.read_bytes()
contract=json.loads((r/'tmp/tail_return_20261116/upper_contract.json').read_text())
rows=[]
for e in contract['examples']:
 ms=e['messages'];rows.append('{'+','.join(map(str,[e['side']=='red',ms[0]['U'],ms[1]['U'],ms[2]['A'],ms[2]['B']])).lower().replace('true','1').replace('false','0')+'}')
checks=r'''
static void tail_setup(bool red)
{
  wire_prepare(TASK_REMOTE_ACTION,0);configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
  accepted_context.opcode=VISION_CMD_RELEASE_BOTH;accepted_context.flags=VISION_CMD_VALID;
  remote_action=(RemoteActionState){.type=REMOTE_ACTION_RELEASE_BOTH,.command=VISION_CMD_RELEASE_BOTH,.done=true};
  task_status.gripper_closed=false;first_delivery_done=false;distance_result=MOTOR_DISTANCE_RUNNING;
}
static VisionMissionCommand tail_step(unsigned step,bool red,int a,int b,unsigned u)
{
  return (VisionMissionCommand){.command=VISION_CMD_BULLDOZER_STEP,
    .flags=VISION_CMD_VALID|(red?VISION_CMD_RED_SIDE:0)|(step<<4),.target_x_mm=a,.target_y_mm=b,.heading_cdeg=u};
}
static void tail_paths(void)
{
  const int rows[][5]={ROWS};
  for(unsigned i=0;i<8;i++) {
    bool red=rows[i][0];unsigned opponent=rows[i][1],heading=rows[i][2];tail_setup(red);
    const uint8_t l=Servo_GetAngle(4),rr=Servo_GetAngle(2);
    VisionMissionCommand back=tail_step(15,red,-150,0,opponent);
    wire_send_action(back,11,100,1);assert(command_accepted && !remote_action.done && !accepted_context.completed);
    task_process_remote_action(&active_mission,120);
    assert(fabsf(distance_request+.15f)<.0001f && heading_output==opponent*.01f);
    test_pose.path_mm=99999;test_pose.x_mm=0;test_pose.y_mm=0;
    task_process_remote_action(&active_mission,140);assert(!remote_action.done);
    uint32_t started=remote_action.started_ms;wire_send_action(back,11,150,2);
    assert(command_accepted && remote_action.started_ms==started);
    VisionMissionCommand turn=tail_step(14,red,0,0,heading);
    wire_send_action(turn,12,160,3);assert(!command_accepted && remote_action.step==15);
    VisionMissionCommand pause={.command=VISION_CMD_PAUSE,.flags=VISION_CMD_VALID};
    wire_send_action(pause,11,170,4);assert(mission_paused && remote_action.paused && finite_pauses>0);
    wire_send_action(back,11,190,5);assert(command_accepted && !mission_paused);
    distance_result=MOTOR_DISTANCE_DONE;task_process_remote_action(&active_mission,200);
    assert(remote_action.done && task_context_complete() && task_protocol_mode()==25);
    unsigned calls=distance_calls;wire_send_action(back,11,210,6);task_process_remote_action(&active_mission,220);
    assert(command_accepted && distance_calls==calls && remote_action.done);
    VisionMissionCommand bad=back;bad.target_x_mm=-100;wire_send_action(bad,11,230,7);assert(!command_accepted);
    wire_send_action(turn,12,240,8);
    assert(command_accepted && !remote_action.done && !accepted_context.completed && remote_action.step==14);
    test_pose.heading_mdeg=opponent*10;task_process_remote_action(&active_mission,260);
    assert(!remote_action.done && reverse_output==0 && lateral_output==0 && yaw_output!=0);
    started=remote_action.started_ms;wire_send_action(turn,12,270,9);
    assert(command_accepted && remote_action.started_ms==started);
    bad=turn;bad.heading_cdeg=(heading+1000)%36000;wire_send_action(bad,12,280,10);
    assert(!command_accepted && remote_action.heading_cdeg==heading);
    VisionMissionCommand move=tail_step(13,red,rows[i][3],rows[i][4],heading);
    wire_send_action(move,13,290,11);assert(!command_accepted && remote_action.step==14);
    test_pose.heading_mdeg=heading*10;task_process_remote_action(&active_mission,300);
    assert(remote_action.done && task_context_complete() && task_protocol_mode()==25);
    wire_send_action(move,13,320,12);assert(command_accepted && !accepted_context.completed && !remote_action.done);
    task_process_remote_action(&active_mission,340);
    assert(reverse_output<0 && fabsf(lateral_output)<.1f && fabsf(heading_output-heading*.01f)<.001f);
    started=remote_action.started_ms;move.target_x_mm=100;move.target_y_mm+=100;
    wire_send_action(move,13,360,13);task_process_remote_action(&active_mission,380);
    assert(command_accepted && remote_action.started_ms==started && heading_output==heading*.01f && !remote_action.done);
    bad=move;bad.heading_cdeg=(heading+100)%36000;wire_send_action(bad,13,390,14);
    assert(!command_accepted && remote_action.heading_cdeg==heading);
    move.target_x_mm=0;move.target_y_mm=0;wire_send_action(move,13,400,15);task_process_remote_action(&active_mission,420);
    assert(remote_action.done && task_context_complete() && reverse_output==0);
    move.target_x_mm=100;wire_send_action(move,13,440,16);task_process_remote_action(&active_mission,460);
    assert(command_accepted && remote_action.done && reverse_output==0);
    assert(!first_delivery_done && Servo_GetAngle(4)==l && Servo_GetAngle(2)==rr);
    VisionMissionCommand ready=tail_step(12,red,0,0,0);wire_send_action(ready,14,480,17);
    assert(command_accepted && !accepted_context.completed && !remote_action.done);
    for(uint32_t t=500;t<2500 && state!=TASK_SEARCH;t+=20)task_process_remote_action(&active_mission,t);
    assert(state==TASK_SEARCH && accepted_context.completed);
  }
  puts("8 upstream geometry paths: release -> measured150 -> owned arbitrary turn -> live reverse -> D0 -> search, no stale DONE/replay/servo/recount PASS");
}
static void tail_boundaries(void)
{
  for(unsigned red=0;red<2;red++) {
    unsigned opponent=red?27000:9000;tail_setup(red);
    VisionMissionCommand turn=tail_step(14,red,0,0,12345);
    wire_send_action(turn,11,100,1);assert(!command_accepted); // Release alone cannot skip the retreat.
    wire_prepare(TASK_SEARCH,0);task_status.gripper_closed=false;
    wire_send_action(turn,11,100,1);assert(!command_accepted);
    tail_setup(red);VisionMissionCommand back=tail_step(15,red,-100,0,opponent);
    wire_send_action(back,11,100,1);task_process_remote_action(&active_mission,120);
    assert(command_accepted && fabsf(distance_request+.1f)<.0001f);
    distance_result=MOTOR_DISTANCE_DONE;task_process_remote_action(&active_mission,140);
    accepted_context.task_id=8;wire_send_action(turn,12,150,2);assert(!command_accepted);
    accepted_context.task_id=7;task_status.gripper_closed=true;
    wire_send_action(turn,12,160,3);assert(!command_accepted);
    task_status.gripper_closed=false;VisionMissionCommand ready=tail_step(12,red,0,0,0);
    wire_send_action(ready,12,170,4);assert(command_accepted); // Already at station: original no-turn path.
    wire_prepare(TASK_NAVIGATE,0);configured_color=red?VISION_COLOR_RED:VISION_COLOR_BLUE;
    accepted_context.opcode=VISION_CMD_NAVIGATE_WAYPOINT;distance_command_done=true;route_to_stash=true;
    task_status.gripper_closed=true;audit_received=audit_valid=audit_initial_stash=true;
    wire_send_action(turn,11,100,1);assert(!command_accepted);
    turn.heading_cdeg=opponent;wire_send_action(turn,11,110,2);assert(command_accepted && remote_action.step==14);
    test_pose.heading_mdeg=opponent*10;task_process_remote_action(&active_mission,120);
    VisionMissionCommand move=tail_step(13,red,500,0,opponent);
    wire_send_action(move,12,130,3);assert(!command_accepted); // Loaded turn never grants empty navigation.
    tail_setup(red);back.target_x_mm=-150;wire_send_action(back,11,100,1);
    distance_result=MOTOR_DISTANCE_DONE;task_process_remote_action(&active_mission,120);
    turn.heading_cdeg=35999;wire_send_action(turn,12,140,2);assert(command_accepted);
    VisionMissionCommand stop={.command=VISION_CMD_STOP,.flags=VISION_CMD_VALID};
    wire_send_action(stop,13,160,3);assert(command_accepted && state==TASK_STOPPED);
  }
  puts("Legacy100, no-turn ready, loaded opponent-only, wrong task/closed/SEARCH/unfinished/STOP boundaries PASS");
}
'''.replace('ROWS',','.join(rows))
try:
 s=original.decode('utf-8').replace('\r\n','\n').replace('int main(void)\n{',checks+'\nint main(void)\n{\n tail_paths();tail_boundaries();return 0;\n',1)
 p.write_text(s,encoding='utf-8',newline='\n');exe=r/'tmp/tail_return_20261116/check.exe'
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
finally:p.write_bytes(original)
