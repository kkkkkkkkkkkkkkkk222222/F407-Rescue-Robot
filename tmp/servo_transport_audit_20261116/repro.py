from pathlib import Path
import subprocess
r=Path.cwd();p=r/'tools/test_audit_handshake.c';old=p.read_bytes()
check=r'''
static void reproduce_transport_lift(void) {
 prepare();configured_color=VISION_COLOR_RED;initial_claw_ready=true;
 first_delivery_done=true;audit_received=audit_host_stable=audit_valid=true;
 Lift_SetTravelPosition();task_begin_single_filter(0,false);
 task_process_single_filter(1);
 assert(Servo_GetAngle(1)==APP_LIFT_FIRST_GREEN_FILTER_ANGLE);
 task_stop(TASK_FAULT_MOTOR,100);
 assert(state==TASK_RECOVERY_WAIT&&!single_filter_active);
 printf("filter interrupted -> recovery: S1=%u\n",Servo_GetAngle(1));
 recovery_reference_ready=true;task_status.claw_visible=true;Camera_SetAngle(140);
 // Real closed CORE observations, then the existing recovery GRAB -> post-grab audit.
 VisionMissionCommand c=injury_audit(1);c.audit_left_class=VISION_CARGO_CORE;c.audit_flags=VISION_AUDIT_STABLE;
 command_visual_new=true;task_accept_mission_legacy(&c,200);
 c.audit_id=2;c.vision_frame=3;task_accept_mission_legacy(&c,320);
 assert(audit_valid);
 VisionMissionCommand grab={.command=VISION_CMD_GRAB_CONFIRMED,.flags=VISION_CMD_VALID,.received=true};
 task_accept_mission_legacy(&grab,340);assert(state==TASK_CLOSE_CLAW);
 // Finish the original close, whose next state is the normal closed review.
 Claw_Touch(400);assert(Claw_Touch(1500));
 task_status.gripper_closed=true;task_clear_audit_result();task_enter(TASK_POST_GRAB_AUDIT,1500);task_status.claw_visible=true;
 c.audit_id=3;c.vision_frame=4;task_accept_mission_legacy(&c,1600);
 c.audit_id=4;c.vision_frame=5;task_accept_mission_legacy(&c,1720);
 assert(state==TASK_WAIT_NAVIGATION&&audit_valid);
 VisionMissionCommand nav={.received=true,.command=VISION_CMD_NAVIGATE_WAYPOINT,
 .flags=VISION_CMD_VALID|VISION_CMD_RED_SIDE|VISION_CMD_DISTANCE_VALID|VISION_CMD_STAGE_ONLY,.target_x_mm=100,.heading_cdeg=9000};
 task_accept_mission_legacy(&nav,1800);assert(state==TASK_NAVIGATE);
 printf("normal closed CORE NAV accepted: S1=%u (transport configuration=%u)\n",Servo_GetAngle(1),APP_LIFT_TRAVEL_ANGLE);
}
'''
try:
 source=old.decode().replace('\r\n','\n').replace('int main(void)\n{',check+'\nint main(void)\n{\ntest_filter_recovery_restores_transport_lift();puts("Filter interruption -> real CORE re-grab -> normal NAV restores85 PASS");return 0;\n',1)
 p.write_text(source,encoding='utf-8',newline='\n')
 exe=r/'tmp/servo_transport_audit_20261116/repro.exe'
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
finally:p.write_bytes(old)

