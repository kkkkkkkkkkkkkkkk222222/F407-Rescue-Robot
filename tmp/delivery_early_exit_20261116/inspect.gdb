set pagination off
set confirm off
start
set $blue=0

define begin_delivery
call wire_prepare(TASK_ALIGN_SAFE_ZONE, 900)
set initial_claw_ready=1
set test_vision.config_ready=1
set first_delivery_done=0
set configured_color=($blue ? 0x12 : 0x11)
set accepted_context.opcode=4
set accepted_context.flags=($blue ? 1 : 9)
set accepted_context.completed=1
set safe_align_done=1
set safe_align_visual_confirmed=1
set test_pose.heading_mdeg=($blue ? 270000 : 90000)
set test_pose.path_mm=0
call Claw_Touch(0)
call Claw_Touch(800)
set test_vision.mission.command=5
set test_vision.mission.flags=($blue ? 67 : 75)
call wire_send_action(test_vision.mission, 11, 1000, 1)
set test_vision.mission.received=0
call Task_Process(1000)
end

define send_command
set test_vision.mission=active_mission
set test_vision.mission.command=$arg0
set test_vision.mission.flags=$arg1
set test_vision.mission.action_id=$arg2
set test_vision.mission.generation=$arg3
set test_vision.mission.sequence=$arg3
set test_vision.mission.tick_ms=$arg4
if $arg0 == 8
set test_vision.mission.target_x_mm=700
set test_vision.mission.target_y_mm=0
set test_vision.mission.heading_cdeg=9000
end
call Task_Process($arg4)
end

begin_delivery
printf "CHECK enter_budget_origin=%d\n", delivery.active && delivery.front_budget_active && delivery.front_elapsed_ms==0 && state==TASK_NAVIGATE
call Task_Process(2000)
printf "CHECK normal_front_running=%d\n", delivery.front_elapsed_ms==1000 && state==TASK_NAVIGATE && reverse_output>0
send_command 6 9 12 2 2100
printf "CHECK early_complete_owns_open_not_done=%d\n", state==TASK_OPEN_FOR_RAM && accepted_context.opcode==6 && accepted_context.action_id==12 && task_status.gripper_closed && !first_delivery_done && !task_context_complete() && !delivery.front_budget_active
set $open_start=claw_step_ms
send_command 6 9 12 3 2120
printf "CHECK complete_repeat_no_reopen=%d\n", claw_step_ms==$open_start && state==TASK_OPEN_FOR_RAM
send_command 5 75 11 4 2140
printf "CHECK old_enter_cannot_resume_push=%d\n", accepted_context.opcode==6 && accepted_context.action_id==12 && state==TASK_OPEN_FOR_RAM && reverse_output==0
set $open_finish=$open_start+claw_wait_ms
call Task_Process($open_finish)
printf "CHECK actual_open_then_exit_done=%d\n", state==TASK_EXIT_SAFE_ZONE && !task_status.gripper_closed && first_delivery_done && task_context_complete() && delivery.finish==DELIVERY_FINISH_COMPLETE && !audit_valid && locked_cargo_counts==0
set $exit_ready=delivery_exit_phase_ready_ms
set test_vision.mission=active_mission
set test_vision.mission.target_x_mm=700
set test_vision.mission.target_y_mm=0
set test_vision.mission.heading_cdeg=9000
send_command 8 31 13 5 $open_finish+1
printf "CHECK return_waits_for_original_exit=%d\n", state==TASK_EXIT_SAFE_ZONE && accepted_context.opcode==8 && delivery_exit_phase_ready_ms==$exit_ready && !delivery.active

begin_delivery
call Task_Process(4999)
printf "CHECK 3999ms_not_timeout=%d\n", state==TASK_NAVIGATE && delivery.front_elapsed_ms==3999
send_command 5 75 11 2 5000
printf "CHECK 4000ms_local_open_keeps_enter=%d\n", state==TASK_OPEN_FOR_RAM && delivery.finish==DELIVERY_FINISH_FRONT_TIMEOUT && accepted_context.opcode==5 && accepted_context.action_id==11 && task_status.gripper_closed && !first_delivery_done && !task_context_complete()
set $open_start=claw_step_ms
send_command 6 9 12 3 5020
printf "CHECK late_complete_rejected_keeps_reason=%d\n", accepted_context.opcode==5 && accepted_context.action_id==11 && delivery.finish==DELIVERY_FINISH_FRONT_TIMEOUT && accepted_context.rejection_valid && claw_step_ms==$open_start
send_command 5 75 13 4 5040
printf "CHECK new_enter_cannot_replay_mechanics=%d\n", accepted_context.action_id==11 && claw_step_ms==$open_start && state==TASK_OPEN_FOR_RAM
set $open_finish=$open_start+claw_wait_ms
call Task_Process($open_finish)
printf "CHECK timeout_exit_enter_done=%d\n", state==TASK_EXIT_SAFE_ZONE && accepted_context.opcode==5 && task_context_complete() && !task_status.gripper_closed && first_delivery_done

begin_delivery
send_command 1 9 11 2 2000
printf "CHECK pause_keeps_used1000=%d\n", mission_paused && delivery.front_elapsed_ms==1000
call Task_Process(9000)
printf "CHECK pause_excluded=%d\n", mission_paused && delivery.front_elapsed_ms==1000 && state==TASK_NAVIGATE
send_command 5 75 11 3 10000
call Task_Process(12999)
printf "CHECK resume_not_new_budget=%d\n", !mission_paused && delivery.front_elapsed_ms==3999 && state==TASK_NAVIGATE
call Task_Process(13000)
printf "CHECK resumed_budget_timeout=%d\n", delivery.finish==DELIVERY_FINISH_FRONT_TIMEOUT && state==TASK_OPEN_FOR_RAM

begin_delivery
send_command 0 9 12 2 2000
send_command 10 9 12 3 3000
call Task_Process(9000)
printf "CHECK manual_stop_no_auto_open=%d\n", state==TASK_NAVIGATE && delivery.front_elapsed_ms==1000 && delivery.finish==DELIVERY_FINISH_NONE && reverse_output==0
send_command 5 75 13 4 10000
printf "CHECK stop_resume_retains_budget=%d\n", accepted_context.opcode==5 && accepted_context.action_id==13 && delivery.front_elapsed_ms==1000 && reverse_output>0
call Task_Process(13000)
printf "CHECK stop_resume_timeout=%d\n", state==TASK_OPEN_FOR_RAM && delivery.finish==DELIVERY_FINISH_FRONT_TIMEOUT

begin_delivery
send_command 10 9 11 2 2000
send_command 10 9 11 3 5000
printf "CHECK hold_cannot_reset_or_block_timeout=%d\n", state==TASK_OPEN_FOR_RAM && delivery.finish==DELIVERY_FINISH_FRONT_TIMEOUT && claw_action==CLAW_ACTION_OPEN
set $open_finish=claw_step_ms+claw_wait_ms
call Task_Process($open_finish)
printf "CHECK hold_does_not_stall_local_open=%d\n", state==TASK_EXIT_SAFE_ZONE && !task_status.gripper_closed && task_context_complete()

begin_delivery
set test_pose.path_mm=400
call Task_Process(2500)
printf "CHECK front400_switches_to_original_final=%d\n", nav_final_push_active && !delivery.front_budget_active && delivery.finish==DELIVERY_FINISH_NONE && nav_final_push_started_ms==2500
call Task_Process(3999)
printf "CHECK final1499_running=%d\n", state==TASK_NAVIGATE && nav_final_push_active
call Task_Process(4000)
printf "CHECK final1500_normal_open=%d\n", state==TASK_OPEN_FOR_RAM && delivery.finish==DELIVERY_FINISH_NONE && accepted_context.opcode==5
call Task_Process(4020)
set $open_finish=claw_step_ms+claw_wait_ms
call Task_Process($open_finish)
call Task_Process($open_finish+300)
printf "CHECK normal_path_still_mode15=%d\n", state==TASK_RAM_VERIFY && task_context_complete() && !first_delivery_done
send_command 6 9 12 2 $open_finish+301
printf "CHECK normal_complete_no_extra_second=%d\n", state==TASK_EXIT_SAFE_ZONE && first_delivery_done && task_context_complete()




begin_delivery
send_command 6 9 11 2 1100
printf "CHECK same_action_opcode_change_rejected=%d\n", accepted_context.opcode==5 && state==TASK_NAVIGATE
send_command 6 9 10 3 1120
printf "CHECK old_action_rejected=%d\n", accepted_context.action_id==11 && state==TASK_NAVIGATE
set test_vision.mission=active_mission
set test_vision.mission.command=6
set test_vision.mission.flags=9
set test_vision.mission.task_id=8
set test_vision.mission.action_id=12
set test_vision.mission.generation=4
set test_vision.mission.tick_ms=1140
call Task_Process(1140)
printf "CHECK other_task_rejected=%d\n", accepted_context.task_id==7 && accepted_context.opcode==5 && state==TASK_NAVIGATE
send_command 6 1 12 5 1160
printf "CHECK wrong_side_rejected=%d\n", accepted_context.opcode==5 && delivery.finish==DELIVERY_FINISH_NONE
set test_vision.mission=active_mission
set test_vision.mission.command=6
set test_vision.mission.flags=9
set test_vision.mission.action_id=12
set test_vision.mission.target_x_mm=1
set test_vision.mission.generation=6
set test_vision.mission.tick_ms=1180
call Task_Process(1180)
printf "CHECK malformed_complete_rejected=%d\n", accepted_context.opcode==5 && delivery.finish==DELIVERY_FINISH_NONE

begin_delivery
set test_pose.path_mm=400
call Task_Process(5000)
printf "CHECK reached400_at_deadline_uses_final=%d\n", state==TASK_NAVIGATE && nav_final_push_active && !delivery.front_budget_active && delivery.finish==DELIVERY_FINISH_NONE
call Task_Process(6500)
call Task_Process(6520)
set $original_open_ms=claw_step_ms
send_command 6 9 12 2 6540
printf "CHECK complete_during_open_preserves_servo_epoch=%d\n", state==TASK_OPEN_FOR_RAM && accepted_context.opcode==6 && claw_step_ms==$original_open_ms && !task_context_complete()
send_command 6 9 13 3 6560
printf "CHECK second_complete_action_cannot_replace_owner=%d\n", accepted_context.action_id==12 && claw_step_ms==$original_open_ms
send_command 1 9 12 4 6580
call Task_Process(16580)
printf "CHECK pause_during_release_not_done=%d\n", mission_paused && state==TASK_OPEN_FOR_RAM && !task_context_complete() && !first_delivery_done
send_command 6 9 12 5 16600
printf "CHECK resume_release_no_reopen=%d\n", !mission_paused && state==TASK_EXIT_SAFE_ZONE && !task_status.gripper_closed && first_delivery_done && task_context_complete()

begin_delivery
call Task_Process(5000)
set test_vision.mission=active_mission
set test_vision.mission.command=0
set test_vision.mission.flags=9
call wire_send_action(test_vision.mission, 12, 5020, 2)
set test_vision.config_ready=0
call Task_Process(10000)
printf "CHECK stop_during_local_open_cancels=%d\n", state==TASK_STOPPED && !delivery.active && task_status.gripper_closed && !first_delivery_done

set $blue=1
begin_delivery
printf "CHECK blue_formal_enter=%d\n", state==TASK_NAVIGATE && delivery.active && nav_locked_heading_deg==270
send_command 6 1 12 2 1100
set $finish=claw_step_ms+claw_wait_ms
call Task_Process($finish)
printf "CHECK blue_early_completion=%d\n", state==TASK_EXIT_SAFE_ZONE && accepted_context.opcode==6 && !task_status.gripper_closed && task_context_complete()
quit




