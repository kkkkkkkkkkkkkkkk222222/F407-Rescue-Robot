set pagination off
set confirm off
start
call prepare()
set first_delivery_done=0
set search_yaw_direction=1
call task_enter(TASK_SEARCH, 0)
set search_phase=SEARCH_SWEEP_120
call task_process_search(&test_vision, 1000)
printf "CHECK initial_ccw=%d\n", search_yaw_direction==1 && yaw_output==-150
set $trip=1
while $trip <= 4
call task_enter(TASK_OPEN_FOR_RAM, $trip*1000)
set task_status.gripper_closed=0
set locked_cargo_counts=2
set delivery.finish=($trip % 3)
call task_finish_delivery_exit($trip*1000+500)
set $expected=($trip % 2) ? 1 : -1
printf "CHECK trip%d_direction=%d\n", $trip, search_yaw_direction==$expected && state==TASK_EXIT_SAFE_ZONE
call task_finish_delivery_exit($trip*1000+501)
printf "CHECK trip%d_duplicate_no_flip=%d\n", $trip, search_yaw_direction==$expected
call task_enter(TASK_SEARCH, $trip*1000+600)
set search_phase=SEARCH_SWEEP_120
call task_reset_turn_tracker()
call task_process_search(&test_vision, $trip*1000+620)
printf "CHECK trip%d_high_sweep=%d\n", $trip, yaw_output==-150*$expected
set search_phase=SEARCH_SWEEP_90
call task_reset_turn_tracker()
call task_process_search(&test_vision, $trip*1000+640)
printf "CHECK trip%d_low_sweep=%d\n", $trip, yaw_output==-150*$expected
set search_phase=SEARCH_WAIT_RETURN
call task_process_search(&test_vision, $trip*1000+660)
printf "CHECK trip%d_wait_scan=%d\n", $trip, yaw_output==-150*$expected
call task_enter(TASK_APPROACH_RECOVER, $trip*1000+680)
call task_enter(TASK_SEARCH, $trip*1000+700)
printf "CHECK trip%d_recovery_preserves=%d\n", $trip, search_yaw_direction==$expected
set $trip=$trip+1
end
call task_reset_turn_tracker()
set test_pose.heading_mdeg=0
call task_full_turn_reached()
set test_pose.heading_mdeg=270000
call task_full_turn_reached()
set test_pose.heading_mdeg=180000
call task_full_turn_reached()
set test_pose.heading_mdeg=90000
call task_full_turn_reached()
set test_pose.heading_mdeg=0
set $turn_done=task_full_turn_reached()
printf "CHECK clockwise_full_turn_count=%d\n", $turn_done
call task_initialize(10000)
printf "CHECK new_match_resets_direction=%d\n", search_yaw_direction==1 && !first_delivery_done
quit
