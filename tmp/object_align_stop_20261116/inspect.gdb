set pagination off
set confirm off
start
call prepare()
set state=TASK_APPROACH
set approach_phase=APPROACH_TRACK
set cluster_target_active=0
set test_imu.gyro_mdps[2]=0
set test_brake_active=0
set test_vision.x=512
set test_vision.tick_ms=300
set test_vision.report_generation=1
call task_track_target(&test_vision, 0)
set steering_mm_s=130
call task_approach_align_output(1)
printf "CHECK breakaway130=%d\n", steering_mm_s == 130 && !approach_align.turned
set test_vision.report_generation=2
set test_vision.tick_ms=340
call task_track_target(&test_vision, 0)
call task_approach_align_output(1)
printf "CHECK new_report_preserves_breakaway=%d\n", steering_mm_s == 130 && !approach_align.turned
set test_imu.gyro_mdps[2]=5000
call task_approach_align_output(0)
printf "CHECK moving_has_no130_floor=%d\n", approach_align.turned && steering_mm_s > 50 && steering_mm_s < 60 && !approach_align.braking
set test_vision.x=540
set test_vision.report_generation=3
set test_vision.tick_ms=380
call task_track_target(&test_vision, 0)
set $stops=test_stop_calls
call task_approach_align_output(1)
printf "CHECK predicted_real_brake_before_window=%d\n", approach_align.braking && test_stop_calls == $stops+1 && steering_mm_s == 0 && yaw_output == 0 && approach_align.raw_error_px > 64
set test_brake_active=1
set test_imu.gyro_mdps[2]=0
set $stops=test_stop_calls
call task_approach_align_output(0)
printf "CHECK existing_pulse_not_restarted=%d\n", approach_align.braking && test_stop_calls == $stops
set test_brake_active=0
set test_imu.gyro_mdps[2]=5000
call task_approach_align_output(0)
printf "CHECK residual_rotation_braked_again=%d\n", test_stop_calls == $stops+1 && approach_align.braking
set test_imu.gyro_mdps[2]=0
call task_approach_align_output(0)
printf "CHECK old_picture_cannot_rekick=%d\n", approach_align.braking && steering_mm_s == 0
set test_vision.report_generation=4
set test_vision.tick_ms=500
call task_track_target(&test_vision, 0)
call task_approach_align_output(1)
printf "CHECK stopped_new_error_can_restart=%d\n", !approach_align.braking && !approach_align.turned && steering_mm_s == 8
call task_reset_tracking()
printf "CHECK reset_clears_alignment_state=%d\n", !approach_align.sample_valid && !approach_align.turned && !approach_align.braking && approach_align.closing_px_s == 0
set test_vision.x=768
set test_vision.report_generation=5
set test_vision.tick_ms=600
call task_track_target(&test_vision, 0)
set steering_mm_s=-130
set test_imu.gyro_mdps[2]=-5000
call task_approach_align_output(1)
printf "CHECK mirrored_running=%d\n", approach_align.turned && steering_mm_s < -50 && steering_mm_s > -60
set test_vision.x=512
set test_vision.report_generation=6
set test_vision.tick_ms=640
call task_track_target(&test_vision, 0)
set $stops=test_stop_calls
call task_approach_align_output(1)
printf "CHECK reverse_brakes_before_drive=%d\n", approach_align.braking && test_stop_calls == $stops+1 && yaw_output == 0
set $case=0
while $case < 4
call prepare()
call task_enter(TASK_APPROACH, 0)
set locked_cargo_counts=1
set cluster_claws_ready=1
set cluster_target_active=($case == 1)
if $case == 2
set approach_phase=APPROACH_CLUSTER_ALIGN
end
if $case == 3
set approach_phase=APPROACH_ALIGN_125
end
set $oldphase=approach_phase
set test_vision.x=640
set test_vision.y=512
set test_vision.valid=1
set test_vision.found=1
set test_vision.classification_valid=1
set test_vision.cargo_counts=1
set test_vision.tick_ms=300
set test_vision.report_generation=20
set test_imu.gyro_mdps[2]=40000
set test_brake_active=0
call task_process_approach(&test_vision, 300)
printf "CHECK phase%d_high_rate_no_handoff=%d\n", $case, approach_phase == $oldphase && approach_align.braking && yaw_output == 0
set test_imu.gyro_mdps[2]=0
call task_process_approach(&test_vision, 320)
printf "CHECK phase%d_old_frame_no_handoff=%d\n", $case, approach_phase == $oldphase
set test_vision.report_generation=21
set test_vision.tick_ms=340
set test_brake_active=1
call task_process_approach(&test_vision, 340)
printf "CHECK phase%d_active_brake_no_handoff=%d\n", $case, approach_phase == $oldphase
set test_vision.report_generation=22
set test_vision.tick_ms=400
set test_brake_active=0
call task_process_approach(&test_vision, 400)
printf "CHECK phase%d_stopped_handoff=%d\n", $case, ($case < 2 && approach_phase == APPROACH_RANGE_WAIT) || ($case == 2 && approach_phase == APPROACH_CLUSTER_CAMERA_TO_140) || ($case == 3 && approach_phase == APPROACH_CAMERA_TO_140)
set $case=$case+1
end
call prepare()
set state=TASK_ALIGN_SAFE_ZONE
set configured_color=0x11
set nav_ready=0
set test_imu.gyro_mdps[2]=10000
set test_pose.heading_mdeg=86000
call task_process_safe_align(2000)
printf "CHECK safe_zone_unchanged=%d\n", yaw_output == -50 && !nav_ready
quit
