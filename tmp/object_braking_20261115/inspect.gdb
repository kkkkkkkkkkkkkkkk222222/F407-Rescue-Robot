set pagination off
set confirm off
start
set test_imu.gyro_mdps[2] = 0
call prepare()
set state=TASK_APPROACH
set approach_phase=APPROACH_TRACK
set cluster_target_active=0
set test_vision.x=448
set test_vision.report_generation=1
set test_vision.tick_ms=1000
call task_track_target(&test_vision, 0)
printf "CHECK visual_far140=%d\n", steering_target_mm_s == 140.0f
set test_vision.x=512
set test_vision.report_generation=2
set test_vision.tick_ms=1040
call task_track_target(&test_vision, 0)
printf "CHECK visual_mid135=%d\n", steering_target_mm_s == 135.0f
set test_vision.x=575
set test_vision.report_generation=3
set test_vision.tick_ms=1080
call task_track_target(&test_vision, 0)
printf "CHECK visual_edge130=%d\n", steering_target_mm_s >= 130.0f && steering_target_mm_s < 131.0f
set steering_mm_s=140
set test_imu.gyro_mdps[2]=5000
call task_approach_steering()
printf "CHECK brake_between_frames=%d\n", steering_mm_s > 100.0f && steering_mm_s < 101.0f
set test_imu.gyro_mdps[2]=-5000
call task_approach_steering()
printf "CHECK opposite_rate_no_boost=%d\n", steering_mm_s <= steering_target_mm_s && steering_mm_s > 100.0f
set test_vision.x=705
set test_vision.report_generation=4
set test_vision.tick_ms=1120
call task_track_target(&test_vision, 0)
printf "CHECK reverse_clears_slew=%d\n", steering_mm_s == 0 && steering_target_mm_s < -130 && steering_target_mm_s > -131
set test_vision.x=640
set test_vision.report_generation=5
set test_vision.tick_ms=1160
call task_track_target(&test_vision, 0)
printf "CHECK actual_center_zero=%d\n", steering_mm_s == 0 && steering_target_mm_s == 0
call task_reset_tracking()
set approach_phase=APPROACH_CLUSTER_ALIGN
set test_vision.x=520
set test_vision.report_generation=6
call task_track_target(&test_vision, 0)
printf "CHECK cluster_max140=%d\n", steering_target_mm_s == 140
call task_reset_tracking()
set approach_phase=APPROACH_ALIGN_125
set test_vision.x=568
set test_vision.report_generation=7
call task_track_target(&test_vision, 0)
printf "CHECK align125_bounds=%d\n", steering_target_mm_s >= 130 && steering_target_mm_s <= 140
call prepare()
set state=TASK_ALIGN_SAFE_ZONE
set configured_color=0x11
set nav_ready=0
set test_imu.gyro_mdps[2]=0
set test_pose.heading_mdeg=0
call task_process_safe_align(2000)
printf "CHECK safe90_max250=%d\n", yaw_output == -250 && !nav_ready
set test_pose.heading_mdeg=70000
call task_process_safe_align(2020)
printf "CHECK safe_early_decel=%d\n", yaw_output == -228 && !nav_ready
set test_pose.heading_mdeg=82000
call task_process_safe_align(2040)
printf "CHECK safe_min130=%d\n", yaw_output == -130 && !nav_ready
set test_pose.heading_mdeg=86000
set test_imu.gyro_mdps[2]=10000
call task_process_safe_align(2060)
printf "CHECK safe_gyro_envelope=%d\n", yaw_output == -50 && !nav_ready
set test_pose.heading_mdeg=86800
call task_process_safe_align(2080)
printf "CHECK safe_edge_soft_brake=%d\n", yaw_output < -9.9 && yaw_output > -10.1 && !nav_ready
set test_pose.heading_mdeg=94000
set test_imu.gyro_mdps[2]=-10000
call task_process_safe_align(2100)
printf "CHECK safe_mirrored_brake=%d\n", yaw_output == 50 && !nav_ready
set test_pose.heading_mdeg=90000
call task_process_safe_align(2120)
printf "CHECK safe_actual_arrival=%d\n", yaw_output == 0 && nav_ready
quit



