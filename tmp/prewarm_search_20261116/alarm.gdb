set pagination off
set confirm off
start

define sound_frame
call (void *)memset(sound_payload, 0, 8)
set sound_payload[0]=$arg0
set sound_payload[1]=$arg1
set sound_payload[2]=$arg2
call vision_build_frame(sound_wire, 0x20, 0, sound_payload)
call Vision_ParseBytes(sound_wire, 15, $arg3)
end

call Vision_Init()
call BoardF405_ProcessBeep(0)
printf "CHECK boot_silent=%d\n", test_sound_pin==0 && test_sound_writes==0 && !operator_alarm_requested
sound_frame 2 1 0 1
call BoardF405_ProcessBeep(10)
printf "CHECK alarm_starts=%d\n", operator_alarm_requested && test_sound_pin==1
set $writes=test_sound_writes
sound_frame 2 1 0 40
call BoardF405_ProcessBeep(40)
sound_frame 1 4 1 50
call BoardF405_ProcessBeep(50)
printf "CHECK repeated_on_and_v1_preserve_phase=%d\n", test_sound_writes==$writes && operator_alarm_requested
call BoardF405_ProcessBeep(130)
printf "CHECK alarm_on120=%d\n", test_sound_pin==0
call BoardF405_ProcessBeep(190)
printf "CHECK alarm_gap60=%d\n", test_sound_pin==1
set $n=1
while $n <= 20
call BoardF405_ProcessBeep(190+($n-1)*180+120)
set $off=test_sound_pin==0
call BoardF405_ProcessBeep(190+$n*180)
if !$off || test_sound_pin!=1
printf "CHECK sustained_alarm=0\n"
end
set $n=$n+1
end
printf "CHECK alarm_continues_past_four_beeps=%d\n", operator_alarm_requested && test_sound_pin==1
sound_frame 2 0 0 3800
call BoardF405_ProcessBeep(3800)
printf "CHECK silence_before_next_edge=%d\n", !operator_alarm_requested && test_sound_pin==0
call BoardF405_ProcessBeep(10000)
printf "CHECK no_old_query_replay=%d\n", test_sound_pin==0
sound_frame 1 2 0 10010
call BoardF405_ProcessBeep(10010)
set $v1a=test_sound_pin==1
call BoardF405_ProcessBeep(10110)
set $v1b=test_sound_pin==0
call BoardF405_ProcessBeep(10260)
set $v1c=test_sound_pin==1
call BoardF405_ProcessBeep(10360)
call BoardF405_ProcessBeep(10510)
printf "CHECK v1_double_short_preserved=%d\n", $v1a && $v1b && $v1c && test_sound_pin==0
sound_frame 1 1 1 11000
call BoardF405_ProcessBeep(11000)
call BoardF405_ProcessBeep(11349)
set $long=test_sound_pin==1
call BoardF405_ProcessBeep(11350)
printf "CHECK v1_long350_preserved=%d\n", $long && test_sound_pin==0
sound_frame 2 1 0 12000
call BoardF405_ProcessBeep(12000)
sound_frame 2 0 1 12010
sound_frame 2 2 0 12020
sound_frame 3 0 0 12030
printf "CHECK invalid_version_operation_reserved_ignored=%d\n", operator_alarm_requested && beep_request==0
call (void *)memset(sound_payload, 0, 8)
set sound_payload[0]=2
call vision_build_frame(sound_wire, 0x20, 0, sound_payload)
set sound_wire[12]=sound_wire[12]^1
call Vision_ParseBytes(sound_wire, 15, 12040)
printf "CHECK bad_crc_cannot_silence=%d\n", operator_alarm_requested && beep_request==0
call (void *)memset(sound_payload, 0, 8)
set sound_payload[0]=0x99
set sound_payload[1]=1
call vision_build_frame(sound_wire, 0x11, 1, sound_payload)
call Vision_ParseBytes(sound_wire, 15, 12100)
printf "CHECK invalid_config_does_not_clear=%d\n", !latest_data.config_ready && operator_alarm_requested
set sound_payload[0]=0x11
call vision_build_frame(sound_wire, 0x11, 2, sound_payload)
call Vision_ParseBytes(sound_wire, 15, 12200)
call BoardF405_ProcessBeep(12200)
printf "CHECK accepted_config_clears_alarm=%d\n", latest_data.config_ready && latest_data.session_generation==1 && !operator_alarm_requested && test_sound_pin==0
sound_frame 2 1 0 12300
call BoardF405_ProcessBeep(12300)
call (void *)memset(sound_payload, 0, 8)
set sound_payload[0]=0x11
set sound_payload[1]=1
call vision_build_frame(sound_wire, 0x11, 3, sound_payload)
call Vision_ParseBytes(sound_wire, 15, 12310)
printf "CHECK ignored_config_repeat_not_new_match=%d\n", latest_data.session_generation==1 && operator_alarm_requested
sound_frame 2 0 0 12400
sound_frame 2 1 0 12401
sound_frame 2 0 0 12402
call BoardF405_ProcessBeep(12410)
printf "CHECK latest_silence_wins_pending_alarm=%d\n", !operator_alarm_requested && test_sound_pin==0 && beep_request==0
sound_frame 1 4 1 12500
sound_frame 2 0 0 12501
call BoardF405_ProcessBeep(12510)
call BoardF405_ProcessBeep(15000)
printf "CHECK silence_cancels_pending_v1=%d\n", test_sound_pin==0

call Vision_Init()
call (void *)memset(sound_payload, 0, 8)
set sound_payload[1]=7
set sound_payload[3]=11
call vision_build_frame(sound_wire, 0x1d, 42, sound_payload)
call Vision_ParseBytes(sound_wire, 15, 20000)
set $stamp=latest_data.last_frame_tick_ms
set $streak=config_streak
sound_frame 2 1 0 20001
sound_frame 1 2 0 20002
sound_frame 2 0 1 20003
printf "CHECK audio_preserves_pending_pair=%d\n", context_pending && context_task==7 && context_action==11 && context_sequence==42 && latest_data.last_frame_tick_ms==$stamp && config_streak==$streak && mission_generation==0 && ack_remaining==0
call (void *)memset(sound_payload, 0, 8)
set sound_payload[1]=1
call vision_build_frame(sound_wire, 0x18, 42, sound_payload)
call Vision_ParseBytes(sound_wire, 15, 20004)
printf "CHECK mission_pair_still_accepted=%d\n", latest_data.mission.context_valid && latest_data.mission.payload_valid && latest_data.mission.task_id==7 && latest_data.mission.action_id==11 && latest_data.mission.sequence==42 && mission_generation==1 && !context_pending
quit

