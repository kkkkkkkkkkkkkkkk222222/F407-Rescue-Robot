tcl port disabled
gdb port disabled
telnet port disabled
init

set ticks [mrw 0x4002601c]
set edges [mrw 0x400264a8]
if {([mrw 0xe0042000] & 0xfff) != 0x413 ||
    ([mrw 0x40000800] & 1) == 0 || ([mrw 0x4000080c] & 0x200) == 0 ||
    [mrw 0x40000828] != 83 || [mrw 0x4000082c] != 19999 ||
    ([mrw 0x40026010] & 1) == 0 || [mrw 0x40026018] != 0x40000834 ||
    ([mrw 0x400264a0] & 1) == 0 || [mrw 0x400264ac] != 0x40020818 ||
    $ticks < 0x20000000 || $ticks > 0x2001ffe0 || ($ticks & 3) ||
    $edges < 0x20000000 || $edges > 0x2001ffe0 || ($edges & 3)} {
  error "Active F405 PWM schedule does not match the expected driver"
}
if {[mrw [expr {$ticks+4}]] != 5100 || [mrw [expr {$ticks+12}]] != 10100 ||
    [mrw [expr {$ticks+20}]] != 15100 || [mrw [expr {$ticks+28}]] != 100 ||
    [mrw [expr {$edges+8}]] != 1 || [mrw [expr {$edges+12}]] != 0x10000 ||
    [mrw [expr {$edges+24}]] != 4 || [mrw [expr {$edges+28}]] != 0x40000} {
  error "Claw GPIO slots do not match"
}

set left_fall [expr {$ticks+24}]
set original [mrw $left_fall]
set right_pulse [expr {[mrw [expr {$ticks+8}]]-5100}]
# Each pulse microsecond is 0.09 degrees, requiring 0.18 ms at the requested rate limit.
if {$original < 15600 || $original > 17600 || $right_pulse < 500 || $right_pulse > 2500 ||
    $start_pulse < 500 || $start_pulse > 2500 || $end_pulse < 500 || $end_pulse > 2500 ||
    $step_us < 1 || $step_us > 2000 || $interval_ms < 20 || $step_us*9 > $interval_ms*50 ||
    $hold_ms < 0 || $cycles < 1} {
  error "Claw pulse or sweep parameters are outside the expected range"
}

proc wait_step {ms flag} {
  while {$ms > 0} {
    if {[file exists $flag]} { return 0 }
    set part [expr {$ms < 50 ? $ms : 50}]
    sleep $part
    set ms [expr {$ms-$part}]
  }
  return [expr {![file exists $flag]}]
}

proc slew {address goal step interval flag} {
  set value [mrw $address]
  while {$value != $goal} {
    if {[file exists $flag]} { return 0 }
    if {$value < $goal} {
      set next [expr {$value+$step > $goal ? $goal : $value+$step}]
    } else {
      set next [expr {$value-$step < $goal ? $goal : $value-$step}]
    }
    mww $address $next
    set value [mrw $address]
    if {$value != $next} { error "S4 target was overwritten during the sweep" }
    if {![wait_step $interval $flag]} { return 0 }
  }
  return 1
}

proc report_stop {address} {
  echo [format "STOPPED S4_PULSE_US=%u" [expr {[mrw $address]-15100}]]
  shutdown
}

set start [expr {15100+$start_pulse}]
set end [expr {15100+$end_pulse}]
echo [format "SETUP S2_LOGICAL=90 S2_PULSE_US=%u S4_ORIGINAL_PULSE_US=%u" $right_pulse [expr {$original-15100}]]
if {![slew $left_fall $start $step_us $interval_ms $stop_file]} { report_stop $left_fall; return }
for {set cycle 1} {$cycle <= $cycles} {incr cycle} {
  echo [format "CYCLE %u/%u" $cycle $cycles]
  if {![slew $left_fall $end $step_us $interval_ms $stop_file]} { report_stop $left_fall; return }
  echo [format "HOLD END S4_PULSE_US=%u MS=%u" $end_pulse $hold_ms]
  if {![wait_step $hold_ms $stop_file]} { report_stop $left_fall; return }
  if {![slew $left_fall $start $step_us $interval_ms $stop_file]} { report_stop $left_fall; return }
  echo [format "HOLD START S4_PULSE_US=%u MS=%u" $start_pulse $hold_ms]
  if {![wait_step $hold_ms $stop_file]} { report_stop $left_fall; return }
}
if {![slew $left_fall $original $step_us $interval_ms $stop_file]} { report_stop $left_fall; return }
echo [format "DONE S4_RESTORED_PULSE_US=%u" [expr {$original-15100}]]
shutdown
