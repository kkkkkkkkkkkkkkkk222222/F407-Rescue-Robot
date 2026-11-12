# Interactive calibration only. Reuse the running four-servo DMA pulse table;
# no halt, reset, flash, clock changes, or motor writes.
init

proc f405_servo_table {} {
  set ticks [mrw 0x4002601c]
  set edges [mrw 0x400264a8]
  if {([mrw 0xe0042000] & 0xfff) != 0x413 ||
      ([mrw 0x40000800] & 1) == 0 ||
      [mrw 0x40000828] != 83 || [mrw 0x4000082c] != 19999 ||
      ([mrw 0x40026010] & 1) == 0 || [mrw 0x40026018] != 0x40000834 ||
      ([mrw 0x400264a0] & 1) == 0 || [mrw 0x400264ac] != 0x40020818 ||
      $ticks < 0x20000000 || $ticks > 0x2001ffe0 || ($ticks & 3) ||
      $edges < 0x20000000 || $edges > 0x2001ffe0 || ($edges & 3)} {
    error "Running PWM driver does not match the F405 DMA table"
  }
  set pins {8192 1 2 4}
  for {set i 0} {$i < 4} {incr i} {
    set rise [mrw [expr {$edges+8*$i}]]
    set fall [mrw [expr {$edges+8*$i+4}]]
    set next [expr {$i == 3 ? 100 : 5100+5000*$i}]
    if {[mrw [expr {$ticks+8*$i+4}]] != $next ||
        ($rise != 0 && $rise != [lindex $pins $i]) ||
        $fall != ([lindex $pins $i] << 16)} {
      error "Servo GPIO slots do not match the running DMA driver"
    }
  }
  return [list $ticks $edges]
}

proc f405_claw_addresses {} {
  lassign [f405_servo_table] ticks edges
  if {[mrw [expr {$edges+8}]] != 1 || [mrw [expr {$edges+24}]] != 4} {
    error "Both claw channels must be enabled; use the old closed-pose button first"
  }
  return [list [expr {$ticks+24}] [expr {$ticks+8}]]
}

proc servo_readall {} {
  if {[catch {
    lassign [f405_servo_table] ticks edges
    set result {OK}
    for {set i 0} {$i < 4} {incr i} {
      set pulse [expr {[mrw [expr {$ticks+8*$i}]]-(100+5000*$i)}]
      set enabled [expr {[mrw [expr {$edges+8*$i}]] != 0}]
      if {$pulse < 500 || $pulse > 2500} { error "Servo pulse outside 500..2500us" }
      lappend result $pulse $enabled
    }
  } reason]} { return "ERROR $reason" }
  return $result
}

proc servo_set {id pulse} {
  if {[catch {
    if {![string is integer -strict $id] || $id < 1 || $id > 4 ||
        ![string is integer -strict $pulse] || $pulse < 500 || $pulse > 2500} {
      error "Servo id must be1..4; pulse must be500..2500us"
    }
    lassign [f405_servo_table] ticks edges
    set i [expr {$id-1}]
    if {[mrw [expr {$edges+8*$i}]] == 0} { error "Enable this servo with the existing S command first" }
    set address [expr {$ticks+8*$i}]
    set start [expr {100+5000*$i}]
    mww $address [expr {$start+$pulse}]
    set actual [expr {[mrw $address]-$start}]
    if {$actual != $pulse} { error "Servo pulse table was overwritten" }
  } reason]} { return "ERROR $reason" }
  return "OK $id $actual"
}

proc claw_read {} {
  if {[catch {
    lassign [f405_claw_addresses] left right
    set lp [expr {[mrw $left]-15100}]
    set rp [expr {[mrw $right]-5100}]
    if {$lp < 500 || $lp > 2500 || $rp < 500 || $rp > 2500} {
      error "Current pulse is outside the existing 500..2500us output range"
    }
  } reason]} { return "ERROR $reason" }
  return "OK $lp $rp"
}

proc claw_set {lp rp} {
  if {[catch {
    if {![string is integer -strict $lp] || ![string is integer -strict $rp] ||
        $lp < 500 || $lp > 2500 || $rp < 500 || $rp > 2500} {
      error "Pulse must be an integer within 500..2500us"
    }
    lassign [f405_claw_addresses] left right
    mww $left [expr {15100+$lp}]
    mww $right [expr {5100+$rp}]
    set left_read [expr {[mrw $left]-15100}]
    set right_read [expr {[mrw $right]-5100}]
    if {$left_read != $lp || $right_read != $rp} {
      error "Pulse table was overwritten; check that the board is in maintenance"
    }
  } reason]} { return "ERROR $reason" }
  return "OK $left_read $right_read"
}
