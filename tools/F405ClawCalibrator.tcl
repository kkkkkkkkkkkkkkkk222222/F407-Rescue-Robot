# Interactive calibration only. Reuse the running DMA pulse table; no halt,
# reset, flash, clock changes, or writes to other servo/motor channels.
init

proc f405_claw_addresses {} {
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
  if {[mrw [expr {$ticks+4}]] != 5100 ||
      [mrw [expr {$ticks+12}]] != 10100 ||
      [mrw [expr {$ticks+20}]] != 15100 ||
      [mrw [expr {$ticks+28}]] != 100 ||
      [mrw [expr {$edges+8}]] != 1 ||
      [mrw [expr {$edges+12}]] != 0x10000 ||
      [mrw [expr {$edges+24}]] != 4 ||
      [mrw [expr {$edges+28}]] != 0x40000} {
    error "Both claw channels must be enabled; use the old closed-pose button first"
  }
  return [list [expr {$ticks+24}] [expr {$ticks+8}]]
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
