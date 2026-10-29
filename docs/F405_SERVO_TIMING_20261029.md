# F405 Servo Timing Measurement, 2026-10-29

NormalDebug motion service adds two non-motion commands:

- `SPWM`: report timing for S1 lift, S2 right claw, S3 camera and S4 left claw.
- `SPWM 0`: clear timing statistics. Angles and PWM outputs are unchanged.

Diagnostics are compiled only for the F405 motion-debug service. The task
build keeps its existing servo driver without measurement instrumentation.
No motor settings or servo PWM scheduling were changed by this diagnostic.

The TIM4 handler samples its 1 us counter immediately before writing GPIOC
BSRR. It records high-pulse widths, rising/falling interrupt delays and
coalesced edges. DWT cycle timestamps measure rising-edge periods. Changes
of angle discard initial transition frames before collecting new samples.

These are software edge timestamps, not oscilloscope measurements at a
servo connector. They do not measure supply voltage, signal amplitude,
ground noise, servo shaft position or mechanical vibration. Counter
resolution and the short interval before the BSRR write limit accuracy.

## Stationary Test

Flashed build/NormalDebug/WWW.elf and verified using CMSIS-DAP. Motors stayed
stopped. Restored S1=60, S2=147, S3=90 and S4=23 after all stages.
Log: build/servo_timing_20261029_193934.log (local generated artifact).

| Stage | Channel / angle | Software high width, us |
| --- | --- | --- |
| Closed, idle | S1 / 60 | 1159..1173 |
| Closed, idle | S2 / 147 | 2126..2140 |
| Closed, idle | S3 / 90 | 1493..1507 |
| Closed, idle | S4 / 23 | 748..762 |
| MLOG enabled | S4 / 23 | 748..762 |
| Touch, both claws | S2 / 104 | 1648..1662 |
| Touch, both claws | S4 / 76 | 1337..1351 |
| Open, both claws | S2 / 72 | 1293..1304 |
| Open, both claws | S4 / 108 | 1693..1707 |
| Right signal off | S4 / 76 | 1337..1351 |
| Left signal off | S2 / 104 | 1648..1662 |

Idle rising periods were 19993..20006 us. Maximum observed rising and falling
delays were 8 us. Each stage contained hundreds of measured pulses; no
coalesced rising/falling edge was recorded. Disabling one signal does not
guarantee that its particular servo releases holding torque.

The tests confirm software pulse-width variation at fixed targets. They
do not establish that timing is the sole cause of the reported left-claw
vibration; physical observations and electrical measurements remain separate.
