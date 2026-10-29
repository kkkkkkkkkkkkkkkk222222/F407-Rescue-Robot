# F405 Debug Lateral Tuning, 2026-10-29

Scope: NormalDebug with APP_NORMALDEBUG_MOTION_TEST=ON. These changes are
guarded by APP_BOARD_F405_ROBO and APP_ENABLE_MOTION_DEBUG_TASK. The task
build, ordinary MOVE, TURN and the old-board MotionDebug retain their gains.

## Accepted Control

- Preserve the measured drive, encoder, forward and gyro polarities.
- Lateral wheel-speed P is 15; I remains 0.12 and D remains zero.
- When a breakaway pulse produces measured motion below target speed,
  transfer its output into the wheel PI integral, within existing bounds.
- Remove the per-sample 15% integral decay. Ordinary PI handles overspeed.
- Keep the existing heading PI (P=2.2, I=0.005) in wheel-speed targets.
- In addition, raw heading error requests a gyro rate at 3 degrees/second
  per degree of error, limited to +/-30 degrees/second.
- Gyro rate error acts on common wheel PWM at 2 permille per degree/second,
  converted with APP_MOTOR_HEADING_OUTPUT_SIGN, limited to +/-80 permille.
- Apply the existing wheel-direction restrictions and 60% overall PWM cap
  after adding this common effort. All assistance clears at stop.
- Retain the existing startup tracking check and bounded breakaway pulses.

MLOG adds I100 (three wheel integral contributions), YAW_PWM, ERR_MDEG and
REF_MDPS. YAW_PWM is the requested common correction before per-wheel clamps;
PWM contains the actual final wheel commands. Motion record format is 10.

## Physical Validation

NormalDebug was built, flashed and verified with CMSIS-DAP. The old-board
MotionDebug build also passed. No new host self-test program was added.

Logs are local generated artifacts in build/:

- lateral_polarity_20261029_191612.log: common positive wheel targets reduced
  raw gyro yaw; negative targets increased it. Initial lateral failure showed
  a front wheel near zero while the rear kept moving.
- lateral_rate_pwm_20261029_192746.log: 200 mm left/right, requested 300 mm/s.
  Settled raw yaw changes were approximately +2.5 and -5.7 degrees before
  separate return-to-heading commands.
- lateral_rate_pwm_20261029_192825.log: three 300 mm lateral moves completed
  at requested 400 mm/s. The operator accepted the observed behavior and
  requested no further tuning. The queued fourth lateral move was not run.

Return-to-heading commands belong to the host test procedure; BODY itself
still ends from odometry distance progress. A DONE reply does not prove
floor displacement or absence of wheel slip. No automatic permanent fault
latch, final automatic heading turn or new serial command was introduced.
