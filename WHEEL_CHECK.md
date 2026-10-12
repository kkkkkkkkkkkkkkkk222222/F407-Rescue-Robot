# MotionDebug Serial Tests

This is a USART1 command inside the existing MotionDebug firmware.
There is no separate application mode or build preset. Motor/IMU parameters
are shared with NormalRun and Gamepad as documented in the calibration history
below; the text console itself remains exclusive to MotionDebug.

Build the existing MotionDebug preset and flash `build/MotionDebug/WWW.elf`.
The check never starts automatically on boot.

## Serial Motion Commands

The programmer's USART1 console is available in MotionDebug at 115200 8N1.
`VER` or `STAT` must report `MOTIONDEBUG SERIAL_V3`; the response also includes
the firmware build date/time. These commands are not added to NormalRun.

| Command | Meaning |
| --- | --- |
| `TURN 30 60` | Relative +30 degrees at 60 degrees/s requested speed |
| `TURN -30 60` | Relative -30 degrees at 60 degrees/s |
| `MOVE 200 150` | Body-forward 200 mm at 150 mm/s, with existing heading hold |
| `MOVE -200 150` | Body-backward 200 mm at 150 mm/s |
| `BODY 200 100 150` | Body forward/left displacement in mm, speed in mm/s |
| `FIELD 200 100 150` | Field-frame x/y displacement in mm, speed in mm/s |
| `STOP` | Immediate stop; abort a wheel sequence or motion from either input |
| `HOLD` | Stop and report DONE using the existing hold semantics |
| `RESET` | Reset the odometry reference while idle; not IMU zeroing |
| `STAT` / `STATE` | State, source, fault, IMU/odometry readiness, motor feedback |
| `GYRO` | Read-only raw IMU yaw and change since the previous query |
| `IMU` | Timestamp/integration diagnostics, bias, sampling gaps and scale |
| `PING` | Console liveness, RX/error/overflow/timeout and TX/drop counters |
| `MOTOR` | Wheel target/actual mm/s and actual signed drive PWM per mille |
| `ENC` | Raw cumulative encoder low 32 bits and last 10 ms deltas |
| `WHEELS` | The suspended-chassis sequence below |
| `HELP` / `VER` | Command help and firmware feature/build identification |

Angles and distances accept signed integers. Turn speed is degrees/s, not
mm/s. If omitted, turn speed defaults to 30 degrees/s and translation speed
to 150 mm/s. Explicit translation speeds must be 50..700 mm/s; the existing
10 m displacement limit is retained. Turn angles are limited to +/-360 degrees.

Motion commands are queued to the same Debug task and shared NormalRun motor
controllers used by USART3, not executed in UART interrupts. A second motion
while busy is rejected; STOP/HOLD can interrupt. During a serial action,
new USART3 motion commands are consumed and ignored, but USART3 STOP wins.
Commands received during the serial action are not replayed at completion.
Normal USART3 operation resumes after the action finishes. Existing binary
command/status formats are unchanged.

Each accepted serial motion reports its sequence, QUEUED/RUNNING state, final
DONE/STOPPED/FAULT, raw IMU yaw change since acceptance, and wheel feedback.
Fault names distinguish invalid requests, IMU not ready, invalid odometry,
timeout, and motor faults. Progress units follow existing MotionDebug status:
tenths of a degree for TURN and millimeters for translation. Speed is requested;
the existing end-of-turn slowdown and braking still apply.

For the next physical heading check, put the chassis on a clear floor, send
`TURN 30 60` once, and record the actual direction and complete output through
the `FINAL` line. Do not run WHEELS on the floor during this measurement.

## Procedure

1. Support the chassis securely with all wheels clear of the ground.
2. Keep hands, wires and loose objects away. Do not hold a wheel.
3. Open the programmer's USB serial port, connected to PCB serial port 2
   (USART1 PA9 TX / PA10 RX), at 115200, 8N1, no flow control.
4. Use text with CR or LF; turn OFF periodic/automatic sending.
5. Send `STAT` and verify the `MOTIONDEBUG` response.
6. Send `WHEELS` once. Port 1 starts after one second, then ports 2 and 3
   run in order. Each pulse is positive logical drive, open-loop 25%, for
   at most 400 ms, with a two-second gap after each result.
7. Send `STOP` at any point to cancel the entire sequence.

TIM6 bounds each pulse independently of console processing. Speed PID,
heading correction and wheel mixing are bypassed only during a pulse.
Other wheels receive no propulsion command. Existing braking then coasts.
The whole sequence also has a 10-second console timeout.

While testing, USART3 motion commands are consumed but do not run. USART3
STOP also cancels the sequence. After completion/cancellation, ordinary
MotionDebug commands resume; the pre-test action is not resumed, and commands
received during testing are not queued for later execution.

`RESULT W1+ RAW=a,b,c FB=x,y,z` reports encoder changes in port order 1,2,3,
including braking/settling. `RAW` is unmodified; `FB` uses the current encoder
polarity. The selected wheel's `FB` should be positive for each positive pulse.
Record the physical wheel that moved, the direction of its top tread relative
to the gripper/front, whether the other wheels moved, and each result line.
Positive individual wheel drive does not mean all three wheels propel the
chassis forward; this test is only for port mapping and feedback polarity.

## Heading Polarity

`GYRO` is read-only; it does not start/stop motion or zero the IMU. It reports
raw IMU yaw modulo one revolution and the unwrapped yaw change since the
previous successful `GYRO` query, in millidegrees (1000 = one degree).
The first query has `DELTA_VALID=0`; subsequent ready queries have valid deltas.

After checking wheel feedback, put the chassis on a clear floor. Query `GYRO`,
run the existing upper-computer positive 30-degree relative turn, then query
`GYRO` again after the chassis stops. Do not reset odometry/IMU between queries.
Record the actual chassis turn direction and both complete responses.

Positive rotation adds positive speed to all three wheel targets. For the
existing error formula (reference yaw minus current yaw), a positive measured
yaw response requires heading output sign +1; a negative response requires -1.
This check does not change that sign or any PID gain automatically.

The recorded positive-turn test produced +359563 mdeg. During the following
200 mm MOVE, positive yaw grew while the third-wheel heading correction grew
from 0 to +150 mm/s, ending at +165650 mdeg. This confirms that the previous
heading output sign -1 was positive feedback on the installed chassis.
The shared heading output sign is now +1; PID gains, PWM limits, the turn
controller and the startup MoveSpin task are unchanged. This calibration
also applies to NormalRun's shared distance/vector heading hold.

After flashing, send `GYRO` and verify `HEADING_SIGN=1`. On a clear floor,
run only `MOVE 200 150` first. Report the actual forward/backward direction,
heading drift and full ACCEPT-to-FINAL log before increasing distance/speed.
The previous MOVE also showed near-zero left-wheel feedback for negative
targets initially; recheck reverse wheel response if drift remains after
correcting the heading polarity. Do not change multiple parameters at once.

The follow-up 200 mm MOVE ended at -1027 mdeg (after an intermediate +8844
mdeg), confirming that heading error is now corrected rather than amplified.
The user observed physical backward travel for that positive MOVE. Forward
axis calibration is therefore now -1: gripper-forward drives M1 negative and
M2 positive. The mixer, encoder distance progress and Location forward
odometry use the same sign. Per-wheel motor/encoder signs and pure yaw are
unchanged; lateral polarity is left untouched pending a separate physical
check. This shared forward calibration affects MotionDebug, NormalRun and
Gamepad translations, not just the USART1 command parser.

After flashing, `STAT` must show `FORWARD_SIGN=-1 HEADING_SIGN=1`. On a clear
floor, send `MOVE 200 150` once and report whether it travels toward the
gripper, its approximate physical distance, heading drift and complete log.
Only after it finishes, `MOVE -200 150` checks the opposite direction.
Do not run the full NormalRun task until basic direction checks are complete.

## Softer MOVE heading correction (2026-10-12)

The user confirmed left/right hunting during MotionDebug MOVE. Shared heading
Kp is now 2 (was 4), Ki is 0.005 (was 0.02), and Kd stays zero. STAT/VER prints
HOLD_P_X100=200 and HOLD_I_X10000=50 to identify the flashed trial. These are
scaled parameter displays, not extra gains. Wheel-speed PID and feed-forward
are unchanged, as are Scheme A signs and the 60% propulsion cap.

Repeat MOVE 200 150 under the same conditions before increasing distance or
speed. Send the full motion log and report whether oscillation is reduced or
replaced by a steady curve. This trial does not change the TURN controller or
add automatic heading correction to Gamepad manual control.

## Serial recovery and input diagnosis (2026-10-12)

The user observed missing/truncated replies after input mistakes. The screenshot
also contained MOVE 600 850; 850 mm/s exceeds the unchanged 50..700 console
translation range. Range errors reject the request without latching a motion
fault. The screenshot alone cannot prove that a hardware UART error occurred.

The USART1 error callback is now routed separately from USART3. It latches the
error, disables RX interrupts and clears the F4 SR error flags (SR/DR read).
Main then aborts/rearms receive and discards the damaged partial line. Merely
calling HAL_UART_AbortReceive does not clear these hardware flags in this HAL
implementation. USART3 protocol and NormalRun/Gamepad callbacks are unchanged.

The console also tracks TX completion explicitly. If a 128-byte TX chunk has
not completed within 250 ms, it aborts TX, drops stale queued replies, stops
the diagnostic motion and restarts with an error notice. The TX ring is now
1024 bytes and full-queue drops are counted rather than silently unobservable.

A partial command with no new input for 1000 ms is discarded and stops motion;
it is never auto-executed without a newline. Parser recovery does not replay
old movement. A new valid line can then be accepted. On RX recovery a newline
resynchronizes the parser; waiting one second also clears the partial state.
Motor stall interlocks remain disabled; these are console communication guards.

STAT/VER now includes RX_GUARD=1. Each completed input is echoed as RXCMD before
its reply. PING returns PONG, uptime and counters: RX_BYTES, LINES, RX_ERRORS,
LAST_RX_ERROR (HAL bit mask), OVERFLOWS, REARM_FAIL, LINE_TIMEOUTS, TX_TIMEOUTS
and TX_DROPS. If HAL cleared the error code before callback delivery, the error
event is still latched and counted even if LAST_RX_ERROR is zero.

After flashing, enable text sending with CR/LF and disable cyclic sending.
Send STOP, PING and STAT separately, waiting for replies. Then, on a clear floor,
send MOVE 200 150 once. MOVE 600 850 should show RXCMD followed by an ERR SPEED
reply, with no movement; it does not permanently disable future valid commands.
Do not try to clear pending input by sending only a newline to an older build,
because that could execute the previously buffered movement command.

## Timestamp integration and visual steering trial (2026-10-12)

Wheel speed PI remains 6 / 0.15 / 0 and shared heading hold is 2 / 0.005 / 0.
NormalRun's visual chassis steering PD is now 0.45 / 0.01 (was 0.72 / 0.02),
with minimum active correction 20 mm/s (was 40). STAT displays
STEER_P_X1000=450 and STEER_D_X10000=100. Servo-camera PID, navigation heading
gain, PWM cap, signs and serial protocol are unchanged. Gamepad remains manual
velocity control without a new automatic heading lock.

IMU660RC yaw now uses sensor timestamps and trapezoidal integration with
fractional microdegree carry, longer startup zero-rate calibration and gated
stationary bias adaptation. Query IMU for ALG=TS_TRAPZ, DT_US, MAX_DT_US, GAPS,
clock calibration, bias and bias-update counters. This does not enable FIFO or
quaternion fusion; see docs/IMU660RC_YAW_IMPROVEMENTS.md for limitations.

GYRO now also prints TOTAL_MDEG, the unwrapped accumulated angle since boot or
IMU zeroing (display saturates at signed 32-bit millidegrees). The algorithm's
internal angle remains signed 64-bit. YAW_MOD_MDEG is modulo one revolution;
DELTA_MDEG is only the change since the previous GYRO query. Do not confuse
either of those fields with the cumulative multi-turn angle.

For a known-angle check, keep the chassis stationary during startup, wait for
READY, send IMU and GYRO, then manually rotate on a marked level surface by one
known full turn. Stop for two seconds, query GYRO and IMU, and record both
complete responses. Do not use TURN 360 as the independent reference to
calibrate the same gyro that stops that command. No guessed scale correction
is applied: Z_SCALE_PPM remains 1000000. The six-axis sensor has no absolute
yaw reference; improvements do not guarantee zero long-term drift.
