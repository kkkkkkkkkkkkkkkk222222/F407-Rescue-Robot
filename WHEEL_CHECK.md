# MotionDebug Three-Wheel Bench Check

This is a USART1 command inside the existing MotionDebug firmware.
There is no separate application mode or build preset. NormalRun, Gamepad,
PID gains and motor calibration are unchanged.

Build the existing MotionDebug preset and flash `build/MotionDebug/WWW.elf`.
The check never starts automatically on boot.

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
