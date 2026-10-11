# Single-Wheel Bench Check

This is a temporary MotionDebug variant, not a new application mode.
NormalRun, MotionDebug, Gamepad, PID gains and motor calibration are unchanged.
USART3 motion commands cannot start motors in this variant.

## Build And Flash

```powershell
cmake --preset WheelCheck -DAPP_AUTO_BACKUP=OFF
cmake --build --preset WheelCheck
```

Flash `build/WheelCheck/WWW.elf`, NOT `build/MotionDebug/WWW.elf`.
The backup option above allows only the intended changes to be committed
manually, without including unrelated dirty files in the automatic backup.

## Bench Procedure

1. Support the chassis securely with all wheels clear of the ground.
2. Keep hands, wires and loose objects away from the wheels. Do not hold a wheel.
3. Open the programmer's USB serial port, connected to PCB serial port 2
   (USART1 PA9 TX / PA10 RX), at 115200, 8N1, no flow control.
4. Send text with CR or LF; turn OFF periodic/automatic sending.
5. Send `STAT`. Verify the `WHEELCHECK ONLY` response before any wheel command.
6. Send exactly one command and wait for its `RESULT` before sending another.

| Command | Action |
| --- | --- |
| `W1+`, `W1-` | Motor port 1, positive/negative logical drive |
| `W2+`, `W2-` | Motor port 2, positive/negative logical drive |
| `W3+`, `W3-` | Motor port 3, positive/negative logical drive |
| `STOP` | Cancel the pulse and stop |
| `STAT` | Confirm bench firmware and limits; no motion |

Every pulse is open-loop 25% for at most 400 ms. No speed PID, heading
correction or wheel mixing is used. TIM6 stops the pulse even if serial or
main-loop processing stalls. Existing braking then coasts the wheel.
Startup is stationary. Other wheels receive no propulsion command.

`RESULT W1+ RAW=a,b,c FB=x,y,z` reports cumulative encoder changes since
the pulse began, in port order 1,2,3, including the brief braking/settling
interval. `RAW` is unmodified; `FB` applies the current encoder polarity.
The selected wheel's `FB` sign should match the command sign. An unexpected
channel changing indicates a possible motor/encoder port mismatch.

For each result, record the actual wheel that moved, the direction of its
top tread relative to the gripper/front, whether other wheels moved, and
whether it stopped automatically. Do not change polarity or PID settings
until the physical motion and feedback have both been checked.

Restore the original MotionDebug or NormalRun firmware after calibration.
