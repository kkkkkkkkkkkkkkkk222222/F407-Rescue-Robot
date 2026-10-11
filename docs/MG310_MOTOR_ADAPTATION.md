# MG310 Hall motor adaptation (2026-10-09)

## Confirmed hardware and sources

- User confirmed: MG310 Hall encoder, unchanged 70 mm omni wheels, regulated 12 V supply.
- User image: actual gear ratio 20.409, rated voltage 7.4 V, recommended 7-13 V,
  no-load speed 500 +/-13% RPM, rated speed 400 +/-13% RPM; continuous stall is not allowed.
- [SiFli SDK MG310 encoder example](https://docs.sifli.com/projects/sdk/latest/en/sf32lb52x/example/rt_device/motor/README.html)
  confirms 13 Hall lines, quadrature x4 and ratio 20.409, giving 1061.268 counts/rev.
  Its different quoted no-load speed is not used: this build follows the user's motor image.
- [MG310 product specification](https://www.yunhanzhilan.com/zw/product.html?id=mg310_motor)
  also lists 7.4 V, 7-13 V, 500/400 RPM and no continuous stall.
- [TI brushed DC PWM interface explanation](https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/963987/faq-what-interface-ph-en-or-pwm-to-use-for-controlling-brushed-dc-motors)
  explains duty-based speed control. Duty is not an actual current limit.
- [AT8236 manufacturer datasheet, page 5](https://datasheet.lcsc.com/datasheet/pdf/d8c76808c9e9a8c1969b3dbfc127e078.pdf?productCode=C2827823)
  defines drive/coast fast decay and drive/brake slow decay.
- [TI current recirculation and decay modes](https://www.ti.com/lit/an/slva321/slva321.pdf)
  explains the reverse winding voltage during fast decay.

## Firmware changes

`APP_MOTOR_GEAR_RATIO=20.409f`, Hall lines=13 and quadrature=4 now yield a
fractional average `1061.268 counts/wheel-rev`. No integer truncation to 20:1.
The same constants drive motor speed, encoder distance and Location odometry.
Encoder timer TI12 x4 mode, wheel diameter, channel wiring and sign conventions are unchanged.

The drive command scale remains +/-1000, but the final hardware mapping clamps
positive/negative propulsion to +/-600 (60% actual drive time). This covers
Motor_Move, SetSpeed, MoveAngle, MoveDistance, TurnAngle and MoveSpin.
With the slow-decay mapping below, 12*0.60=7.2 V is only an ideal average-voltage
estimate, not a regulated 7.2 V rail:
the instantaneous supply pulses remain 12 V and current is not measured.
On 2026-10-11 the user requested a modest increase from 55% to 60%.
The configured 7.4/12 voltage-ratio guard is about 61.7%; this is a software
policy, not a manufacturer duty limit or a substitute for current monitoring.
The new cap still requires hardware validation; startup compensation, PID,
encoder scale and disabled local fault stops are unchanged.

On 2026-10-11 propulsion changed from AT8236 fast decay to slow decay. For a
positive drive fraction D, IN1 is continuously high and IN2 is high for 1-D
of each period. Negative drive swaps these. Thus the bridge alternates
between drive (10/01) and low-side recirculation (11), instead of drive/coast.
At a 4200-count PWM period and command +600, IN1/IN2 compare values are 4200/1680;
command -600 swaps them. A raw input pin can be high 100% while actual
propulsion remains capped at 60%. Command zero still writes both inputs low.
`MotorStatus.command` remains the signed drive fraction, not an input-pin duty.

Both CCR values use the HAL-configured preload and are written with update
events temporarily disabled. This makes changes take effect together on a
timer boundary, including reversals, braking and return to coast. The existing
update-disable state is preserved; PWM frequency and GPIO allocation are unchanged.
The previous VM*duty estimate did not account for fast-decay reverse voltage;
it must not be used to infer actual winding voltage for the old firmware.
This change can materially increase winding current at the same command:
validate briefly with the wheels suspended and stop promptly if they do not turn.

Both speed-control paths use voltage-scaled, RPM-dependent feed-forward.
Ordinary static compensation is 100/1000 and MoveSpin is 80/1000; the old
fixed 45% floor and 90-100% output paths are removed. Per the user's correction,
the original PID values are restored: Kp=6, Ki=0.15 per 10 ms iteration,
Kd=0, correction limit=450, integral limit=2500 and MoveSpin boost=1.8.
The existing PID implementation is unchanged; newly added dynamic PID bounds
are removed. Final PWM limiting is separate from the original PID correction.

The target limit is 300 wheel RPM: 53.0634 counts/10 ms or about 1099.56 mm/s
wheel circumference speed. The omni mixer retains proportional scaling of
all three wheel targets, so this does not define one universal chassis speed.
Existing task speeds and rescue sequencing are not changed.

## Near-zero wheel stability (2026-10-11)

The user confirmed MotionDebug straight-line motion with the chassis suspended:
the nominally idle third wheel repeatedly changed direction. A pure straight
`Motor_Move(v, 0, 0)` already produces exactly zero for that wheel. However,
Debug's heading/cross-track feedback and floating-point frame transforms can
produce tiny nonzero wheel targets; applying the full static PWM offset to each
sign change makes those corrections mechanically visible in slow decay.

All three wheel targets now use a small zero-speed hysteresis in physical units:
at or below 3 mm/s an active target becomes zero; starting or reversing requires
more than 6 mm/s. Zero targets clear speed-PID history and coast, so encoder noise
cannot cause the idle wheel to counter-drive. This also means an intentional
standalone wheel command of 6 mm/s or less will not start the wheel.

MotionDebug MOVE_BODY/MOVE_FIELD additionally remove a 0.5-degree heading band
and a 2 mm cross-track band before calculating feedback. Errors outside each
band use only the excess, avoiding an output jump at the edge. PID gains,
the 60% cap, slow decay and the existing action timeouts remain unchanged.
The wheel is not forcibly disabled: real corrections and commanded turns still
drive it. Suspended wheels do not represent ground odometry, so a sufficiently
large accumulated apparent path error can still legitimately request correction.

The existing AT8236 60 ms stop-brake sequence remains. Both inputs high is
the brake command, not a 100% forward-drive exception. Normal PWM writes
are limited to 60%; brake handling is deliberately separate.

## Stall handling

Per explicit user request, APP_ENABLE_LOCAL_MOTOR_FAULT_STOP=0 in every run
mode. No local stall, reverse-count or distance-no-progress detector stops
and latches the three motors. The new Task motor-fault interlock and injury
backoff's direction/stall check are removed. Keep wire fields for compatibility.
Existing upper yield/escape logic, IMU checks, communication handling,
explicit STOP/ABORT, finite-action timeouts and audit recovery are unchanged.

## Upper-computer update

Apply `rdk_patches/0006-mg310-hall-odometry.patch` to the upper repository.
This is separate from the cumulative rescue-flow patch 0005 and can be
applied alongside it. It updates localization defaults, the example config
and the T265 map builder's wheel conversion to 1061.268.

Verify the ACTUALLY LOADED configuration, including an existing custom
`--config` file or `rescue_map/runtime/localization.conf`:

```ini
wheel_diameter_m = 0.070
counts_per_wheel_revolution = 1061.268
```

Rebuild the localization/map binaries after C++ changes and restart them.
Check the localization startup log for `wheel=70 mm counts/rev=1061.268`.
Do not change UART raw encoder frames; TYPE 0x15 remains unchanged.
Using the old 1768 scale would report only about 60.0% of the actual wheel
distance. External wheel-fusion/navigation therefore needs the update too.

## Verification and first run

The slow-decay firmware builds in `build/MotionDebug` with APP_AUTO_BACKUP=OFF;
no commit, push or flash is performed by that verification build.
Twelve production motor/debug-controller scenarios pass in `tools/test_mg310_motor.c`:
encoder units; the hardware duty cap and brake distinction; physical mixer
speed limit; normal/spin forward/reverse caps; low-speed/overspeed behavior;
preserved original PID parameters; absence of stall/direction interlocks;
slow-decay truth table/paired update guard; brake release and queued restart;
idle-wheel command noise; zero-band hysteresis/reversal; forward/backward
MotionDebug pose noise with real correction retained.
The 22 Python UART tests also pass. The earlier cargo-audit verification covered
ten injury/audit recovery scenarios; those are not rerun for this PWM-only change.
Upper C++ runtime and actual motor performance have not been tested on hardware.

First lift the chassis and check each wheel at 50/100/150 mm/s, especially
encoder direction after replacing motors. Turn each marked wheel ten output
revolutions by hand and expect about 10613 total raw counts. Then check
straight movement over a known distance and loaded speed tracking.
If signs are wrong, correct the corresponding motor/encoder sign; do not
increase duty to compensate for incorrect feedback. If a wheel fails to
start, inspect the load, wiring and compensation against logs before tuning.

Native-host regression command (GCC on PATH):

```powershell
gcc -std=c11 -O1 -ffunction-sections -fdata-sections -Itools/motor_test_support -IMain/Inc tools/test_mg310_motor.c Main/Src/pid.c '-Wl,--gc-sections' -lm -o build/MotionDebug/test_mg310_motor.exe
./build/MotionDebug/test_mg310_motor.exe
```
