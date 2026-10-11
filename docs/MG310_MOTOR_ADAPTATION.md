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
Encoder timer TI12 x4 mode, wheel diameter, channel wiring and per-wheel drive/
encoder polarity are unchanged. Later measured forward-axis and heading-hold
calibrations are documented in the Scheme A section below.

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

The earlier debug-specific heading and cross-track deadbands have now been
removed with the duplicate debug correction loop. Non-straight translation
uses NormalRun's Motor_MoveAngle controller instead. The wheel speed PID,
60% cap, slow decay and existing action timeout settings are unchanged.

## NormalRun-aligned straight MotionDebug (2026-10-11)

MOVE_BODY with nonzero forward distance and exactly zero left distance now
calls Motor_MoveDistance directly, exactly as NormalRun's straight actions do.
This includes the same stationary-start confirmation, distance feedback,
acceleration, terminal slowdown, raw-IMU heading PID and braking controller.
The user explicitly requested restoring the main program's heading correction.
There is no separate no-heading branch or duplicate Debug heading controller.
KEEP_HEADING retains its heading-hold meaning. The rear wheel can participate
when the shared heading controller requests yaw correction; it is not locked
at zero. Blocking a front wheel does not add cross-track correction from a
virtual lateral displacement, but a real yaw change can still drive the rear wheel.

Progress comes from the shared controller's signed M1/M2 cumulative-count
differences after its stationary-start confirmation:
forward_mm = APP_OMNI_FORWARD_SIGN *
             (delta_M1 * sign_M1 - delta_M2 * sign_M2) * mm_per_count / sqrt(3).
Multiply by commanded forward/backward sign to obtain completion progress.
M3 counts and the estimated world pose cannot determine its distance completion.
Wrong-way encoder travel is not treated as successful progress. Backward DONE
reports negative progress, consistent with the running status.

The speed=0 fallback is 500 mm/s for this straight diagnostic. Explicit wire
speeds remain authoritative (including 300 mm/s). The slowdown profile and
3 mm completion tolerance now match NormalRun's Motor_MoveDistance defaults.
Finite debug action timeout, STOP and health checks are retained. MOVE_FIELD
and MOVE_BODY with a lateral component use Motor_MoveAngle for vector ramps
and the existing raw-IMU heading controller; Debug only handles distance
projection and status, without an extra yaw or cross-track control loop.
NormalRun's public motion APIs are shared, including the later Scheme A forward/
heading calibration. Per-wheel motor/encoder signs, speed PID, 60% cap and slow
decay remain unchanged by the Debug controller alignment.

## Shared NormalRun turn controller (2026-10-11)

TURN_REL now calls Motor_TurnAngleSpeed and reports Motor_GetTurnTravelledDeg,
instead of running an independent signed-field-yaw completion loop. The shared
controller measures abs(current_raw_yaw - start_raw_yaw), so a different IMU
feedback sign does not turn real rotation into zero progress. This matches
NormalRun; it does NOT certify that the physical left/right direction is calibrated.

Motor_Update executes slowdown in the last 30 degrees, completion within
1 degree, and the existing 10 s action timeout. These stop the wheels even
without Debug_Process polling. DONE/FAULT is latched and the same UART sequence
cannot restart the action. A fresh sequence starts from a new yaw reference.
Explicit speed is still converted from 0.1 degree/s to wheel tangent mm/s;
speed=0 selects NormalRun's APP_MOTOR_TURN_FAST_MM_S (500 mm/s). Slowdown never
increases a caller's already-low requested turn speed. Lost IMU readiness,
STOP and terminal results still brake and release outputs through Motor_Stop.

For the upper codex/gamepad-teleop branch based on 0c03b690, the small patch
[upper-motion-debug-speed-500.patch](upper-motion-debug-speed-500.patch) changes
only the map's shared speed-control default from 300 to 500 mm/s. It is not a
replacement protocol patch and does not modify the actual upper checkout here.
Alternatively select 500 mm/s in the existing UI. The same speed control is
also used for turning; lower it before turn tests if appropriate. The upper
process must be restarted after applying the default change. No T265 extrinsics,
localization modes, UART framing or serial pairing are changed.

Removing the extra Debug pose-feedback loop avoids stacking it onto the
NormalRun heading controller; this does not prove physical M1/M2 mapping,
motor polarity or encoder feedback signs.
If a fixed straight command still rotates or travels backward, stop and verify
those hardware mappings before increasing speed or attempting ground navigation.

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

## Scheme A and comparison with the MG370 firmware

Baseline: `d3913f4f1c80f0c06f3672831d4dac722ed62847`, the parent of the
first MG310 adaptation commit `76f69b1`. This is the last pre-MG310 source
snapshot, not a claim that every historical Debug path was physically verified.

The user selected Scheme A. The existing values already implement it:

```c
APP_OMNI_M1_MOTOR_SIGN = APP_OMNI_M2_MOTOR_SIGN = APP_OMNI_M3_MOTOR_SIGN = 1;
APP_OMNI_M1_ENCODER_SIGN = APP_OMNI_M2_ENCODER_SIGN = APP_OMNI_M3_ENCODER_SIGN = -1;
APP_OMNI_FORWARD_SIGN = -1;
APP_MOTOR_HEADING_OUTPUT_SIGN = 1.0f;
```

These are parameter values, not assignment statements to paste over macros.
No Scheme B per-wheel polarity reversal was applied. The current update labels
this selection explicitly; it does not flip the already-corrected values again.

| Item | Pre-MG310 / MG370 baseline | Current MG310, Scheme A |
| --- | --- | --- |
| Wheel diameter | 70 mm | 70 mm, unchanged |
| Configured gear ratio | 34 | 20.409 |
| Hall lines and decoding | 13, x4 | 13, x4, unchanged |
| Counts per output revolution | 1768 | 1061.268 |
| Per-wheel drive signs | +1, +1, +1 | +1, +1, +1, unchanged |
| Per-wheel encoder signs | -1, -1, -1 | -1, -1, -1, unchanged |
| API forward to wheel X axis | Implicit +1 | Explicit -1 |
| Heading-hold output sign | -1 | +1, based on measured yaw response |
| Location IMU yaw sign | -1 | -1, unchanged |
| Ordinary speed PID | 6 / 0.15 / 0, every 10 ms | Unchanged |
| Heading PID | 4 / 0.02 / 0 | Unchanged gains; output polarity corrected |
| Ordinary feed-forward | Fixed signed 450/1000 base term | 100/1000 static term plus voltage-scaled RPM term |
| MoveSpin feed-forward | 220..900/1000, target-proportional | 80/1000 static term plus voltage-scaled RPM term |
| Propulsion limits | Ordinary 900/1000; MoveSpin 1000/1000 | Both 600/1000, including final hardware clamp |
| PWM decay | Drive/coast fast decay | Drive/brake slow decay |
| PWM timers, frequency and pins | Existing 20 kHz configuration | Unchanged |
| Wheel target zero hysteresis | None | Enter zero at <=3 mm/s; start/reverse only above 6 mm/s |
| Wheel target count limit | Fixed 60 counts/10 ms | 53.0634 counts/10 ms, corresponding to 300 wheel RPM |
| Local fault interlocks | Disabled in NormalRun, enabled in Debug/Gamepad | Disabled in all modes, per user request |
| Debug straight control | Separate pose-projected motion and correction | Shared Motor_MoveDistance, encoder progress and raw-IMU heading hold |
| Debug angle control | Separate completion loop using Motor_Move | Shared Motor_TurnAngleSpeed and timer-owned completion |
| MotionDebug USART1 | No motion console | Text motion commands, state/yaw/motor/encoder feedback and WHEELS |

The base PWM entries are feed-forward terms, not guaranteed minimum outputs:
PID can reduce output, and zero targets coast. Duty values across fast/slow
decay are not directly equivalent torque or winding-voltage settings.
The first MG310 commit still used fast decay and a 55% cap; `bd6b451` later
introduced slow decay, the 60% cap and near-zero hysteresis.

`pid.c`, `encoder.c`, `Core/Src/main.c` and the MCU MSP pin configuration have
no differences between the baseline and this comparison. The PWM timer and
encoder decoding changes are therefore not the source of the new direction
definitions. Raw encoder counts retain their meaning; coordinate signs are
applied above the raw encoder layer.

The forward sign is applied in three matched places: wheel mixing, encoder
distance progress and Location forward odometry. It affects Debug, NormalRun
and Gamepad translations. Pure yaw and lateral polarity were not reversed.
The heading sign affects Motor_MoveDistance and Motor_MoveAngle heading hold;
it does not automatically calibrate Task's independent nav_yaw, startup
MoveSpin yaw, visual steering or the upper computer's coordinate convention.
T265 extrinsics, location mode selection and binary UART framing/pairing were
not modified by this Scheme A selection.

Keeping the same speed-PID numbers does not imply an identical physical loop:
the PID error is in encoder counts per 10 ms, and 1061.268/1768 is about 0.6003.
Motor mechanics, feed-forward and PWM decay also changed. No gain rescaling
is applied here. The observed delayed startup at roughly 20% output, followed
by motion around 30% on that run, is a separate compensation/load calibration
issue; do not restore the old 45% term or change PID just from this table.

Current verification is firmware compilation plus the user's incremental
physical measurements. The user observed restored heading correction after
the heading-sign change (200 mm request: peak +8.844 degrees, final -1.027
degrees), but physical backward travel. The forward-axis fix is selected and
compiled; physical gripper-forward direction still requires the next test.
Flash MotionDebug, query STAT for FORWARD_SIGN=-1 and HEADING_SIGN=1, then run
only `MOVE 200 150` on a clear floor and report direction, distance and log.
Do not run the full NormalRun task before basic axis checks are complete.

## Historical verification and first run (before Scheme A)

The automated results and commands below are retained from earlier work.
They were not rerun for Scheme A and are not current physical validation.
No additional self-test programs are introduced for this selection.

The slow-decay firmware builds in `build/MotionDebug` with APP_AUTO_BACKUP=OFF;
no commit, push or flash is performed by that verification build.
Twenty-four production motor/debug-controller scenarios pass in `tools/test_mg310_motor.c`:
encoder units; the hardware duty cap and brake distinction; physical mixer
speed limit; normal/spin forward/reverse caps; low-speed/overspeed behavior;
preserved original PID parameters; absence of stall/direction interlocks;
slow-decay truth table/paired update guard; brake release and queued restart;
idle-wheel command noise; zero-band hysteresis/reversal; forward/backward
field-mode pose noise with shared heading correction retained; straight motion
through pose rotation; signed front-encoder distance/completion; blocked-front
rear-wheel isolation; default/explicit speed; IMU turn completion; debug timeout/STOP;
both turn/feedback signs with no same-sequence restart; timer-owned turn timeout;
IMU failure and STOP cancellation; low-speed turn slowdown; matching NormalRun
and Debug straight heading targets; bounded stationary-start confirmation.
The 22 Python UART tests also pass. The earlier cargo-audit verification covered
ten injury/audit recovery scenarios; those are not rerun for this debug-controller change.
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
