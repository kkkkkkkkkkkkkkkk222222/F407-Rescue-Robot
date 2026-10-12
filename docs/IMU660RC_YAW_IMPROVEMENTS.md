# IMU660RC Yaw: First Precision Trial

## Sources And Hardware

- User hardware: Seekfree IMU660RC, not RA/RB. The vendor repository identifies
  RC as ST LSM6DSV16XTR. Reference repository commit:
  d73c66c9748db4daea36c2234f9b8b792088c58c.
- [Seekfree product/driver references](https://gitee.com/seekfree/IMU660RX_Product)
- [ST LSM6DSV16X driver](https://github.com/STMicroelectronics/lsm6dsv16x-pid)
  reference revision: 2808e5cd6b85f91b66758e1dd0faab5f043aba07.
- [ST AN5763, timestamp and FIFO sections](https://www.st.com/resource/en/application_note/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf)
- [x-io Fusion, stationary bias estimation and AHRS tradeoffs](https://github.com/xioTechnologies/Fusion)

The existing WHO_AM_I=0x70, +/-500 dps sensitivity (17.5 mdps/LSB), +/-4 g,
high-accuracy nominal 1 kHz sensor configuration and raw yaw polarity remain.
No T265 extrinsics, location mode, encoder scale or UART pairing is changed.
No external code/library is copied into the firmware in this trial.

## What Was Changed

1. Enable the sensor timestamp without overwriting unrelated FUNCTIONS_ENABLE
   bits. Read the signed factory INTERNAL_FREQ correction and the 32-bit
   timestamp using the ST register mapping.
2. Use sensor-clock intervals rather than millisecond host-loop timestamps for
   yaw integration. Unsigned subtraction handles the normal timestamp wrap.
   Timestamp conversion is bounded before multiplication to avoid overflow on
   a reset or long outage. The existing 100 ms integration-gap ceiling remains.
3. Integrate the average of previous/current corrected gyro rates (trapezoid),
   retaining signed fractional microdegrees. This reduces rectangular-rule
   error at speed changes and avoids a growing nanosecond-scale accumulator
   that would overflow after a small number of full turns.
4. Increase startup settling from 50 to 250 ms and zero-rate calibration from
   512 to 2048 fresh samples. Calibration timeout is 8 seconds. Keep the
   chassis still during startup; variance and a broad absolute mean-rate guard
   reject obvious motion, but cannot prove that arbitrarily slow rotation is
   absent.
5. Adapt gyro offset slowly only after three seconds of eligible rest. All
   motor drive/target commands must be zero, all encoder 10 ms deltas zero,
   each corrected gyro axis within 0.25 dps, and acceleration magnitude within
   0.95..1.05 g. The offset time constant is 30 seconds. Motion resets the dwell.
   The algorithm does not force yaw to zero or snap angles to 360 degrees.
6. Service pending IMU updates before console/vision processing and again
   after LCD work, keeping SPI out of the motor timer ISR.

## Parameters Shared By Modes

- Per-wheel speed PID: 6 / 0.15 / 0, unchanged.
- Distance/vector heading hold: 2 / 0.005 / 0, already shared by MotionDebug
  and NormalRun. Gamepad manual velocity commands do not invoke heading hold.
- Visual chassis steering: Kp 0.72 -> 0.45, Kd 0.02 -> 0.01, minimum active
  correction 40 -> 20 mm/s. The lower minimum matters near the image center,
  where the old floor could hide a reduction in P. Existing pixel deadbands,
  maximum correction, slew limit and sign conversion remain.
- Camera-tilt servo PID, standalone navigation gain, speed feed-forward,
  acceleration ramps, 60% PWM and slow decay are unchanged.

This visual PD is executed on F407 from upper-computer pixel coordinates;
the upper-computer repository/protocol is not rewritten. All three firmware
presets use the same IMU and motor configuration. Only MotionDebug has the
USART1 diagnostic commands.

## Diagnostics And Interpretation

`IMU` reports ALG=TS_TRAPZ, sensor ID, fresh sample/error counts, current and
maximum interval in microseconds, GAPS, signed factory clock correction,
gyro offsets in mdps, stationary flag, bias-update count and Z_SCALE_PPM.
GAPS counts intervals above 5 ms or invalid/skipped intervals. A skipped
invalid interval reports DT_US=0; it does not mean the actual gap was zero.

`GYRO` includes TOTAL_MDEG (unwrapped) in addition to the legacy modulo-one-turn
and last-query delta fields. The displayed total is clamped to signed 32-bit
millidegrees; internal accumulation remains 64-bit. A 360-degree turn is
360000 mdeg in magnitude, not a modulo heading of 360000.

The user reported about 2..3 degrees of error that appears to stop growing.
This alone cannot identify scale error, zero-bias drift, filter/query timing,
manual alignment error or confusion between modulo/delta/total readouts.
Use known physical angles, not a gyro-controlled TURN command, as the reference.

## First Physical Checks

1. Flash MotionDebug and hold the chassis stationary through startup/READY.
   Then leave it still for at least ten seconds before the first comparison.
2. Send STAT, IMU and GYRO separately, waiting for each reply.
3. Rotate manually on a level, marked surface through one known turn at a
   moderate rate, stop for two seconds, then query GYRO and IMU again.
4. Repeat in the opposite direction, and then through three known turns.
   Record total-angle differences, query deltas, elapsed time, GAPS and bias.
5. Also compare a stationary 30-second interval. Do not change gains, speed
   scale or mechanical alignment during that comparison.

The optional APP_IMU_GYRO_Z_SCALE_PPM remains identity (1000000). Only with a
reliable external angle measurement should scale be calibrated as:

new_scale = current_scale * true_angle / measured_angle

Use multiple clockwise/counter-clockwise runs at comparable speeds/durations;
do not infer a gain from an approximate unsigned "two or three degrees".

## Limitations And Next Steps

This remains a chassis-level Z-rate estimator, not a full 3D AHRS. Tilt can
project rotation onto other axes. Six-axis gravity feedback cannot supply an
absolute yaw reference, so a quaternion/Kalman algorithm alone cannot make
unreferenced heading drift disappear. The RC's SFLP and software AHRS are
possible later paths after coordinate/frame validation, not enabled here.

The host still polls latest data, not FIFO. Timestamp/trapezoid processing
cannot reconstruct missed angular-rate samples during long display or other
host delays. If GAPS/MAX_DT_US show substantial delays, the next acquisition
step should be timestamped FIFO batching with overflow/latency handling,
following ST's example, rather than further PID or scale changes.

Three firmware presets were compiled. A short native numerical check covered
timestamp wrap/factory correction/gap bounds, positive and negative 35-turn
integration, ramps, jittered intervals, fractional carry, accumulator bounds
and bias adaptation. That arithmetic check does not emulate SPI timing,
prove stationary detection on hardware, or establish physical angular accuracy.
No automatic flashing or vehicle movement was performed.
