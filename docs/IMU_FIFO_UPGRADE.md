# IMU660RC FIFO acquisition (2026-10-15)

Update 2026-10-16: runtime fault/recovery behavior below is historical.
See UPPER_RECOVERY_MODE51.md for the current retry, heading-reference recovery,
mode51 handshake and diagnostics format 5. FIFO parsing/integration remain shared.

## Scope

All modes share Main/Src/imu.c. Preserve gyro polarity, 500 dps full scale,
1 kHz high-accuracy ODR, Z gain 1000000 ppm, timestamp frequency trim, and
unwrapped 64-bit trapezoidal yaw. No motor PID, turn profile, UART pairing,
or localization mounting parameters change. This is not a quaternion fusion
replacement and does not eliminate six-axis yaw drift or mounting tilt error.

## Acquisition

- FIFO_CTRL3=0x90: gyro batching at the configured ODR; accelerometer not batched.
- FIFO_CTRL4=0x46: continuous uncompressed FIFO, timestamp every batching cycle.
- Read 7-byte tagged words; retain incomplete gyro/timestamp pairs across calls.
- Pair consecutive gyro and timestamp words in either order. Unexpected or
  duplicate tags fail closed; no interpolation across lost samples.
- Integrate each hardware sample using FIFO timestamp differences, including
  unsigned 32-bit timestamp wrap. Accept 0.5 to 1.5 ms sample intervals.
- Poll latency does not become the sample integration interval. More than 128
  queued words (about 64 ms at this configuration), overflow, malformed data,
  or SPI failure invalidate the IMU. Missing gyro samples for 250 ms also fault.
- Start FIFO on first foreground update, after boot-time display work. ZeroYaw
  schedules a FIFO flush in the same foreground owner; pre-zero samples cannot
  change the new heading origin. Do not zero while driving.
- Latest accelerometer output is used only for stationary/bias qualification,
  not for attitude fusion. Bias adaptation still requires zero motor commands,
  zero encoder deltas and three seconds of low angular rate. Slow external
  rotation cannot be unambiguously distinguished from bias by this gate.
- Startup calibration additionally rejects samples outside 0.9 to 1.1 g.
  Keep the vehicle stationary during boot; this check cannot prove zero yaw rate.
- Temperature is observed, not compensated using an unmeasured coefficient.

## Diagnostics and field validation

MotionDebug `IMU` reports ALG=FIFO_TS_TRAPZ plus IMU_FIFO with PEAK (words),
OVERRUN, FAULTS, MAX_POLL_MS and TEMP_MC (millidegrees Celsius). Periodic DIAG
includes the FIFO line. DT_US now measures adjacent hardware samples; a large
MAX_POLL_MS with normal DT_US is expected when queued samples are recovered.
Retained diagnostic format advances to 4; older retained records are rejected.

Build MotionDebug, NormalRun and Gamepad. Host test command:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itools/imu_test_support -IMain/Inc tools/test_imu_fifo.c -o tmp/test_imu_fifo.exe
.\tmp\test_imu_fifo.exe
```

The mock tests execute the actual imu.c parser/integrator, not a separate model.
They do not prove sensor register behavior, actual clock calibration or physical
angle accuracy. First flash MotionDebug manually and check stationary IMU output
for 30 seconds: READY=1, increasing SAMPLE, OVERRUN=0, FAULTS=0, DT near 1000 us.
Only then compare physical marked +/-360-degree manual rotations and commanded
turns. Record readings after coming fully to rest. Stop on an IMU fault; do not
disable the guards to force a motion. No firmware was flashed by this change.

## Official references

- ST FIFO example: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_fifo.c
- ST register definitions: https://github.com/STMicroelectronics/lsm6dsv16x-pid
- ST AN5763 (timestamp and FIFO sections): https://www.st.com/resource/en/application_note/DM00851721.pdf
- SEEKFREE IMU660RX product references: https://gitee.com/seekfree/IMU660RX_Product

Future work: FIFO hardware validation first, then compare SFLP attitude with raw
integration; calibrate scale only against an independent physical angle reference.
