# F405 NormalDebugTimed Interface Contract 1.1

Date: 2026-11-01. This is an incremental revision of f81e9ef, not a rollback.
Upper baseline: danmo-teng/shijue_fangan, codex/gamepad-teleop,
`c4fdc330f52aab44162f966b15de0313e16afb6d` (remote verified).

## Delivery Status

- The user's RDK queried actual running Git `f81e9ef9a33981d8453eafe9574572e87dea90c6`,
  source_dirty=1. The older expected=43fa6ab is NOT the running identity.
- New implementation/build identities and complete build-input diff are in
  [validation JSON](F405_TIMED_VALIDATION_20261101.json) and
  [build diff](F405_TIMED_BUILD_DIFF_20261101.patch).
- This task did NOT flash, reset the MCU, or send any motion/STOP/reference command.
  One read-only COM18/115200 PING+STAT probe returned no bytes. It does not establish
  a new running identity, boot_epoch, or sensor/telemetry quality.
- The new firmware is NOT source-time-qualified for T265 SDK injection.
  Required drift, rear physical edge latency, and IMU physical source latency
  bounds remain unknown. Nothing is replaced with zero or a convenient estimate.
- Host C/peripheral-model evidence is not board timing, physical UART integrity,
  wheel distance, rotation accuracy, SDK internal fusion, or localization accuracy.

Machine-readable complete contract: [JSON](F405_TIMED_CONTRACT_20261101.json).
Independent encodings and actual host-C packet outputs: [examples](F405_TIMED_EXAMPLES_20261101.json).
Checks: [host log](F405_TIMED_HOST_SIMULATION_20261101.log).
The implementation Git in the validation file is embedded at build time;
later documentation commits are not automatically the running firmware.

## Scope And Compatibility

Preset `NormalDebugTimed`: STM32F405RGT6 ROBO mapping, APP_BOARD_F405_ROBO=1,
APP_ACTIVE_MODE=5, APP_NORMALDEBUG_MOTION_TEST=1, service=1/MOTIONDEBUG,
UART4 PC10/PC11 at 230400 8N1, USART1 PB6/PB7 at 115200, gyro UART5 PC12/PD2 at 230400.
New streaming remains OFF at boot. Ordinary NormalDebug/115200 rejects high-rate
stream enable with status4/BANDWIDTH. NormalDebugTask remains the competition service.

Wire envelope version stays 1. Identity protocol major/minor is explicitly **1.1**.
Capability bit10/0x400 announces read-only quality queries; full capabilities=0x7FF.
No existing payload changes size, byte order, velocity/angle scale, source time
event, sensor sign, or SEQ meaning. Additions use extended types 0B/0C, schema1.
The versioned validity extension is explicit: identity epoch_fault=2 means an
observed clock fault; clock_flags.bit0 now also requires runtime clock checks;
SYNC epoch_valid=0 and existing EPOCH_INVALID bits reject an observed discontinuity.
epoch_fault=1 still denotes unavailable RNG; bit1 still denotes RNG availability.

Legacy 0x15, 0x1F, 0x19, 0x1A and motor control remain unchanged. In 0x1F:
yaw=i32 BE mdeg, rate=i16 BE 0.1deg/s, age/status retain their old definitions.
Yaw/rate are CCW-positive. Old TURN positive is clockwise; only the host action
adapter converts it. 0x15 and 0x1F SEQ remain independent. Do not pair equal SEQs.
Legacy action ACKs are still 8bit and lack a strong cross-reboot action epoch.
No online combined-motion interface is introduced. No ESCAPE, permanent motion
STOP latch, mode51 replacement, competition flow change, wheel-reference segment
clear, camera rotation/lever arm, lidar parser, or hardcoded field135 offset.

## Qualification And Missing Evidence

| Quantity | Current evidence | Published bound | What is still required |
| --- | --- | --- | --- |
| MCU tick resolution | TIM5 nominal 1us counter | resolution only | not an SI or host-relative frequency error guarantee |
| MCU scale/drift | nominal clock tree, checked at runtime | FFFFFFFF ppm | fitted crystal BOM total error over voltage/temp/load/age, or independent clock calibration with validity period |
| wheel snapshot read bracket | actual begin/end plus observed maximum | not total uncertainty | rear edge-to-ISR/update delay, interrupt masking, timer input filter and inter-wheel skew characterization |
| gyro receive event | final UART byte processed by own ISR | event exists | physical sample/filter/queue/link/IRQ total delay and its variation |
| yaw source event | own valid BB message and receive tick | FFFFFFFF us | independent yaw delay evidence; do not borrow rate's time |
| rate source event | own valid AA message and receive tick | FFFFFFFF us | independent rate delay evidence; do not borrow yaw's time |

`TIME_BOUND_KNOWN` stays clear. Wheel source_uncertainty_us and both gyro
uncertainty fields stay FFFFFFFF. `SOURCE_TIME_UNBOUNDED` stays set. Healthy
wheel flags can still be 0x81 and healthy fresh rate flags 0x343. This is honest
valid feedback with **unqualified time**, not a newly qualified SDK input.
There is no calibrated guarantee/expiry to publish; validity deadline is absent,
not unlimited. All observed maxima are history, not worst-case certification.

The unchanged host budgets are: mapping plus collection uncertainty <=5ms;
oldest related physical sample to actual SDK invocation plus uncertainty <=50ms;
gyro coverage of the entire wheel interval with adjacent gaps <=20ms.
The current unknown bounds fail these budgets. A nominal 50/100Hz stream, a
successful configuration load, DONE, or successful SDK call does not certify them.

## Clock, Time Events And Epochs

Nominal source: 8MHz HSE -> PLLM8/PLLN336/PLLP2 -> SYSCLK168MHz,
APB1/4 with timer x2 -> TIM5=84MHz, PSC83 -> 1MHz. PLLQ7 supplies RNG's clock.
An integer divider proves the nominal ratio, not crystal ppm. Neither SysTick,
DWT nor another timer driven by the same HSE independently calibrates HSE.
One SYNC constrains a causal offset candidate, not drift or future validity.

TIM5 low32 wraps every 4294967296 nominal us. Wire ticks remain u64 us with a
software high32 overflow extension. PRIMASK-protected reads compensate pending
UIF; low-word wrap is not a reboot or reference reset. Observed backward time or
timer/clock-tree changes invalidate timing qualification for the boot, without
clamping the clock into false continuity or stopping the robot. Fault bits:
1=counter regressed, 2=timer configuration changed/stopped, 4=clock tree changed.
The raw epoch is not regenerated to disguise a fault. New SYNC returns invalid;
07/08 values carry their existing epoch-invalid flags and invalid velocities.

Limits: firmware cannot detect every possible simultaneous clock/CPU stop,
unobserved multi-wrap IRQ blackout, or unrecorded debugger halt. Such conditions
are outside a valid timing session. After them discard mapping and verify a real
new boot/identity; do not reinterpret a resumed timestamp as proven wall time.
debug_apb1_freeze is exposed for review, not a substitute for halt detection.

boot_epoch is generated by MCU hardware RNG at every MCU initialization and
fixed in that boot. It is probabilistically unique, not a persistent zero-collision
boot counter. RNG failure returns invalid zero epoch, never a fixed UID/RDK UUID.
Physical cross-reboot change of this new image is NOT verified in this task.
reference_epoch increments only for Location_ResetReference, modulo2^32;
raw wheel counts, gyro cumulative yaw, source IDs, and TIM5 are not reset.

SYNC t2 is processing the final request byte parsed from UART4 DMA. t3 is response
preparation before CRC, enqueue and physical transmission. With host monotonic H1
before write and H4 after validated receive, the nominal causal offset candidate is:

```text
lower_ns = t3_us*1000 - H4_ns - quantization_margin_ns
upper_ns = t2_us*1000 - H1_ns + quantization_margin_ns
```

Do not assume symmetric transit, RTT/2 one-way delay, zero queueing, or zero ppm.
`observed_scale_interval` uses two separated exchanges to report a **past interval
average** MCU/host rate interval. It never changes MCU drift_bound_ppm or grants a
future calibration deadline. A host CLOCK_MONOTONIC frequency adjustment also
belongs in a relative-clock qualification model.

## Wheel Feedback

M1 right-front, M2 left-front, M3 rear. Counts/rev=1061.268, wheel diameter=0.070m.
07 raw counters and deltas are unmodified hardware/software counter convention.
Motor-control feedback signs remain [-1,-1,-1], applied only inside that feedback
layer. Do not apply them again to these raw equations:

```text
K = pi * 0.070 / 1061.268                       [m/count]
dt = (t1_us-t0_us)*1e-6                         [nominal s]
forward = K*(delta1-delta2)/(sqrt(3)*dt)          [m/s]
left    = K*(delta1+delta2-2*delta3)/(3*dt)       [m/s]
wire forward_um_s/left_um_s = round(velocity*1e6)
```

Forward raw signs: (+,-,0); left: (+,+,-); pure CCW raw wheel signs: (+,+,+).
Control-layer feedback signs are opposite. Raw u32 cumulative counters wrap
modulo2^32; derive signed increments using modular subtraction and a physically
valid <half-range interval. Two genuine nominal10ms samples form one nominal20ms
interval average. No velocity smoothing window or commanded target is used.

Three counter reads now occur within a short interrupt-protected bracket, keeping
the rear software count stable during the snapshot. Pending EXTI before/after
invalidates that interval. t0/t1 remain **counter-bracket midpoint events**, not
physical simultaneous edge timestamps. The maximum bracket includes the start
boundary bracket. Its 2us example never means total source uncertainty=2us.
Rear IRQ service duration measures work after entry, not unknown pre-entry delay.
Two skipped AB edges can escape detection; no slip/disconnection proof is claimed.

Checks cover pending EXTI, illegal quadrature transitions, >15ms sample gaps,
invalid/nonmonotonic interval, >50ms aggregate interval, raw/delta mismatch,
numerical clipping, clock/epoch error, missing feedback, and reference crossing.
Invalid velocity is INT32_MIN, not zero. A real observed stationary interval may
be valid zero. Lost intervals are not replaced by fabricated stationary data.

## Main Gyro

UART5 receives independent 5A AA and 5A BB five-byte messages. Each source owns
its value, receive event, valid-message counter and flags. Source IDs are MCU
counters, not a vendor sensor sequence. 08 gyro rate uses own decoded AA Z data,
not a derivative of yaw or an action target. Native CCW-positive yaw mdeg/rate mdps
are unchanged, with yaw i64 cumulative and rate i32.

The available vendor manual advertises 1kHz and 230400 but exposes no acquisition
tick, sampling trigger, internal FIFO depth or bounded Kalman group delay.
One five-byte 8N1 packet takes nominal217.014us at230400, two take434.028us;
this excludes sampling, internal processing/queueing and IRQ latency. A 1ms output
period is NOT a proof of <=1ms acquisition-to-receive age. No constant is subtracted
to manufacture a physical sample timestamp.

Telemetry checks its own receive age >20ms as STALE, ready/recovering/yaw uncertain,
rate numeric saturation near the protocol range, and clock/epoch state. These
checks do not prove physical sample freshness. Both unchanged IDs now suppress
an 08 rather than resending at100Hz to inflate apparent source rate. If only one
source changes, the other keeps its source ID/event and explicit REPEAT flag.
The publication scheduler skips backlog; it does not replay historical slots.

## Sending Integrity And Statistics

RDK reported eight extra two-byte prefixes such as D4 4D A3 B3 1F and
A3 B3 D4 4D 01 07. We have the report, not a physical reproduction in this task.
The exact root cause remains unestablished. Published hardening is not a claim
that one particular line was proven to cause all eight observations.

- Only main-loop Uart_Receive(0) starts TX DMA; enqueue/completion IRQ never starts it.
- Reserve queue ownership before HAL can enable DMA; use a private persistent
  DMA buffer, never the caller's stack and never a replaceable queued sample.
- Check UART/DMA states, stream EN, DMAT, TXEIE/TCIE and UART TC before starting.
  The local ST HAL starts DMA before enabling DMAT and does not propagate every
  HAL_DMA_Start_IT failure, so checking UART HAL_OK alone is insufficient.
- FNV1a-32 RAM seals are checked before start and at UART TC. They are low-cost
  corruption diagnostics, not crypto and not a replacement for CRC16/MODBUS.
- DMA completion alone is not UART wire completion. Release a buffer only at
  UART TC with stream disabled and DMAT clear. Unexpected callbacks are counted.
- On error/stall, abort/drop the interrupted frame visibly, wait for TC and stream
  quiescence, then permit reuse. HAL abort alone does not drain DR/shifter bytes.
  A partial aborted frame may still exist on the wire and must fail strict parsing;
  firmware never claims it became a valid complete frame or edits host CRC policy.
- Same-kind unsent samples coalesce, unsent samples older than8ms expire, and
  starts/completions/bytes/overwrites/expiry/errors/recovery/seal events are visible.
  No motion STOP or fault latch is called by this recovery path.

Quality query has **no** stream-enable, motor, servo, STOP, reset, or counter-clear
side effect. Request payload: nonce u64, schema u8=1, page u8=0/1.
Reply payload160: nonce, boot_epoch, reference_epoch, snapshot_us, schema, page,
field_count=30, thirty BE u32 fields. Page0=transport, page1=timing evidence.
These are independent read-only reports, NOT fragments that must be assembled
into one sensor snapshot. Each reply is one CRC-protected packet of171 bytes.
Bad schema/page returns7F status5, bad length status3, bad CRC gets no reply.

All counter maxima are **observed**, start at zero without observations, saturate
at FFFFFFFF, and do not certify an upper latency bound. Event counters wrap u32.
Queue delay is enqueue -> before DMA API. Transfer span is before DMA API -> UART
TC callback. Wheel age uses t1; gyro age uses older own receive event if both exist.
None includes unknown physical IMU acquisition/filter delay or host SDK queueing.
missing_evidence_bits=7: crystal total-error spec1, rear physical-edge latency2,
IMU physical sample/filter/link/IRQ latency4. They do not clear just because
statistics look small. See JSON quality_units and generated field tables.

## Frequency And Bandwidth

Nominal: encoder capture100Hz, 07 output50Hz, gyro module configured1kHz, 08
publication ceiling100Hz when sources update. Actual period/gaps must be measured;
source counters and existing stats provide observations, not guaranteed frequency.

8N1 TX model at230400: old0x15=1500B/s, old0x1F=1500, status=300,
07=4750, 08=9500, SYNC5Hz=285, identity1Hz=148, oldstats1Hz=143:
total18126B/s,78.67% utilization. Two quality pages each1Hz add342B/s:
18468B/s,80.16%, about19.84% nominal headroom. RX requests use the independent
full-duplex RX direction. Reply bursts still consume real TX time.
95-byte telemetry takes nominal4.123ms;171-byte quality reply takes7.422ms.
Stagger pages and queries; do not batch all polls. Queries can legitimately cause
8ms-expiry losses and gyro coverage failure. Watch drop/gap/age counters and raw
bytes; do not raise gate budgets to hide this. Real hardware throughput, interrupt
blackout, acquisition-to-wire latency and maximum backlog remain unverified.

## Build And Deployment

```powershell
cmake --preset NormalDebugTimed
cmake --build --preset NormalDebugTimed --parallel 6
# Artifact: build/NormalDebugTimed/WWW.elf
```

The validation JSON records embedded Git/firmware_id/source_dirty and ELF SHA256.
Existing Task.c whitespace remains dirty=1; supplied patch reproduces that source
difference. IDE mappings and tmp reference repositories are not part of the
firmware identity diff and are not uploaded. Do not force source_dirty=0.
Exact byte-for-byte ELF reproduction is not asserted: existing __DATE__/__TIME__
banners and absolute toolchain paths affect ELF bytes. Pin source, patch, compiler,
flags and compare firmware input identity; distribute the recorded ELF hash.

This task does not perform deployment. After an explicitly authorized flash,
the RDK unique UART owner must query actual identity (mode5/service1/baud230400,
protocol1.1/firmware_id/Git/dirty/boot_epoch), SYNC, then subscribe profile1.
Update the reviewed version range and decode0B/0C using the provided codec.
The old running f81e9ef must not be relabeled as the new build. Retain all new raw
bytes and original damaged prefixes; compare TX counters with strict receive
parsing. SDK input remains refused while any source/clock uncertainty is unknown.
Only after timing review and sustained **actual velocity API acceptance** should
the user manually perform A/B plus independent tape/ground-mark distance/angle
measurements. SDK acceptance is still not a localization accuracy pass.

## Evidence And Next Required Work

1. Host actual Uart.c: 2000 mixed frames byte-exact; caller-buffer overwrite,
   active-slot preservation, coalescing, expiry, HAL_BUSY, stale DMAT, abort drain,
   unexpected TC and intentional RAM mutation tested. One seal error and recovery
   counts in that log are deliberate injected failures, not observations on F405.
2. Host actual DebugClock.c: nominal divider, pending/serviced low32 wrap,
   backward time quarantine and timer/clock-tree change tested. Not ppm calibration.
3. Host actual DebugTelemetry.c: queries, new pages, independent source IDs,
   no-source-repeat suppression, flags, counter/reference/boot fixtures tested.
4. Python codec: wire CRC/length, extra-prefix rejection evidence, valid legacy
   payload magic, partial captures, explicit unknown uncertainty and past-clock
   observation interval checked. No CRC-ignore or byte splicing.
5. Board flash/readback/timing/wire validation and physical motion are NOT performed.

Remaining physical evidence: identify fitted crystal and total error/conditions,
or independently calibrate against a trusted time reference with expiry; bound
rear edge -> software counter and all interrupt blocking using an external edge
reference/logic analyzer (ISR service time alone is insufficient); obtain vendor
physical sampling/filter/FIFO timing or a measurable hardware source event with
mapping. If the current gyro cannot supply a reliable bound, keep its unknown
flag and use a timestamped/synchronized source for the strict SDK channel rather
than relabeling its UART arrival. Record actual wire traces at full real load.

ST references: [official HAL source](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_uart.c),
[AN2867 oscillator design](https://www.st.com/resource/en/application_note/an2867-oscillator-design-guide-for-stm8afals-stm32-mcus-and-mpus-stmicroelectronics.pdf),
[AN4776 timer measurement](https://www.st.com.cn/resource/en/application_note/an4776-how-to-use-generalpurpose-timer-peripheral-on-stm32-mcus-stmicroelectronics.pdf),
[ES0182 device errata](https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405-407xx-and-stm32f415-417xx-device-limitations-stmicroelectronics.pdf).
These explain mechanisms, not this board's unmeasured timing guarantee or a
specific diagnosis of the reported extra prefixes.

<!-- GENERATED_FIELD_TABLES -->

## Complete Wire Field Tables

All offsets below are payload-relative, not absolute frame offsets. All numerical fields are BE except CRC LE.

Envelope: D4 4D | version1:u8 | TYPE:u8 | N:u16 BE | SEQ:u16 BE | payload:N | CRC16 LE | E4.
CRC16/MODBUS initFFFF polyA001 covers frame[2:8+N]. Max TX payload160, RX total64, partial timeout20ms, no fragmentation.

| TYPE | Name | Payload bytes |
| --- | --- | --- |
| 0x01 | identity_request | 8 |
| 0x02 | identity_reply | 137 |
| 0x03 | sync_request | 8 |
| 0x04 | sync_reply | 46 |
| 0x05 | configure_request | 12 |
| 0x06 | configure_reply | 48 |
| 0x07 | wheel | 84 |
| 0x08 | gyro | 84 |
| 0x09 | stats_request | 8 |
| 0x0A | stats_reply | 132 |
| 0x7F | error_reply | 12 |
| 0x0B | quality_request | 10 |
| 0x0C | quality_reply | 160 |

01/03/09 requests each contain only nonce:u64 BE. Replies echo request SEQ and nonce; telemetry SEQ is independent global u16.
Periodic SEQ is not source ID or time. Pair by boot epoch/source IDs/real intervals, never equal SEQ.

### identity_request

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | client correlation nonce, unique for outstanding requests |

### identity_reply

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo |
| 8 | 2 | u16 | epoch_fault | v1.1: 0 RNG available and clock checks healthy, 1 RNG unavailable, 2 observed clock fault |
| 10 | 16 | opaque | boot_epoch | MCU hardware RNG, stable during boot |
| 26 | 4 | u32 | reference_epoch | Location reference generation |
| 30 | 8 | u64 | capabilities | presence bits, not time qualification |
| 38 | 2 | u16 | board | compiled board mapping identifier, 405 |
| 40 | 1 | u8 | mode | APP_ACTIVE_MODE=5 |
| 41 | 1 | u8 | service | 1=MOTIONDEBUG |
| 42 | 1 | u8 | protocol_major | 1 |
| 43 | 1 | u8 | protocol_minor | 1 |
| 44 | 4 | u32 | tick_hz | nominal 1000000 |
| 48 | 4 | u32 | uart_baud | actual running UART4 configuration |
| 52 | 4 | u32 | clock_flags | v1.1 bit0 checked configured TIM5/clock tree with no observed discontinuity; bit1 RNG available; neither proves ppm |
| 56 | 4 | u32 | drift_bound_ppm | 0xffffffff unknown |
| 60 | 12 | opaque | mcu_uid | three hardware UID words, each BE |
| 72 | 4 | u32 | device_idcode | runtime DBGMCU IDCODE; family does not uniquely identify board SKU |
| 76 | 4 | u32 | cpu_id | runtime SCB CPUID |
| 80 | 2 | u16 | flash_kib | runtime flash size register |
| 82 | 32 | opaque | firmware_id | image-embedded SHA256 build-input/config fingerprint, not an ELF self hash |
| 114 | 20 | opaque | git_commit | Git revision embedded when building |
| 134 | 1 | u8 | source_dirty | tracked source diff at build |
| 135 | 2 | u16 | max_payload | 160 MCU TX payload limit; MCU RX total frame limit is 64 |

### sync_reply

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo |
| 8 | 16 | opaque | boot_epoch | boot identity |
| 24 | 8 | u64 | receive_event_us | MCU handles last request byte from DMA parse, before main CRC validation |
| 32 | 8 | u64 | response_prepare_us | main prepares response timestamp; before CRC build/enqueue, not first UART bit |
| 40 | 4 | u32 | tick_resolution_us | 1, resolution only, not source uncertainty |
| 44 | 2 | u16 | epoch_valid | v1.1: 1 RNG available and clock checks healthy; 0 do not map this clock |

### configure_request

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo |
| 8 | 1 | u8 | enabled | 0 stop new stream, 1 enable |
| 9 | 1 | u8 | profile | 1 only |
| 10 | 2 | opaque | reserved | must be zero |

### configure_reply

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo |
| 8 | 16 | opaque | boot_epoch | boot identity |
| 24 | 4 | u32 | reference_epoch | Location reference generation |
| 28 | 2 | u16 | status | 0 OK,4 bandwidth,5 invalid profile |
| 30 | 1 | u8 | profile | 1 |
| 31 | 1 | u8 | enabled | resulting state |
| 32 | 4 | u32 | uart_baud | actual |
| 36 | 4 | u32 | wheel_period_us | 20000 nominal |
| 40 | 4 | u32 | gyro_period_us | 10000 nominal |
| 44 | 4 | u32 | required_baud | 230400 minimum |

### wheel

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 16 | opaque | boot_epoch | boot identity |
| 16 | 4 | u32 | reference_epoch | reference generation at interval end |
| 20 | 4 | u32 | measurement_id | real two-interval encoder feedback record ID; wraps modulo 2^32 |
| 24 | 8 | u64 | t0_us | previous encoder capture-bracket midpoint |
| 32 | 8 | u64 | t1_us | current capture-bracket midpoint |
| 40 | 4 | i32 | forward_um_s | rotation-center body X velocity, 1e-6 m/s; INT32_MIN invalid |
| 44 | 4 | i32 | left_um_s | rotation-center body Y velocity, 1e-6 m/s; INT32_MIN invalid |
| 48 | 4 | u32 | raw1_u32 | right front unmodified cumulative count low32 |
| 52 | 4 | u32 | raw2_u32 | left front low32 |
| 56 | 4 | u32 | raw3_u32 | rear low32 |
| 60 | 4 | i32 | delta1 | raw unmodified count increment for this exact interval |
| 64 | 4 | i32 | delta2 | same |
| 68 | 4 | i32 | delta3 | same |
| 72 | 4 | u32 | flags | feedback health; VALID does not imply source-time qualification or no slip |
| 76 | 4 | u32 | source_uncertainty_us | 0xffffffff unknown, rear software edge latency unbounded |
| 80 | 4 | u32 | capture_window_us | maximum observed CPU read-bracket width in interval, not total sensor timing bound |

### gyro

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 16 | opaque | boot_epoch | boot identity |
| 16 | 4 | u32 | reference_epoch | Location reference generation; raw yaw not reset |
| 20 | 4 | u32 | publication_id | publish attempt ID, not source sample ID |
| 24 | 8 | i64 | yaw_mdeg | native CCW-positive cumulative yaw, 0.001 deg |
| 32 | 8 | u64 | yaw_receive_us | MCU ISR last-byte processing time, NOT acquisition time |
| 40 | 4 | u32 | yaw_source_id | MCU valid angle-message counter, NOT vendor sensor ID |
| 44 | 4 | u32 | yaw_flags | source health/new/repeat and receive-time semantics |
| 48 | 4 | u32 | yaw_uncertainty_us | 0xffffffff unknown source-time bound |
| 52 | 4 | i32 | rate_mdps | native CCW-positive Z angular rate, 0.001 deg/s |
| 56 | 8 | u64 | rate_receive_us | own MCU ISR last-byte processing time |
| 64 | 4 | u32 | rate_source_id | MCU valid rate-message counter |
| 68 | 4 | u32 | rate_flags | independent flags; heading uncertainty applies only to yaw |
| 72 | 4 | u32 | rate_uncertainty_us | 0xffffffff unknown source-time bound |
| 76 | 8 | u64 | publish_us | main snapshot publication event, before CRC/enqueue, not physical TX |

### stats_reply

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo |
| 8 | 16 | opaque | boot_epoch | identity |
| 24 | 4 | u32 | reference_epoch | reference generation |
| 28 | 8 | u64 | snapshot_us | snapshot creation |
| 36 | 96 | u32[24] | counters | ordered counters in validation JSON / Python STATS_NAMES |

### error_reply

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo if request has >=8 payload bytes, otherwise 0 |
| 8 | 2 | u16 | status | 1 bad version,2 unsupported type,3 bad length |
| 10 | 1 | u8 | request_type | request kind |
| 11 | 1 | u8 | supported_version | 1 |

### quality_request

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | unique outstanding transaction nonce |
| 8 | 1 | u8 | quality_schema | 1 |
| 9 | 1 | u8 | page | 0 or 1; invalid schema/page returns error5 |

### quality_reply_page_0

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo request nonce |
| 8 | 16 | opaque | boot_epoch | unchanged MCU boot identity |
| 24 | 4 | u32 | reference_epoch | reference generation |
| 28 | 8 | u64 | snapshot_us | MCU response preparation, not UART first bit |
| 36 | 1 | u8 | quality_schema | 1 |
| 37 | 1 | u8 | page | 0 transport, 1 timing evidence |
| 38 | 2 | u16 | field_count | 30 |
| 40 | 4 | u32 | tx_started | observed value; ordered page index 0 |
| 44 | 4 | u32 | tx_completed | observed value; ordered page index 1 |
| 48 | 4 | u32 | tx_hal_busy | observed value; ordered page index 2 |
| 52 | 4 | u32 | tx_start_errors | observed value; ordered page index 3 |
| 56 | 4 | u32 | tx_recoveries | observed value; ordered page index 4 |
| 60 | 4 | u32 | tx_abort_errors | observed value; ordered page index 5 |
| 64 | 4 | u32 | tx_unexpected_tc | observed value; ordered page index 6 |
| 68 | 4 | u32 | tx_buffer_seal_errors | observed value; ordered page index 7 |
| 72 | 4 | u32 | tx_precondition_blocks | observed value; ordered page index 8 |
| 76 | 4 | u32 | tx_stuck | observed value; ordered page index 9 |
| 80 | 4 | u32 | tx_expired | observed value; ordered page index 10 |
| 84 | 4 | u32 | tx_replaced | observed value; ordered page index 11 |
| 88 | 4 | u32 | tx_queue_high_water | observed value; ordered page index 12 |
| 92 | 4 | u32 | tx_bytes_started | observed value; ordered page index 13 |
| 96 | 4 | u32 | tx_bytes_completed | observed value; ordered page index 14 |
| 100 | 4 | u32 | max_queue_us | observed value; ordered page index 15 |
| 104 | 4 | u32 | max_transfer_us | observed value; ordered page index 16 |
| 108 | 4 | u32 | max_wheel_enqueue_age_us | observed value; ordered page index 17 |
| 112 | 4 | u32 | max_gyro_enqueue_age_us | observed value; ordered page index 18 |
| 116 | 4 | u32 | max_wheel_tc_age_us | observed value; ordered page index 19 |
| 120 | 4 | u32 | max_gyro_tc_age_us | observed value; ordered page index 20 |
| 124 | 4 | u32 | max_wheel_tc_gap_us | observed value; ordered page index 21 |
| 128 | 4 | u32 | max_gyro_tc_gap_us | observed value; ordered page index 22 |
| 132 | 4 | u32 | dma_enabled | observed value; ordered page index 23 |
| 136 | 4 | u32 | uart_dmat | observed value; ordered page index 24 |
| 140 | 4 | u32 | uart_tx_irq_bits | observed value; ordered page index 25 |
| 144 | 4 | u32 | dma_remaining | observed value; ordered page index 26 |
| 148 | 4 | u32 | last_uart_error | observed value; ordered page index 27 |
| 152 | 4 | u32 | tx_drain_waits | observed value; ordered page index 28 |
| 156 | 4 | u32 | tx_tc_during_recovery | observed value; ordered page index 29 |

### quality_reply_page_1

| Offset | Bytes | Type | Field | Meaning |
| --- | --- | --- | --- | --- |
| 0 | 8 | u64 | nonce | echo request nonce |
| 8 | 16 | opaque | boot_epoch | unchanged MCU boot identity |
| 24 | 4 | u32 | reference_epoch | reference generation |
| 28 | 8 | u64 | snapshot_us | MCU response preparation, not UART first bit |
| 36 | 1 | u8 | quality_schema | 1 |
| 37 | 1 | u8 | page | 0 transport, 1 timing evidence |
| 38 | 2 | u16 | field_count | 30 |
| 40 | 4 | u32 | clock_faults | observed value; ordered page index 0 |
| 44 | 4 | u32 | clock_regressions | observed value; ordered page index 1 |
| 48 | 4 | u32 | clock_wrap_irqs | observed value; ordered page index 2 |
| 52 | 4 | u32 | rcc_cr | observed value; ordered page index 3 |
| 56 | 4 | u32 | rcc_cfgr | observed value; ordered page index 4 |
| 60 | 4 | u32 | rcc_pllcfgr | observed value; ordered page index 5 |
| 64 | 4 | u32 | tim5_psc | observed value; ordered page index 6 |
| 68 | 4 | u32 | tim5_arr | observed value; ordered page index 7 |
| 72 | 4 | u32 | tim5_cr1 | observed value; ordered page index 8 |
| 76 | 4 | u32 | tim5_dier | observed value; ordered page index 9 |
| 80 | 4 | u32 | debug_apb1_freeze | observed value; ordered page index 10 |
| 84 | 4 | u32 | drift_bound_ppm | observed value; ordered page index 11 |
| 88 | 4 | u32 | wheel_time_order_errors | observed value; ordered page index 12 |
| 92 | 4 | u32 | wheel_raw_delta_mismatches | observed value; ordered page index 13 |
| 96 | 4 | u32 | rear_exti_services | observed value; ordered page index 14 |
| 100 | 4 | u32 | max_rear_exti_service_us | observed value; ordered page index 15 |
| 104 | 4 | u32 | gyro_no_new_suppressed | observed value; ordered page index 16 |
| 108 | 4 | u32 | max_gyro_enqueue_gap_us | observed value; ordered page index 17 |
| 112 | 4 | u32 | max_wheel_capture_us | observed value; ordered page index 18 |
| 116 | 4 | u32 | max_wheel_interval_us | observed value; ordered page index 19 |
| 120 | 4 | u32 | max_yaw_receive_gap_us | observed value; ordered page index 20 |
| 124 | 4 | u32 | max_rate_receive_gap_us | observed value; ordered page index 21 |
| 128 | 4 | u32 | yaw_source_id | observed value; ordered page index 22 |
| 132 | 4 | u32 | rate_source_id | observed value; ordered page index 23 |
| 136 | 4 | u32 | sensor_uart_baud | observed value; ordered page index 24 |
| 140 | 4 | u32 | sensor_uart_rx_errors | observed value; ordered page index 25 |
| 144 | 4 | u32 | sensor_uart_checksum_errors | observed value; ordered page index 26 |
| 148 | 4 | u32 | rear_edge_delay_bound_us | observed value; ordered page index 27 |
| 152 | 4 | u32 | imu_source_delay_bound_us | observed value; ordered page index 28 |
| 156 | 4 | u32 | missing_evidence_bits | observed value; ordered page index 29 |

### capabilities

| Name | Mask |
| --- | --- |
| identity_query | 0x1 |
| synchronization | 0x2 |
| stream_configure | 0x4 |
| body_center_wheel_feedback | 0x8 |
| gyro_receive_timestamps | 0x10 |
| statistics | 0x20 |
| legacy_compatible | 0x40 |
| mcu_rng_boot_epoch | 0x80 |
| reference_epoch | 0x100 |
| image_build_identity | 0x200 |
| read_only_quality_pages | 0x400 |

### wheel_flags

| Name | Mask |
| --- | --- |
| VALID_FEEDBACK | 0x1 |
| TIME_BOUND_KNOWN | 0x2 |
| EXTI_PENDING | 0x4 |
| INVALID_TRANSITION | 0x8 |
| INVALID_INTERVAL | 0x10 |
| NUMERIC_CLIPPED | 0x20 |
| SAMPLING_GAP | 0x40 |
| SLIP_UNKNOWN | 0x80 |
| EPOCH_INVALID | 0x100 |
| NO_NEW_FEEDBACK | 0x200 |
| REFERENCE_CROSSED | 0x400 |

### gyro_flags

| Name | Mask |
| --- | --- |
| VALID_VALUE | 0x1 |
| READY | 0x2 |
| RECOVERING | 0x4 |
| UNCERTAIN | 0x8 |
| STALE | 0x10 |
| SATURATED | 0x20 |
| NEW | 0x40 |
| REPEAT | 0x80 |
| MCU_RECEIVE_EVENT | 0x100 |
| SOURCE_TIME_UNBOUNDED | 0x200 |
| EPOCH_INVALID | 0x400 |

### clock_fault_bits

| Name | Mask |
| --- | --- |
| counter_regressed | 0x1 |
| timer_configuration_changed | 0x2 |
| clock_tree_changed | 0x4 |

### missing_evidence_bits

| Name | Mask |
| --- | --- |
| crystal_total_error_spec | 0x1 |
| rear_physical_edge_delay | 0x2 |
| imu_acquisition_filter_link_irq_delay | 0x4 |

