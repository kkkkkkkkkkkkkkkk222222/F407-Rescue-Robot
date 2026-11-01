"""Reference codec only: no UART, SDK, or robot actions on import/execution."""
from __future__ import annotations
import struct

MAGIC = b"\xd4\x4d"
VERSION = 1
MAX_PAYLOAD = 160
ID_REQ, ID_REPLY, SYNC_REQ, SYNC_REPLY = 1, 2, 3, 4
CONFIG_REQ, CONFIG_REPLY, WHEEL, GYRO, STATS_REQ, STATS_REPLY = 5, 6, 7, 8, 9, 10
ERROR_REPLY = 127
QUALITY_REQ, QUALITY_REPLY = 11, 12
QUALITY_NAMES = (
    (
        "tx_started", "tx_completed", "tx_hal_busy", "tx_start_errors", "tx_recoveries",
        "tx_abort_errors", "tx_unexpected_tc", "tx_buffer_seal_errors", "tx_precondition_blocks",
        "tx_stuck", "tx_expired", "tx_replaced", "tx_queue_high_water", "tx_bytes_started",
        "tx_bytes_completed", "max_queue_us", "max_transfer_us", "max_wheel_enqueue_age_us",
        "max_gyro_enqueue_age_us", "max_wheel_tc_age_us", "max_gyro_tc_age_us",
        "max_wheel_tc_gap_us", "max_gyro_tc_gap_us", "dma_enabled", "uart_dmat",
        "uart_tx_irq_bits", "dma_remaining", "last_uart_error", "tx_drain_waits",
        "tx_tc_during_recovery",
    ),
    (
        "clock_faults", "clock_regressions", "clock_wrap_irqs", "rcc_cr", "rcc_cfgr",
        "rcc_pllcfgr", "tim5_psc", "tim5_arr", "tim5_cr1", "tim5_dier", "debug_apb1_freeze",
        "drift_bound_ppm", "wheel_time_order_errors", "wheel_raw_delta_mismatches",
        "rear_exti_services", "max_rear_exti_service_us", "gyro_no_new_suppressed",
        "max_gyro_enqueue_gap_us", "max_wheel_capture_us", "max_wheel_interval_us",
        "max_yaw_receive_gap_us", "max_rate_receive_gap_us", "yaw_source_id", "rate_source_id",
        "sensor_uart_baud", "sensor_uart_rx_errors", "sensor_uart_checksum_errors",
        "rear_edge_delay_bound_us", "imu_source_delay_bound_us", "missing_evidence_bits",
    ),
)
STATS_NAMES = (
    "requests", "crc_errors", "bad_headers", "request_queue_drops",
    "partial_timeouts", "wheel_measurements", "wheel_invalid", "wheel_skipped",
    "gyro_enqueued", "gyro_repeated", "gyro_publish_slots_skipped", "enqueue_rejected",
    "extended_started", "extended_completed", "timed_coalesced_or_expired",
    "max_wheel_capture_us", "max_wheel_end_to_dma_return_us", "max_queue_us",
    "max_yaw_receive_gap_us", "max_rate_receive_gap_us", "max_wheel_interval_us",
    "sync_replies", "identity_replies", "max_oldest_gyro_receive_to_dma_return_us",
)


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)
    return value


def packet(kind: int, sequence: int, payload: bytes, version: int = 1) -> bytes:
    if not 0 <= kind <= 255 or not 0 <= sequence <= 65535 or len(payload) > MAX_PAYLOAD:
        raise ValueError("invalid extended packet")
    body = struct.pack(">BBHH", version, kind, len(payload), sequence) + payload
    return MAGIC + body + crc16(body).to_bytes(2, "little") + b"\xe4"


def parse(raw: bytes) -> dict:
    if len(raw) < 11 or raw[:2] != MAGIC or raw[-1] != 0xE4:
        raise ValueError("bad extended envelope")
    version, kind, length, sequence = struct.unpack(">BBHH", raw[2:8])
    if version != VERSION or length > MAX_PAYLOAD or len(raw) != length + 11:
        raise ValueError("bad version or length")
    if crc16(raw[2:-3]) != int.from_bytes(raw[-3:-1], "little"):
        raise ValueError("bad CRC")
    return {"version": version, "type": kind, "sequence": sequence, "payload": raw[8:-3]}


def request(kind: int, nonce: int, sequence: int = 0, *, enable: bool = False,
            quality_page: int = 0) -> bytes:
    if kind not in (ID_REQ, SYNC_REQ, CONFIG_REQ, STATS_REQ, QUALITY_REQ):
        raise ValueError("unsupported request")
    payload = struct.pack(">Q", nonce)
    if kind == CONFIG_REQ:
        payload += bytes((int(enable), 1, 0, 0))
    if kind == QUALITY_REQ:
        if quality_page not in (0, 1):
            raise ValueError("unknown quality page")
        payload += bytes((1, quality_page))
    return packet(kind, sequence, payload)


def decode(raw: bytes) -> dict:
    frame = parse(raw)
    p, kind = frame.pop("payload"), frame["type"]
    if kind == WHEEL:
        values = struct.unpack(">16sIIQQiiIIIiiiIII", p)
        names = ("boot_epoch", "reference_epoch", "measurement_id", "t0_us", "t1_us",
                 "forward_um_s", "left_um_s", "raw1_u32", "raw2_u32", "raw3_u32",
                 "delta1", "delta2", "delta3", "flags", "source_uncertainty_us", "capture_window_us")
    elif kind == GYRO:
        values = struct.unpack(">16sIIqQIIIiQIIIQ", p)
        names = ("boot_epoch", "reference_epoch", "publication_id", "yaw_mdeg",
                 "yaw_receive_us", "yaw_source_id", "yaw_flags", "yaw_uncertainty_us",
                 "rate_mdps", "rate_receive_us", "rate_source_id", "rate_flags",
                 "rate_uncertainty_us", "publish_us")
    elif kind == ID_REPLY:
        values = struct.unpack(">QH16sIQHBBBBIIII12sIIH32s20sBH", p)
        names = ("nonce", "epoch_fault", "boot_epoch", "reference_epoch", "capabilities",
                 "board", "mode", "service", "protocol_major", "protocol_minor",
                 "tick_hz", "uart_baud", "clock_flags", "drift_bound_ppm", "mcu_uid",
                 "device_idcode", "cpu_id", "flash_kib", "firmware_id", "git_commit",
                 "source_dirty", "max_payload")
    elif kind == SYNC_REPLY:
        values = struct.unpack(">Q16sQQIH", p)
        names = ("nonce", "boot_epoch", "receive_event_us", "response_prepare_us",
                 "tick_resolution_us", "epoch_valid")
    elif kind == CONFIG_REPLY:
        values = struct.unpack(">Q16sIHBBIIII", p)
        names = ("nonce", "boot_epoch", "reference_epoch", "status", "profile", "enabled",
                 "uart_baud", "wheel_period_us", "gyro_period_us", "required_baud")
    elif kind == STATS_REPLY:
        values = struct.unpack(">Q16sIQ" + "I" * 24, p)
        names = ("nonce", "boot_epoch", "reference_epoch", "snapshot_us") + STATS_NAMES
    elif kind == QUALITY_REPLY:
        if len(p) < 40:
            raise ValueError("short quality header")
        schema, page, count = struct.unpack(">BBH", p[36:40])
        if schema != 1 or page not in (0, 1) or count != 30 or len(p) != 40 + 4*count:
            raise ValueError("unsupported quality schema/page/length")
        values = struct.unpack(">Q16sIQBBH" + "I"*count, p)
        names = ("nonce", "boot_epoch", "reference_epoch", "snapshot_us", "quality_schema",
                 "page", "field_count") + QUALITY_NAMES[page]
    elif kind == ERROR_REPLY:
        values, names = struct.unpack(">QHBB", p), ("nonce", "status", "request_type", "supported_version")
    else:
        return {**frame, "payload_hex": p.hex()}
    frame.update(zip(names, values))
    for name in ("boot_epoch", "mcu_uid", "firmware_id", "git_commit"):
        if name in frame:
            frame[name] = frame[name].hex()
    return frame


def offset_interval_ns(host_before_ns: int, host_after_ns: int, sync: dict) -> tuple[int, int]:
    """MCU_ns - host_ns bounds from causality; no path symmetry assumption.

    Not a drift/extrapolation guarantee. Unknown oscillator or source timing
    bounds must still fail the upper computer's eligibility gate.
    """
    if host_after_ns < host_before_ns or sync["response_prepare_us"] < sync["receive_event_us"]:
        raise ValueError("non-monotonic synchronization exchange")
    quantum = sync["tick_resolution_us"] * 1000
    lower = sync["response_prepare_us"] * 1000 - host_after_ns - quantum
    upper = sync["receive_event_us"] * 1000 - host_before_ns + quantum
    if lower > upper:
        raise ValueError("inconsistent synchronization exchange")
    return lower, upper


def stream_frames(data: bytes) -> list[bytes]:
    """Decode a complete captured batch, skipping valid legacy 15-byte frames.

    This is an offline helper, not a stateful live UART client.
    """
    result, cursor = [], 0
    while cursor + 2 <= len(data):
        if data[cursor:cursor+2] == b"\xa3\xb3" and cursor+15 <= len(data):
            legacy = data[cursor:cursor+15]
            if legacy[-1] == 0xC3 and crc16(legacy[2:12]) == int.from_bytes(legacy[12:14], "little"):
                cursor += 15
                continue
        if data[cursor:cursor+2] == MAGIC and cursor+8 <= len(data):
            length = int.from_bytes(data[cursor+4:cursor+6], "big") + 11
            raw = data[cursor:cursor+length]
            try:
                parse(raw)
            except ValueError:
                pass
            else:
                result.append(raw)
                cursor += length
                continue
        cursor += 1
    return result


def inspect_capture(data: bytes) -> dict:
    """Offline integrity audit. Never drops evidence of extra bytes or bad CRC.

    A capture boundary can cut a frame; trailing partial bytes are not silently
    counted as a valid frame or joined to a different measurement.
    """
    frames, rejected = [], []
    cursor = 0
    while cursor < len(data):
        rest = data[cursor:]
        if rest == b"\xd4" or rest == b"\xa3":
            break
        if rest[:2] == MAGIC:
            if len(rest) < 8:
                break
            length = int.from_bytes(rest[4:6], "big") + 11
            if length <= MAX_PAYLOAD + 11 and len(rest) < length:
                break
            candidate = rest[:length]
            try:
                parsed = parse(candidate)
            except ValueError:
                pass
            else:
                frames.append(dict(offset=cursor, size=length, family="extended",
                                   type=parsed["type"]))
                cursor += length
                continue
        if rest[:2] == b"\xa3\xb3":
            if len(rest) >= 4 and rest[:4] == b"\xa3\xb3\x01\xc3":
                frames.append(dict(offset=cursor, size=4, family="legacy_ack", type=1))
                cursor += 4
                continue
            if len(rest) < 15:
                break
            candidate = rest[:15]
            if candidate[-1] == 0xc3 and crc16(candidate[2:12]) == int.from_bytes(candidate[12:14], "little"):
                frames.append(dict(offset=cursor, size=15, family="legacy", type=candidate[2]))
                cursor += 15
                continue
        rejected.append(dict(offset=cursor, byte=data[cursor]))
        cursor += 1
    return dict(valid_frames=frames, rejected_bytes=rejected,
                trailing_partial_hex=data[cursor:].hex(),
                stream_integrity_pass=not rejected and cursor == len(data))


def observed_scale_interval(first: dict, second: dict) -> tuple[float, float]:
    """Past-interval average MCU/host rate, NOT a future oscillator guarantee.

    Each MCU response_prepare event occurred between host write-before and
    validated receive. Serial asymmetry/queueing widens, not removes, the range.
    """
    if first["boot_epoch"] != second["boot_epoch"]:
        raise ValueError("cross-boot scale comparison")
    delta = (second["response_prepare_us"]-first["response_prepare_us"])*1000
    lo_dt = second["host_write_before_ns"]-first["receive_monotonic_ns"]
    hi_dt = second["receive_monotonic_ns"]-first["host_write_before_ns"]
    if delta <= 0 or lo_dt <= 0 or hi_dt < lo_dt:
        raise ValueError("non-separated clock observations")
    return max(0, delta-2000)/hi_dt, (delta+2000)/lo_dt
