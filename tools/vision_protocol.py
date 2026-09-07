"""Reference encoder for the STM32F407 rescue UART protocol."""

from __future__ import annotations

import math

FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
PAYLOAD_SIZE = 8
UART_BAUD = 115_200

MSG_CONFIG = 0x11
MSG_REPORT = 0x12
MSG_EVENT = 0x13
MSG_NAV = 0x14
MSG_ODOM = 0x15
MSG_STATUS = 0x16
MSG_MOTION_COMMAND = 0x17
MSG_MOTION_STATUS = 0x18
CONFIG_ACK = bytes((0xA3, 0xB3, 0x01, 0xC3))

EVENT_STOP = 0x01
EVENT_RESCUE = 0x02

DEST_MATERIAL = 0x01
DEST_CASUALTY = 0x02

NAV_HOLD = 0x00
NAV_FORWARD = 0x01
NAV_TURN_LEFT = 0x02
NAV_TURN_RIGHT = 0x03
NAV_BACKWARD = 0x04

NAV_EN_ROUTE = 0x01
NAV_NEAR_SAFE = 0x02

FLAG_FOUND = 0x01
FLAG_NEAR = 0x02
FLAG_GRABBED = 0x04
FLAG_CLASS_VALID = 0x08
FLAG_UNKNOWN = 0x10
FLAG_CLAW_VIEW = 0x20
FLAG_DISTANCE_VALID = 0x40

STATUS_MATCH_STARTED = 0x01
STATUS_FOUND = 0x02
STATUS_GRABBED = 0x04
STATUS_CARGO_VALID = 0x08
STATUS_NORMAL_DELIVERED = 0x10
STATUS_NAV_FRESH = 0x20
STATUS_NEAR_SAFE = 0x40
STATUS_CLAW_EMPTY = 0x80

MOTION_CMD_STOP = 0x00
MOTION_CMD_TURN_REL = 0x01
MOTION_CMD_MOVE_DISTANCE = 0x02
MOTION_TURN_NEGATIVE = 0x01
MOTION_MOVE_FIELD_FRAME = 0x01
MOTION_STATE_IDLE = 0x00
MOTION_STATE_RUNNING = 0x01
MOTION_STATE_DONE = 0x02
MOTION_STATE_FAULT = 0x03
MOTION_STATE_STOPPED = 0x04
MOTION_STATUS_IMU_READY = 0x01
MOTION_STATUS_ODOM_VALID = 0x02
MOTION_STATUS_MOTOR_FAULT = 0x04
MOTION_MAX_DISTANCE_MM = 10_000
MOTION_MIN_SPEED_MM_S = 50
MOTION_MAX_SPEED_MM_S = 700
MOTION_MAX_TURN_CDEG = 36_000


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def build_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if message_type not in (
        MSG_CONFIG, MSG_REPORT, MSG_EVENT, MSG_NAV, MSG_ODOM, MSG_STATUS,
        MSG_MOTION_COMMAND, MSG_MOTION_STATUS,
    ):
        raise ValueError("unsupported message type")
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be 0..255")
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError("payload must contain exactly 8 bytes")

    protected = bytes((message_type, sequence)) + payload
    crc = crc16_modbus(protected)
    return FRAME_HEAD + protected + crc.to_bytes(2, "little") + bytes((FRAME_TAIL,))


def parse_frame(frame: bytes) -> tuple[int, int, bytes]:
    """Validate one complete frame and return (type, sequence, payload)."""
    if len(frame) != 15:
        raise ValueError("frame must contain exactly 15 bytes")
    if frame[:2] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("invalid frame envelope")
    expected = int.from_bytes(frame[12:14], "little")
    if crc16_modbus(frame[2:12]) != expected:
        raise ValueError("invalid frame CRC")
    message_type = frame[2]
    if message_type not in (
        MSG_CONFIG, MSG_REPORT, MSG_EVENT, MSG_NAV, MSG_ODOM, MSG_STATUS,
        MSG_MOTION_COMMAND, MSG_MOTION_STATUS,
    ):
        raise ValueError("unsupported message type")
    return message_type, frame[3], frame[4:12]


def parse_config_ack(frame: bytes) -> dict[str, bool]:
    if frame != CONFIG_ACK:
        raise ValueError("not a configuration ACK")
    return {"accepted": True}


def parse_status(frame: bytes) -> dict[str, int | bool]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_STATUS:
        raise ValueError("not a task status frame")
    flags = payload[4]
    return {
        "sequence": sequence,
        "state": payload[0],
        "destination": payload[1],
        "remaining_s": int.from_bytes(payload[2:4], "big"),
        "match_started": bool(flags & STATUS_MATCH_STARTED),
        "found": bool(flags & STATUS_FOUND),
        "grabbed": bool(flags & STATUS_GRABBED),
        "cargo_valid": bool(flags & STATUS_CARGO_VALID),
        "normal_delivered": bool(flags & STATUS_NORMAL_DELIVERED),
        "nav_fresh": bool(flags & STATUS_NAV_FRESH),
        "near_safe": bool(flags & STATUS_NEAR_SAFE),
        "claw_empty": bool(flags & STATUS_CLAW_EMPTY),
        "fault": payload[5],
        "recovery_count": payload[6],
        "cargo_counts": payload[7],
    }


def parse_odometry(frame: bytes) -> dict[str, int]:
    """Decode raw cumulative low-16-bit encoder counts."""
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_ODOM:
        raise ValueError("not an odometry frame")
    dt_ms = payload[6]
    if dt_ms == 0:
        raise ValueError("odometry dt_ms must be non-zero")
    return {
        "sequence": sequence,
        "m1_count": int.from_bytes(payload[0:2], "big"),
        "m2_count": int.from_bytes(payload[2:4], "big"),
        "m3_count": int.from_bytes(payload[4:6], "big"),
        "dt_ms": dt_ms,
        "status": payload[7],
    }


def odometry_body_velocity(
    odometry: dict[str, int],
    wheel_diameter_mm: float = 70.0,
    counts_per_wheel_rev: int = 1768,
) -> tuple[float, float]:
    """Return (forward, left) chassis velocity in m/s for T265 mapping."""
    if wheel_diameter_mm <= 0 or counts_per_wheel_rev <= 0:
        raise ValueError("wheel geometry must be positive")
    dt_s = odometry["dt_ms"] * 0.001
    if dt_s <= 0:
        raise ValueError("odometry dt_ms must be non-zero")
    metres_per_count = (
        math.pi * wheel_diameter_mm * 0.001 / counts_per_wheel_rev
    )
    m1 = odometry["m1_delta"] * metres_per_count
    m2 = odometry["m2_delta"] * metres_per_count
    m3 = odometry["m3_delta"] * metres_per_count
    # Physical port order used by the current chassis firmware:
    # M1=right wheel, M2=left wheel, M3=rear wheel.
    forward = (m1 - m2) / math.sqrt(3.0) / dt_s
    left = (m1 + m2 - 2.0 * m3) / 3.0 / dt_s
    return forward, left


def pack_counts(normal: int, core: int, casualty: int, danger: int) -> int:
    counts = (normal, core, casualty, danger)
    if any(value < 0 or value > 3 for value in counts):
        raise ValueError("each object count must be 0..3")
    return normal | (core << 2) | (casualty << 4) | (danger << 6)


def config_frame(sequence: int, color: int, zone: int) -> bytes:
    if color not in (0x11, 0x12):
        raise ValueError("color must be 0x11 (red) or 0x12 (blue)")
    if zone not in range(1, 5):
        raise ValueError("zone must be 1..4")
    return build_frame(MSG_CONFIG, sequence, bytes((color, zone, 0, 0, 0, 0, 0, 0)))


def report_frame(
    sequence: int,
    x: int,
    y: int,
    distance_mm: int,
    counts: int,
    flags: int,
) -> bytes:
    if not 0 <= x <= 1279 or not 0 <= y <= 1023:
        raise ValueError("x/y are outside the 1280x1024 image")
    if not 0 <= distance_mm <= 0xFFFF:
        raise ValueError("distance_mm must be 0..65535")
    if not 0 <= flags <= 0x7F:
        raise ValueError("flags may only use bits 0..6")
    found = bool(flags & FLAG_FOUND)
    distance_valid = bool(flags & FLAG_DISTANCE_VALID)
    if found and distance_valid and distance_mm == 0:
        raise ValueError("a valid distance must be in 1..65535 mm")
    if found and not distance_valid and distance_mm != 0:
        raise ValueError("distance_mm must be zero when distance is invalid")
    if not found and (counts or flags & (FLAG_NEAR | FLAG_GRABBED | FLAG_UNKNOWN)):
        raise ValueError("a no-target report cannot contain counts/near/grabbed/unknown")
    if not found and (distance_mm or distance_valid):
        raise ValueError("a no-target report cannot contain distance data")
    payload = (
        x.to_bytes(2, "big")
        + y.to_bytes(2, "big")
        + distance_mm.to_bytes(2, "big")
        + bytes((counts, flags))
    )
    return build_frame(MSG_REPORT, sequence, payload)


def stop_frame(sequence: int) -> bytes:
    return build_frame(MSG_EVENT, sequence, bytes((EVENT_STOP, 0, 0, 0, 0, 0, 0, 0)))


def rescue_frame(sequence: int) -> bytes:
    """Request one upper-computer-authorized 1 s fast reverse recovery."""
    payload = bytes((EVENT_RESCUE, 0, 0, 0, 0, 0, 0, 0))
    return build_frame(MSG_EVENT, sequence, payload)


def nav_frame(
    sequence: int,
    direction: int,
    zone_state: int,
    destination: int,
) -> bytes:
    if direction not in (NAV_HOLD, NAV_FORWARD, NAV_TURN_LEFT,
                          NAV_TURN_RIGHT, NAV_BACKWARD):
        raise ValueError("invalid navigation direction")
    if zone_state not in (NAV_EN_ROUTE, NAV_NEAR_SAFE):
        raise ValueError("invalid safe-zone state")
    if destination not in (DEST_MATERIAL, DEST_CASUALTY):
        raise ValueError("invalid destination")
    if zone_state == NAV_NEAR_SAFE and direction != NAV_HOLD:
        raise ValueError("near-safe frames must command HOLD")
    payload = bytes((direction, zone_state, destination, 0, 0, 0, 0, 0))
    return build_frame(MSG_NAV, sequence, payload)


def _motion_speed(speed_mm_s: int, name: str) -> int:
    if speed_mm_s == 0:
        return 0
    if not MOTION_MIN_SPEED_MM_S <= speed_mm_s <= MOTION_MAX_SPEED_MM_S:
        raise ValueError(
            f"{name} must be 0 or {MOTION_MIN_SPEED_MM_S}.."
            f"{MOTION_MAX_SPEED_MM_S} mm/s"
        )
    return speed_mm_s


def motion_stop_frame(sequence: int) -> bytes:
    return build_frame(MSG_MOTION_COMMAND, sequence, bytes(8))


def motion_turn_frame(sequence: int, angle_deg: float,
                      speed_mm_s: int = 0) -> bytes:
    if not math.isfinite(angle_deg) or not -360.0 <= angle_deg <= 360.0:
        raise ValueError("angle_deg must be in -360..360")
    magnitude_cdeg = int(round(abs(angle_deg) * 100.0))
    if not 0 < magnitude_cdeg <= MOTION_MAX_TURN_CDEG:
        raise ValueError("angle_deg must not be zero")
    speed_mm_s = _motion_speed(speed_mm_s, "speed_mm_s")
    flags = MOTION_TURN_NEGATIVE if angle_deg < 0.0 else 0
    payload = (
        bytes((MOTION_CMD_TURN_REL, flags))
        + magnitude_cdeg.to_bytes(2, "big")
        + speed_mm_s.to_bytes(2, "big")
        + bytes(2)
    )
    return build_frame(MSG_MOTION_COMMAND, sequence, payload)


def motion_move_frame(sequence: int, direction_deg: float,
                      distance_mm: int, speed_mm_s: int = 0,
                      field_frame: bool = False) -> bytes:
    if not math.isfinite(direction_deg) or not 0.0 <= direction_deg < 360.0:
        raise ValueError("direction_deg must be in 0..360")
    direction_cdeg = int(round(direction_deg * 100.0))
    if direction_cdeg >= 36000:
        direction_cdeg = 0
    if not 0 < distance_mm <= MOTION_MAX_DISTANCE_MM:
        raise ValueError(
            f"distance_mm must be in 1..{MOTION_MAX_DISTANCE_MM}"
        )
    speed_mm_s = _motion_speed(speed_mm_s, "speed_mm_s")
    flags = MOTION_MOVE_FIELD_FRAME if field_frame else 0
    payload = (
        bytes((MOTION_CMD_MOVE_DISTANCE, flags))
        + direction_cdeg.to_bytes(2, "big")
        + distance_mm.to_bytes(2, "big")
        + speed_mm_s.to_bytes(2, "big")
    )
    return build_frame(MSG_MOTION_COMMAND, sequence, payload)


def parse_motion_status(frame: bytes) -> dict[str, int | bool]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_MOTION_STATUS:
        raise ValueError("not a motion status frame")
    flags = payload[6]
    return {
        "sequence": sequence,
        "state": payload[0],
        "command": payload[1],
        "progress": int.from_bytes(payload[2:4], "big"),
        "remaining": int.from_bytes(payload[4:6], "big"),
        "imu_ready": bool(flags & MOTION_STATUS_IMU_READY),
        "odom_valid": bool(flags & MOTION_STATUS_ODOM_VALID),
        "motor_fault": bool(flags & MOTION_STATUS_MOTOR_FAULT),
        "command_sequence": payload[7],
    }


if __name__ == "__main__":
    examples = (
        config_frame(0, 0x11, 1),
        report_frame(
            0x10,
            640,
            512,
            350,
            pack_counts(1, 0, 0, 0),
            FLAG_FOUND | FLAG_CLASS_VALID | FLAG_DISTANCE_VALID,
        ),
        nav_frame(0x20, NAV_FORWARD, NAV_EN_ROUTE, DEST_MATERIAL),
        nav_frame(0x21, NAV_HOLD, NAV_NEAR_SAFE, DEST_MATERIAL),
        rescue_frame(0x22),
        stop_frame(0x30),
        motion_turn_frame(0x40, 90.0),
        motion_move_frame(0x41, 90.0, 1000, 300),
    )
    for frame in examples:
        print(frame.hex(" ").upper())
