"""Reference encoder/decoder for the shijue_fangan F407 UART protocol."""

from __future__ import annotations

import math

FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
PAYLOAD_SIZE = 8
UART_BAUD = 115_200

MSG_CONFIG = 0x11
MSG_REPORT = 0x12
MSG_ODOM = 0x15
MSG_FUSED_POSE = 0x16
MSG_STM_STATUS = 0x17
MSG_MISSION = 0x18
CONFIG_ACK = bytes((0xA3, 0xB3, 0x01, 0xC3))

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 1024

FLAG_FOUND = 0x01
FLAG_NEAR = 0x02
FLAG_CLASS_VALID = 0x08
FLAG_DISTANCE_VALID = 0x40

STM_CLAW_VISIBLE = 0x01
STM_GRIPPER_CLOSED = 0x02
STM_MOTORS_ACTIVE = 0x04
STM_AUTO_APPROACH = 0x08
STM_FAULT = 0x80

CMD_STOP = 0x00
CMD_GRAB_CONFIRMED = 0x02
CMD_NAVIGATE_WAYPOINT = 0x03
CMD_ALIGN_SAFE_ZONE = 0x04
CMD_ENTER_SAFE_ZONE = 0x05
CMD_TASK_COMPLETE = 0x06
CMD_ABORT = 0x07
CMD_RETURN_CENTER = 0x08

CMD_VALID = 0x01
CMD_DRIVE_STRAIGHT = 0x02
CMD_USE_FINAL_HEADING = 0x04
CMD_RED_SIDE = 0x08
CMD_DISTANCE_VALID = 0x10


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def build_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if message_type not in (
        MSG_CONFIG, MSG_REPORT, MSG_ODOM, MSG_FUSED_POSE,
        MSG_STM_STATUS, MSG_MISSION,
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
    if len(frame) != 15:
        raise ValueError("frame must contain exactly 15 bytes")
    if frame[:2] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("invalid frame envelope")
    if crc16_modbus(frame[2:12]) != int.from_bytes(frame[12:14], "little"):
        raise ValueError("invalid frame CRC")
    if frame[2] not in (
        MSG_CONFIG, MSG_REPORT, MSG_ODOM, MSG_FUSED_POSE,
        MSG_STM_STATUS, MSG_MISSION,
    ):
        raise ValueError("unsupported message type")
    return frame[2], frame[3], frame[4:12]


def parse_config_ack(frame: bytes) -> dict[str, bool]:
    if frame != CONFIG_ACK:
        raise ValueError("not a configuration ACK")
    return {"accepted": True}


def config_frame(sequence: int, color: int, zone: int) -> bytes:
    if color not in (0x11, 0x12) or zone not in range(1, 5):
        raise ValueError("invalid color or start zone")
    return build_frame(MSG_CONFIG, sequence, bytes((color, zone, 0, 0, 0, 0, 0, 0)))


def pack_counts(normal: int, core: int, casualty: int, danger: int) -> int:
    counts = (normal, core, casualty, danger)
    if any(value < 0 or value > 3 for value in counts):
        raise ValueError("each object count must be 0..3")
    return normal | (core << 2) | (casualty << 4) | (danger << 6)


def report_frame(
    sequence: int,
    x: int,
    y: int,
    distance_mm: int,
    counts: int,
    flags: int,
) -> bytes:
    if not 0 <= x < IMAGE_WIDTH or not 0 <= y < IMAGE_HEIGHT:
        raise ValueError("target pixel is outside the native 1280x1024 image")
    if not 0 <= distance_mm <= 0xFFFF or not 0 <= counts <= 0xFF:
        raise ValueError("invalid distance or count byte")
    if flags & ~0x4B:
        raise ValueError("unsupported vision-report flag")
    found = bool(flags & FLAG_FOUND)
    distance_valid = bool(flags & FLAG_DISTANCE_VALID)
    if distance_valid and distance_mm == 0:
        raise ValueError("a valid distance must be non-zero")
    if not distance_valid and distance_mm != 0:
        raise ValueError("distance must be zero when invalid")
    if not found and (x or y or distance_mm or counts or flags):
        raise ValueError("a no-target report must have an all-zero payload")
    payload = (
        x.to_bytes(2, "big") + y.to_bytes(2, "big")
        + distance_mm.to_bytes(2, "big") + bytes((counts, flags))
    )
    return build_frame(MSG_REPORT, sequence, payload)


def fused_pose_frame(
    sequence: int,
    x_mm: int,
    y_mm: int,
    heading_cdeg: int,
    status: int,
    confidence_and_sigma: int,
) -> bytes:
    if not -32768 <= x_mm <= 32767 or not -32768 <= y_mm <= 32767:
        raise ValueError("x_mm/y_mm must fit int16")
    if not 0 <= heading_cdeg < 36000 or not 0 <= status <= 0x7F:
        raise ValueError("invalid heading or status")
    payload = (
        x_mm.to_bytes(2, "big", signed=True)
        + y_mm.to_bytes(2, "big", signed=True)
        + heading_cdeg.to_bytes(2, "big")
        + bytes((status, confidence_and_sigma))
    )
    return build_frame(MSG_FUSED_POSE, sequence, payload)


def parse_fused_pose(frame: bytes) -> dict[str, int]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_FUSED_POSE:
        raise ValueError("not a fused-pose frame")
    return {
        "sequence": sequence,
        "x_mm": int.from_bytes(payload[0:2], "big", signed=True),
        "y_mm": int.from_bytes(payload[2:4], "big", signed=True),
        "heading_cdeg": int.from_bytes(payload[4:6], "big"),
        "status": payload[6],
        "tracker_confidence": payload[7] & 0x03,
        "mapper_confidence": (payload[7] >> 2) & 0x03,
        "position_sigma_cm": (payload[7] >> 4) & 0x0F,
    }


def stm_status_frame(
    sequence: int,
    flags: int,
    mode: int,
    camera_pitch_cdeg: int,
    acknowledged_sequence: int,
    fault_code: int,
) -> bytes:
    if not 0 <= camera_pitch_cdeg <= 18000:
        raise ValueError("camera pitch must be in 0..18000 cdeg")
    payload = bytes((flags, mode)) + camera_pitch_cdeg.to_bytes(2, "big") + bytes(
        (acknowledged_sequence, fault_code, 0, 0)
    )
    return build_frame(MSG_STM_STATUS, sequence, payload)


def parse_stm_status(frame: bytes) -> dict[str, int | bool]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_STM_STATUS or payload[6:8] != bytes(2):
        raise ValueError("not a valid STM32 status frame")
    flags = payload[0]
    return {
        "sequence": sequence,
        "flags": flags,
        "mode": payload[1],
        "camera_pitch_cdeg": int.from_bytes(payload[2:4], "big"),
        "acknowledged_sequence": payload[4],
        "fault_code": payload[5],
        "claw_visible": bool(flags & STM_CLAW_VISIBLE),
        "gripper_closed": bool(flags & STM_GRIPPER_CLOSED),
        "motors_active": bool(flags & STM_MOTORS_ACTIVE),
        "auto_approach": bool(flags & STM_AUTO_APPROACH),
        "fault": bool(flags & STM_FAULT),
    }


def mission_frame(
    sequence: int,
    command: int,
    flags: int = CMD_VALID,
    target_x_mm: int = 0,
    target_y_mm: int = 0,
    heading_cdeg: int = 0,
) -> bytes:
    if command not in (CMD_STOP, CMD_GRAB_CONFIRMED, CMD_NAVIGATE_WAYPOINT,
                       CMD_ALIGN_SAFE_ZONE, CMD_ENTER_SAFE_ZONE,
                       CMD_TASK_COMPLETE, CMD_ABORT, CMD_RETURN_CENTER):
        raise ValueError("invalid mission command")
    if not flags & CMD_VALID or flags & 0xE0:
        raise ValueError("invalid mission flags")
    if not -32768 <= target_x_mm <= 32767 or not -32768 <= target_y_mm <= 32767:
        raise ValueError("mission target must fit int16")
    if not 0 <= heading_cdeg < 36000:
        raise ValueError("heading must be in 0..35999 cdeg")
    payload = (
        bytes((command, flags))
        + target_x_mm.to_bytes(2, "big", signed=True)
        + target_y_mm.to_bytes(2, "big", signed=True)
        + heading_cdeg.to_bytes(2, "big")
    )
    return build_frame(MSG_MISSION, sequence, payload)


def parse_odometry(frame: bytes) -> dict[str, int]:
    message_type, sequence, payload = parse_frame(frame)
    if message_type != MSG_ODOM or payload[6] == 0:
        raise ValueError("not a valid odometry frame")
    return {
        "sequence": sequence,
        "m1_count": int.from_bytes(payload[0:2], "big"),
        "m2_count": int.from_bytes(payload[2:4], "big"),
        "m3_count": int.from_bytes(payload[4:6], "big"),
        "dt_ms": payload[6],
        "status": payload[7],
    }


def odometry_body_velocity(
    odometry: dict[str, int], wheel_diameter_mm: float = 70.0,
    counts_per_wheel_rev: int = 1768,
) -> tuple[float, float]:
    dt_s = odometry["dt_ms"] * 0.001
    metres_per_count = math.pi * wheel_diameter_mm * 0.001 / counts_per_wheel_rev
    m1 = odometry["m1_delta"] * metres_per_count
    m2 = odometry["m2_delta"] * metres_per_count
    m3 = odometry["m3_delta"] * metres_per_count
    return ((m3 - m1) / math.sqrt(3.0) / dt_s,
            (m1 + m3 - 2.0 * m2) / 3.0 / dt_s)


if __name__ == "__main__":
    examples = (
        config_frame(0, 0x11, 1),
        report_frame(0x10, 640, 512, 0, 1, FLAG_FOUND | FLAG_CLASS_VALID),
        mission_frame(0x20, CMD_NAVIGATE_WAYPOINT,
                      CMD_VALID | CMD_DRIVE_STRAIGHT | CMD_RED_SIDE,
                      0, 950, 9000),
        stm_status_frame(9, 0x09, 2, 7350, 8, 0),
    )
    for example in examples:
        print(example.hex(" ").upper())
