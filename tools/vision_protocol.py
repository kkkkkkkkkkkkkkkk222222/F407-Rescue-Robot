"""Reference encoder for the STM32F407 rescue UART protocol."""

from __future__ import annotations

FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
PAYLOAD_SIZE = 8

MSG_CONFIG = 0x11
MSG_REPORT = 0x12
MSG_EVENT = 0x13
MSG_NAV = 0x14

EVENT_STOP = 0x01

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


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def build_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if message_type not in (MSG_CONFIG, MSG_REPORT, MSG_EVENT, MSG_NAV):
        raise ValueError("unsupported message type")
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be 0..255")
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError("payload must contain exactly 8 bytes")

    protected = bytes((message_type, sequence)) + payload
    crc = crc16_modbus(protected)
    return FRAME_HEAD + protected + crc.to_bytes(2, "little") + bytes((FRAME_TAIL,))


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
    if not 0 <= x <= 639 or not 0 <= y <= 479:
        raise ValueError("x/y are outside the 640x480 image")
    if not 0 <= distance_mm <= 0xFFFF:
        raise ValueError("distance_mm must be 0..65535")
    if not 0 <= flags <= 0x3F:
        raise ValueError("flags may only use bits 0..5")
    found = bool(flags & FLAG_FOUND)
    if found and distance_mm == 0:
        raise ValueError("a found target must have distance_mm in 1..65535")
    if not found and (counts or flags & (FLAG_NEAR | FLAG_GRABBED | FLAG_UNKNOWN)):
        raise ValueError("a no-target report cannot contain counts/near/grabbed/unknown")
    payload = (
        x.to_bytes(2, "big")
        + y.to_bytes(2, "big")
        + distance_mm.to_bytes(2, "big")
        + bytes((counts, flags))
    )
    return build_frame(MSG_REPORT, sequence, payload)


def stop_frame(sequence: int) -> bytes:
    return build_frame(MSG_EVENT, sequence, bytes((EVENT_STOP, 0, 0, 0, 0, 0, 0, 0)))


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


if __name__ == "__main__":
    examples = (
        config_frame(0, 0x11, 1),
        report_frame(
            0x10,
            320,
            240,
            350,
            pack_counts(1, 0, 0, 0),
            FLAG_FOUND | FLAG_CLASS_VALID,
        ),
        nav_frame(0x20, NAV_FORWARD, NAV_EN_ROUTE, DEST_MATERIAL),
        nav_frame(0x21, NAV_HOLD, NAV_NEAR_SAFE, DEST_MATERIAL),
        stop_frame(0x30),
    )
    for frame in examples:
        print(frame.hex(" ").upper())
