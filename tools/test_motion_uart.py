#!/usr/bin/env python3
"""Send one UART motion-debug command and print F407 telemetry."""

from __future__ import annotations

import argparse
import time

import serial

try:
    from tools import vision_protocol as protocol
except ImportError:  # Allow running the file directly from the tools folder.
    import vision_protocol as protocol


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="F407 gyro/odometry motion test")
    parser.add_argument("command", choices=("turn", "move", "stop"))
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=protocol.UART_BAUD)
    parser.add_argument("--angle", type=float, default=90.0,
                        help="turn angle in degrees; negative is clockwise")
    parser.add_argument("--direction", type=float, default=0.0,
                        help="move direction in degrees; 0 forward, 90 left")
    parser.add_argument("--distance", type=int, default=1000,
                        help="move distance in millimetres")
    parser.add_argument("--speed", type=int, default=300,
                        help="motion speed in mm/s; zero uses F407 default")
    parser.add_argument("--sequence", type=int, default=None,
                        help="command sequence 0..255; default uses a monotonic tick")
    parser.add_argument("--field-frame", action="store_true",
                        help="interpret --direction in the local field frame")
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def build_command(args: argparse.Namespace, sequence: int) -> bytes:
    if args.command == "turn":
        return protocol.motion_turn_frame(sequence, args.angle, args.speed)
    if args.command == "move":
        return protocol.motion_move_frame(
            sequence, args.direction, args.distance, args.speed, args.field_frame
        )
    return protocol.motion_stop_frame(sequence)


def consume(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    while True:
        start = buffer.find(protocol.FRAME_HEAD)
        if start < 0:
            if buffer and buffer[-1] == protocol.FRAME_HEAD[0]:
                del buffer[:-1]
            else:
                buffer.clear()
            return frames
        if start:
            del buffer[:start]
        if len(buffer) < protocol.FRAME_SIZE:
            return frames
        candidate = bytes(buffer[:protocol.FRAME_SIZE])
        try:
            protocol.parse_frame(candidate)
        except ValueError:
            del buffer[:1]
            continue
        frames.append(candidate)
        del buffer[:protocol.FRAME_SIZE]


def main() -> int:
    args = arguments()
    if args.timeout <= 0:
        raise ValueError("--timeout must be positive")
    sequence = ((time.monotonic_ns() // 1_000_000) & 0xFF
                if args.sequence is None else args.sequence)
    packet = build_command(args, sequence)
    print(f"TX {packet.hex(' ').upper()}")

    with serial.Serial(args.uart, args.baud, timeout=0.05) as uart:
        uart.write(packet)
        if args.command == "stop":
            return 0

        try:
            return monitor(uart, args, sequence)
        finally:
            uart.write(protocol.motion_stop_frame((sequence + 1) & 0xFF))
            uart.flush()


def monitor(uart, args, sequence) -> int:
        buffer = bytearray()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            chunk = uart.read(256)
            if chunk:
                buffer.extend(chunk)
            for frame in consume(buffer):
                message_type, _, _ = protocol.parse_frame(frame)
                if message_type == protocol.MSG_MOTION_STATUS:
                    status = protocol.parse_motion_status(frame)
                    print("STATUS", status)
                    if status["command_sequence"] == sequence and status["state"] in (
                        protocol.MOTION_STATE_DONE,
                        protocol.MOTION_STATE_FAULT,
                    ):
                        return 0 if status["state"] == protocol.MOTION_STATE_DONE else 2
                elif message_type == protocol.MSG_ODOM:
                    odom = protocol.parse_odometry(frame)
                    print("ODOM", odom)
        print("TIMEOUT waiting for F407 motion status")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
