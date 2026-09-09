#!/usr/bin/env python3
"""Flydigi Vader 4 teleoperation bridge for the F407 Gamepad firmware.

The 2.4 GHz receiver is connected to the RDK X5. Linux evdev input is packed
into the existing 15-byte USART3 envelope as TYPE=0x19 / TELEOP=0x06.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import shlex
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass

FRAME_HEAD = bytes((0xA3, 0xB3))
FRAME_TAIL = 0xC3
MSG_MOTION_COMMAND = 0x19
CMD_STOP = 0x00
CMD_TELEOP = 0x06
FLAG_VALID = 0x01
FLAG_ACK_REQUIRED = 0x08
FLAG_TELEOP_ENABLE = 0x20

BUTTON_CLAW_CLOSE = 0x01
BUTTON_CLAW_OPEN = 0x02
BUTTON_LIFT_UP = 0x04
BUTTON_LIFT_DOWN = 0x08
BUTTON_A = 0x10
BUTTON_B = 0x20
BUTTON_X = 0x40
BUTTON_Y = 0x80


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_frame(sequence: int, payload: bytes) -> bytes:
    if len(payload) != 8:
        raise ValueError("payload must contain eight bytes")
    body = bytes((MSG_MOTION_COMMAND, sequence & 0xFF)) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little") + bytes((FRAME_TAIL,))


def signed_byte(value: int) -> int:
    if not -100 <= value <= 100:
        raise ValueError("teleop axis must be -100..100")
    return value & 0xFF


def teleop_frame(sequence: int, forward: int, left: int, yaw: int,
                 camera: int, buttons: int, speed_percent: int,
                 enabled: bool) -> bytes:
    if not 0 <= buttons <= 0xFF or not 0 <= speed_percent <= 100:
        raise ValueError("invalid buttons or speed percentage")
    flags = FLAG_VALID | FLAG_ACK_REQUIRED
    if enabled:
        flags |= FLAG_TELEOP_ENABLE
    return build_frame(sequence, bytes((
        CMD_TELEOP, flags, signed_byte(forward), signed_byte(left),
        signed_byte(yaw), signed_byte(camera), buttons, speed_percent,
    )))


def stop_frame(sequence: int) -> bytes:
    return build_frame(sequence, bytes((CMD_STOP, FLAG_VALID | FLAG_ACK_REQUIRED, 0, 0, 0, 0, 0, 0)))


def shaped_axis(value: float, deadzone: float, exponent: float = 1.45) -> float:
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = min(1.0, (magnitude - deadzone) / (1.0 - deadzone))
    result = scaled ** exponent
    return -result if value < 0.0 else result


def shaped_stick(x: float, y: float, deadzone: float) -> tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    bounded = min(1.0, magnitude)
    output = ((bounded - deadzone) / (1.0 - deadzone)) ** 1.35
    return x / magnitude * output, y / magnitude * output


@dataclass
class PadState:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    hat_x: int = 0
    hat_y: int = 0
    rb: bool = False
    lb: bool = False
    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False


class VaderInput:
    def __init__(self, path: str | None) -> None:
        try:
            from evdev import InputDevice, ecodes, list_devices
        except ImportError as error:
            raise RuntimeError("missing python3-evdev: sudo apt install python3-evdev") from error
        self.ecodes = ecodes
        paths = [path] if path else list_devices()
        candidates = []
        required = {ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY}
        for candidate_path in paths:
            if not candidate_path:
                continue
            try:
                device = InputDevice(candidate_path)
                absolute = set(device.capabilities().get(ecodes.EV_ABS, []))
                keys = set(device.capabilities().get(ecodes.EV_KEY, []))
                if required.issubset(absolute) and ecodes.BTN_SOUTH in keys:
                    name = device.name.lower()
                    score = 10 if ("flydigi" in name or "vader" in name) else 0
                    score += 3 if "xbox" in name else 0
                    candidates.append((score, device))
                else:
                    device.close()
            except OSError:
                continue
        if not candidates:
            raise RuntimeError("no gamepad with two sticks was found under /dev/input/event*")
        candidates.sort(key=lambda item: item[0], reverse=True)
        self.device = candidates[0][1]
        for _, extra in candidates[1:]:
            extra.close()
        self.device.grab()
        self.state = PadState()
        self.axis_info = {}
        for code in required | {ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y}:
            try:
                self.axis_info[code] = self.device.absinfo(code)
            except OSError:
                pass
        self.has_hat_x = ecodes.ABS_HAT0X in self.axis_info
        self.has_hat_y = ecodes.ABS_HAT0Y in self.axis_info
        self._initialize_state()

    @property
    def name(self) -> str:
        return self.device.name

    def _normal_axis(self, code: int, value: int) -> float:
        info = self.axis_info[code]
        center = (info.min + info.max) * 0.5
        span = max(center - info.min, info.max - center, 1.0)
        return max(-1.0, min(1.0, (value - center) / span))

    def _initialize_state(self) -> None:
        e = self.ecodes
        for code, attribute in ((e.ABS_X, "left_x"), (e.ABS_Y, "left_y"),
                                (e.ABS_RX, "right_x"), (e.ABS_RY, "right_y")):
            setattr(self.state, attribute,
                    self._normal_axis(code, self.axis_info[code].value))
        if e.ABS_HAT0X in self.axis_info:
            self.state.hat_x = int(self.axis_info[e.ABS_HAT0X].value)
        if e.ABS_HAT0Y in self.axis_info:
            self.state.hat_y = int(self.axis_info[e.ABS_HAT0Y].value)
        active = set(self.device.active_keys())
        self._update_keys(active)

    def _update_keys(self, active: set[int]) -> None:
        e = self.ecodes
        self.state.rb = getattr(e, "BTN_TR", -1) in active
        self.state.lb = getattr(e, "BTN_TL", -1) in active
        self.state.a = e.BTN_SOUTH in active
        self.state.b = e.BTN_EAST in active
        self.state.x = e.BTN_WEST in active
        self.state.y = e.BTN_NORTH in active
        if not self.has_hat_x:
            if getattr(e, "BTN_DPAD_LEFT", -1) in active:
                self.state.hat_x = -1
            elif getattr(e, "BTN_DPAD_RIGHT", -1) in active:
                self.state.hat_x = 1
            else:
                self.state.hat_x = 0
        if not self.has_hat_y:
            if getattr(e, "BTN_DPAD_UP", -1) in active:
                self.state.hat_y = -1
            elif getattr(e, "BTN_DPAD_DOWN", -1) in active:
                self.state.hat_y = 1
            else:
                self.state.hat_y = 0

    def poll(self) -> None:
        e = self.ecodes
        try:
            events = self.device.read()
        except BlockingIOError:
            return
        for event in events:
            if event.type == e.EV_ABS:
                mapping = {
                    e.ABS_X: "left_x", e.ABS_Y: "left_y",
                    e.ABS_RX: "right_x", e.ABS_RY: "right_y",
                }
                if event.code in mapping:
                    setattr(self.state, mapping[event.code],
                            self._normal_axis(event.code, event.value))
                elif event.code == e.ABS_HAT0X:
                    self.state.hat_x = int(event.value)
                elif event.code == e.ABS_HAT0Y:
                    self.state.hat_y = int(event.value)
            elif event.type == e.EV_KEY:
                self._update_keys(set(self.device.active_keys()))

    def close(self) -> None:
        try:
            self.device.ungrab()
        finally:
            self.device.close()


def command_from_pad(state: PadState, deadzone: float,
                     precision_percent: int) -> tuple[int, int, int, int, int, int, bool]:
    right_x, right_y = shaped_stick(state.right_x, state.right_y, deadzone)
    # Linux uses negative Y for up. F407 payload uses positive forward/left.
    forward = round(-right_y * 100.0)
    left = round(-right_x * 100.0)
    # Left stick left is positive counter-clockwise yaw.
    yaw = round(-shaped_axis(state.left_x, deadzone) * 100.0)
    # Left stick up decreases servo-3 angle (camera up); down looks lower.
    camera = round(shaped_axis(state.left_y, deadzone) * 100.0)
    buttons = 0
    if state.hat_x < 0:
        buttons |= BUTTON_CLAW_CLOSE
    elif state.hat_x > 0:
        buttons |= BUTTON_CLAW_OPEN
    if state.hat_y < 0:
        buttons |= BUTTON_LIFT_UP
    elif state.hat_y > 0:
        buttons |= BUTTON_LIFT_DOWN
    buttons |= BUTTON_A if state.a else 0
    buttons |= BUTTON_B if state.b else 0
    buttons |= BUTTON_X if state.x else 0
    buttons |= BUTTON_Y if state.y else 0
    speed_percent = precision_percent if state.lb else 100
    return forward, left, yaw, camera, buttons, speed_percent, state.rb


def self_test() -> None:
    frame = teleop_frame(7, 100, -100, 25, -25, 0xA5, 35, True)
    assert len(frame) == 15 and frame[:2] == FRAME_HEAD and frame[-1] == FRAME_TAIL
    assert frame[2] == MSG_MOTION_COMMAND and frame[3] == 7
    assert crc16_modbus(frame[2:12]) == int.from_bytes(frame[12:14], "little")
    assert frame[4:12] == bytes((CMD_TELEOP, 0x29, 100, 156, 25, 231, 0xA5, 35))
    assert command_from_pad(PadState(right_y=-1.0, rb=True), 0.12, 35)[0] == 100
    print("flydigi_vader4_remote self-test passed")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flydigi Vader 4 -> F407 teleoperation bridge")
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--device", help="explicit /dev/input/eventN")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--deadzone", type=float, default=0.12)
    parser.add_argument("--precision-percent", type=int, default=35)
    parser.add_argument("--map-command", help="optional quoted command; {pty} is replaced with the mirrored UART PTY")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def run(options: argparse.Namespace) -> int:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("missing pyserial: sudo apt install python3-serial") from error
    if options.rate < 20.0 or options.rate > 100.0:
        raise ValueError("--rate must be 20..100 Hz")
    if not 0.05 <= options.deadzone <= 0.35:
        raise ValueError("--deadzone must be 0.05..0.35")
    if not 10 <= options.precision_percent <= 100:
        raise ValueError("--precision-percent must be 10..100")

    pad = VaderInput(options.device)
    uart = serial.Serial(options.uart, options.baud, timeout=0, write_timeout=0.05)
    map_master = map_slave = None
    map_process = None
    if options.map_command:
        try:
            import pty
        except ImportError as error:
            raise RuntimeError("--map-command requires a Linux PTY environment") from error
        map_master, map_slave = pty.openpty()
        slave_name = os.ttyname(map_slave)
        command = [part.replace("{pty}", slave_name)
                   for part in shlex.split(options.map_command)]
        map_process = subprocess.Popen(command, start_new_session=True)
        os.set_blocking(map_master, False)

    print(f"gamepad={pad.name!r} uart={options.uart}@{options.baud}")
    print("Hold RB to enable; LB=35% precision; release RB/disconnect=STOP")
    sequence = 0
    period = 1.0 / options.rate
    next_send = time.monotonic()
    last_print = 0.0
    try:
        while True:
            now = time.monotonic()
            readable = [pad.device.fd]
            if map_master is not None:
                readable.append(map_master)
            select.select(readable, [], [], max(0.0, min(0.02, next_send - now)))
            pad.poll()

            waiting = uart.in_waiting
            if waiting:
                received = uart.read(waiting)
                if map_master is not None:
                    try:
                        os.write(map_master, received)
                    except BlockingIOError:
                        pass
            if map_master is not None:
                try:
                    os.read(map_master, 4096)  # discard map-side TX; this process owns F407 TX
                except BlockingIOError:
                    pass

            now = time.monotonic()
            if now < next_send:
                continue
            next_send += period
            if now - next_send > period:
                next_send = now + period
            values = command_from_pad(pad.state, options.deadzone,
                                      options.precision_percent)
            uart.write(teleop_frame(sequence, *values))
            sequence = (sequence + 1) & 0xFF
            if now - last_print >= 1.0:
                forward, left, yaw, camera, buttons, speed, armed = values
                print(f"ARM={armed} F={forward:+d} L={left:+d} Y={yaw:+d} "
                      f"CAM={camera:+d} BTN=0x{buttons:02X} SPD={speed}%")
                last_print = now
            if map_process is not None and map_process.poll() is not None:
                raise RuntimeError("map process exited")
    finally:
        try:
            for _ in range(3):
                uart.write(teleop_frame(sequence, 0, 0, 0, 0, 0, 0, False))
                sequence = (sequence + 1) & 0xFF
                time.sleep(0.02)
            uart.write(stop_frame(sequence))
            uart.flush()
        except (OSError, serial.SerialException):
            pass
        pad.close()
        uart.close()
        if map_process is not None and map_process.poll() is None:
            os.killpg(map_process.pid, signal.SIGINT)
            try:
                map_process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                map_process.terminate()
        for descriptor in (map_master, map_slave):
            if descriptor is not None:
                os.close(descriptor)
    return 0


def main() -> int:
    options = arguments()
    if options.self_test:
        self_test()
        return 0
    try:
        return run(options)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
