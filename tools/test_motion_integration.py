"""Regression checks for mode isolation and updated debug wire format."""
import pathlib
import re
import subprocess
import unittest
from tools import vision_protocol as p

ROOT = pathlib.Path(__file__).resolve().parents[1]

class MotionIntegrationTests(unittest.TestCase):
    def test_normal_core_matches_pinned_upstream(self):
        for name in ("Task.c", "motor.c", "mechanism.c"):
            path = "Main/Src/" + name
            upstream = subprocess.check_output(["git", "show", "68a0802:" + path], cwd=ROOT).decode()
            self.assertEqual(upstream.replace("\r\n", "\n"), (ROOT/path).read_text())

    def test_types_do_not_collide(self):
        self.assertEqual((p.MSG_STM_STATUS, p.MSG_MISSION), (0x17, 0x18))
        self.assertEqual((p.MSG_MOTION_COMMAND, p.MSG_MOTION_STATUS), (0x1B, 0x1C))

    def test_turns(self):
        for angle in (90, 180, 270, 360, -90, -360):
            kind, seq, body = p.parse_frame(p.motion_turn_frame(255, angle, 300))
            self.assertEqual((kind, seq, body[0]), (0x1B,255,1))
            self.assertEqual(body[1], int(angle < 0))
            self.assertEqual(int.from_bytes(body[2:4], "big"), abs(angle)*100)

    def test_moves(self):
        for direction in (0, 90, 180, 270):
            _, _, body = p.parse_frame(p.motion_move_frame(2,direction,1000,300))
            self.assertEqual(int.from_bytes(body[2:4],"big"),direction*100)
            self.assertEqual(int.from_bytes(body[4:6],"big"),1000)

    def test_invalid(self):
        for angle in (0, 361, float("nan")):
            with self.assertRaises(ValueError):
                p.motion_turn_frame(0, angle)
        for distance in (0,10001):
            with self.assertRaises(ValueError):
                p.motion_move_frame(0,0,distance)

    def test_documented_frames_crc(self):
        doc = (ROOT/"docs/f407_motion_debug_handoff.md").read_text(encoding="utf-8")
        for line in doc.splitlines():
            if re.fullmatch(r"(?:[0-9A-F]{2} ){14}[0-9A-F]{2}", line):
                p.parse_frame(bytes.fromhex(line))

    def test_status(self):
        frame=p.build_frame(0x1C,3,bytes((2,1,0x23,0x28,0,0,3,255)))
        s=p.parse_motion_status(frame)
        self.assertEqual(s["progress"],9000)
        self.assertEqual(s["command_sequence"],255)

if __name__ == "__main__":
    unittest.main()
