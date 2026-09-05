import math
import unittest

from tools import vision_protocol as protocol


class VisionProtocolTests(unittest.TestCase):
    def test_uart_and_config_match_upper_computer(self) -> None:
        self.assertEqual(protocol.UART_BAUD, 115_200)
        self.assertEqual(
            protocol.config_frame(0, 0x11, 1).hex(" ").upper(),
            "A3 B3 11 00 11 01 00 00 00 00 00 00 F0 57 C3",
        )

    def test_native_resolution_report_matches_upper_computer(self) -> None:
        frame = protocol.report_frame(
            0x10, 640, 512, 0, 1,
            protocol.FLAG_FOUND | protocol.FLAG_CLASS_VALID,
        )
        self.assertEqual(
            frame.hex(" ").upper(),
            "A3 B3 12 10 02 80 02 00 00 00 01 09 DD FD C3",
        )

    def test_distance_valid_report_matches_upper_computer(self) -> None:
        frame = protocol.report_frame(
            0x10, 640, 512, 350, 1,
            protocol.FLAG_FOUND | protocol.FLAG_CLASS_VALID |
            protocol.FLAG_DISTANCE_VALID,
        )
        self.assertEqual(
            frame.hex(" ").upper(),
            "A3 B3 12 10 02 80 02 00 01 5E 01 49 BC 23 C3",
        )

    def test_each_single_cargo_class_uses_one_count_field(self) -> None:
        for counts in (0x01, 0x04, 0x10, 0x40):
            with self.subTest(counts=counts):
                frame = protocol.report_frame(
                    0x20 + counts.bit_length(), 640, 512, 0, counts,
                    protocol.FLAG_FOUND | protocol.FLAG_CLASS_VALID,
                )
                self.assertEqual(protocol.parse_frame(frame)[2][6], counts)

    def test_no_target_payload_is_all_zero(self) -> None:
        frame = protocol.report_frame(0x11, 0, 0, 0, 0, 0)
        self.assertEqual(frame[4:12], bytes(8))
        with self.assertRaises(ValueError):
            protocol.report_frame(0x12, 1, 0, 0, 0, 0)

    def test_native_resolution_bounds(self) -> None:
        protocol.report_frame(1, 1279, 1023, 0, 1,
                              protocol.FLAG_FOUND |
                              protocol.FLAG_CLASS_VALID)
        with self.assertRaises(ValueError):
            protocol.report_frame(2, 1280, 0, 0, 1,
                                  protocol.FLAG_FOUND)

    def test_mission_waypoint_matches_upper_computer(self) -> None:
        frame = protocol.mission_frame(
            0x20,
            protocol.CMD_NAVIGATE_WAYPOINT,
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_RED_SIDE,
            0, 950, 9000,
        )
        self.assertEqual(
            frame.hex(" ").upper(),
            "A3 B3 18 20 03 0B 00 00 03 B6 23 28 6B E0 C3",
        )

    def test_heading_distance_and_return_commands(self) -> None:
        flags = (
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_USE_FINAL_HEADING | protocol.CMD_DISTANCE_VALID
        )
        outward = protocol.mission_frame(
            0x21, protocol.CMD_NAVIGATE_WAYPOINT, flags,
            1374, 0, 12821,
        )
        returning = protocol.mission_frame(
            0x22, protocol.CMD_RETURN_CENTER, flags,
            480, 0, 26915,
        )
        self.assertEqual(protocol.parse_frame(outward)[2][2:6],
                         bytes.fromhex("05 5E 00 00"))
        self.assertEqual(protocol.parse_frame(returning)[2][2:6],
                         bytes.fromhex("01 E0 00 00"))

    def test_stm_status_matches_upper_computer(self) -> None:
        frame = protocol.stm_status_frame(9, 0x09, 2, 7350, 8, 0)
        self.assertEqual(
            frame.hex(" ").upper(),
            "A3 B3 17 09 09 02 1C B6 08 00 00 00 80 54 C3",
        )
        status = protocol.parse_stm_status(frame)
        self.assertTrue(status["claw_visible"])
        self.assertTrue(status["auto_approach"])
        self.assertEqual(status["camera_pitch_cdeg"], 7350)
        self.assertEqual(status["acknowledged_sequence"], 8)

    def test_fused_pose_signed_coordinates(self) -> None:
        frame = protocol.fused_pose_frame(
            0x43, 1200, -350, 9000, 0x33, 0x94,
        )
        pose = protocol.parse_fused_pose(frame)
        self.assertEqual(pose["x_mm"], 1200)
        self.assertEqual(pose["y_mm"], -350)
        self.assertEqual(pose["heading_cdeg"], 9000)

    def test_odometry_and_body_velocity(self) -> None:
        frame = bytes.fromhex(
            "A3 B3 15 00 12 34 56 78 9A BC 0A 07 90 F6 C3"
        )
        self.assertEqual(protocol.parse_odometry(frame)["dt_ms"], 10)
        forward, left = protocol.odometry_body_velocity(
            {"m1_delta": -10, "m2_delta": 0,
             "m3_delta": 10, "dt_ms": 20}
        )
        metres_per_count = math.pi * 0.070 / 1768
        self.assertAlmostEqual(
            forward, 20 * metres_per_count / math.sqrt(3) / 0.020
        )
        self.assertAlmostEqual(left, 0.0)

    def test_all_supported_frames_have_valid_crc(self) -> None:
        frames = (
            protocol.config_frame(1, 0x12, 4),
            protocol.report_frame(2, 640, 512, 0, 1,
                                  protocol.FLAG_FOUND |
                                  protocol.FLAG_CLASS_VALID),
            protocol.fused_pose_frame(3, 0, 0, 0, 3, 0),
            protocol.mission_frame(4, protocol.CMD_GRAB_CONFIRMED),
            protocol.stm_status_frame(5, 0, 1, 9000, 4, 0),
            protocol.build_frame(protocol.MSG_ODOM, 6,
                                 bytes((0, 1, 0, 2, 0, 3, 10, 7))),
        )
        for frame in frames:
            with self.subTest(frame=frame.hex()):
                self.assertEqual(
                    int.from_bytes(frame[12:14], "little"),
                    protocol.crc16_modbus(frame[2:12]),
                )


if __name__ == "__main__":
    unittest.main()
