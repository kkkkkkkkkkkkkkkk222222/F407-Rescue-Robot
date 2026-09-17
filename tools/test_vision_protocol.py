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

    def test_complete_flow_commands_use_existing_0x18_frame(self) -> None:
        approach = protocol.mission_frame(
            0x31, protocol.CMD_APPROACH_TARGET,
            protocol.CMD_VALID, 640, 512, 0,
        )
        disperse = protocol.mission_frame(
            0x32, protocol.CMD_DISPERSE_PILE,
            protocol.CMD_VALID | protocol.CMD_SIDE_VALID,
        )
        self.assertEqual(protocol.parse_frame(approach)[2][0],
                         protocol.CMD_APPROACH_TARGET)
        self.assertEqual(protocol.parse_frame(approach)[2][2:6],
                         bytes.fromhex("02 80 02 00"))
        self.assertEqual(protocol.parse_frame(disperse)[2][0],
                         protocol.CMD_DISPERSE_PILE)

    def test_cluster_target_flag_is_approach_only(self) -> None:
        approach = protocol.mission_frame(
            0x34, protocol.CMD_APPROACH_TARGET,
            protocol.CMD_VALID | protocol.CMD_CLUSTER_TARGET,
            640, 700, 0,
        )
        self.assertEqual(
            protocol.parse_frame(approach)[2][1],
            protocol.CMD_VALID | protocol.CMD_CLUSTER_TARGET,
        )
        with self.assertRaises(ValueError):
            protocol.mission_frame(
                0x35, protocol.CMD_HOLD,
                protocol.CMD_VALID | protocol.CMD_CLUSTER_TARGET,
            )

    def test_disperse_side_flags_are_disperse_only(self) -> None:
        disperse = protocol.mission_frame(
            0x36, protocol.CMD_DISPERSE_PILE,
            protocol.CMD_VALID | protocol.CMD_SIDE_VALID |
            protocol.CMD_TARGET_RIGHT,
        )
        self.assertEqual(
            protocol.parse_frame(disperse)[2][1],
            protocol.CMD_VALID | protocol.CMD_SIDE_VALID |
            protocol.CMD_TARGET_RIGHT,
        )
        whole_pile = protocol.mission_frame(
            0x37, protocol.CMD_DISPERSE_PILE, protocol.CMD_VALID,
        )
        self.assertEqual(
            protocol.parse_frame(whole_pile)[2][1], protocol.CMD_VALID,
        )
        with self.assertRaises(ValueError):
            protocol.mission_frame(
                0x38, protocol.CMD_DISPERSE_PILE,
                protocol.CMD_VALID | protocol.CMD_TARGET_RIGHT,
            )
        with self.assertRaises(ValueError):
            protocol.mission_frame(
                0x39, protocol.CMD_HOLD,
                protocol.CMD_VALID | protocol.CMD_SIDE_VALID,
            )

        bump = protocol.mission_frame(
            0x3A, protocol.CMD_DISPERSE_PILE,
            protocol.CMD_VALID | protocol.CMD_FIRST_GREEN_BUMP,
        )
        self.assertEqual(
            protocol.parse_frame(bump)[2][1],
            protocol.CMD_VALID | protocol.CMD_FIRST_GREEN_BUMP,
        )
        with self.assertRaises(ValueError):
            protocol.mission_frame(
                0x3B, protocol.CMD_DISPERSE_PILE,
                protocol.CMD_VALID | protocol.CMD_FIRST_GREEN_BUMP |
                protocol.CMD_SIDE_VALID,
            )

    def test_staged_safe_zone_delivery_commands(self) -> None:
        side = protocol.CMD_RED_SIDE
        stage_flags = (
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_USE_FINAL_HEADING | protocol.CMD_DISTANCE_VALID |
            protocol.CMD_STAGE_ONLY | side
        )
        stage = protocol.mission_frame(
            0x40, protocol.CMD_NAVIGATE_WAYPOINT, stage_flags,
            400, 0, 9000,
        )
        pose_align = protocol.mission_frame(
            0x41, protocol.CMD_ALIGN_SAFE_ZONE,
            protocol.CMD_VALID | protocol.CMD_USE_FINAL_HEADING | side,
            0, 0, 9000,
        )
        visual_align = protocol.mission_frame(
            0x42, protocol.CMD_ALIGN_SAFE_ZONE,
            protocol.CMD_VALID |
            protocol.CMD_VISUAL_CORRECTION_VALID | side,
            -52, 0, 0,
        )
        enter = protocol.mission_frame(
            0x43, protocol.CMD_ENTER_SAFE_ZONE,
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_VISUAL_CORRECTION_VALID | side,
            0, 0, 0,
        )
        self.assertEqual(protocol.parse_frame(stage)[2][1], stage_flags)
        self.assertEqual(
            int.from_bytes(protocol.parse_frame(pose_align)[2][6:8], "big"),
            9000,
        )
        self.assertEqual(
            int.from_bytes(
                protocol.parse_frame(visual_align)[2][2:4],
                "big", signed=True,
            ),
            -52,
        )
        self.assertEqual(
            int.from_bytes(protocol.parse_frame(enter)[2][2:4], "big"),
            0,
        )

        fallback = protocol.mission_frame(
            0x44, protocol.CMD_ENTER_SAFE_ZONE,
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_USE_FINAL_HEADING | side,
            0, 0, 9000,
        )
        self.assertEqual(
            int.from_bytes(protocol.parse_frame(fallback)[2][6:8], "big"),
            9000,
        )

        blue_visual = protocol.mission_frame(
            0x45, protocol.CMD_ENTER_SAFE_ZONE,
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_VISUAL_CORRECTION_VALID,
        )
        blue_fallback = protocol.mission_frame(
            0x46, protocol.CMD_ENTER_SAFE_ZONE,
            protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
            protocol.CMD_USE_FINAL_HEADING,
            0, 0, 27000,
        )
        self.assertEqual(protocol.parse_frame(blue_visual)[2][1], 0x43)
        self.assertEqual(protocol.parse_frame(blue_fallback)[2][1], 0x07)

    def test_visual_align_and_enter_reject_mixed_semantics(self) -> None:
        with self.assertRaises(ValueError):
            protocol.mission_frame(
                0x44, protocol.CMD_ALIGN_SAFE_ZONE,
                protocol.CMD_VALID | protocol.CMD_USE_FINAL_HEADING |
                protocol.CMD_VISUAL_CORRECTION_VALID,
                20, 0, 9000,
            )
        with self.assertRaises(ValueError):
            protocol.mission_frame(
                0x45, protocol.CMD_ENTER_SAFE_ZONE,
                protocol.CMD_VALID | protocol.CMD_DRIVE_STRAIGHT |
                protocol.CMD_DISTANCE_VALID |
                protocol.CMD_VISUAL_CORRECTION_VALID,
                200, 0, 9000,
            )

    def test_two_frame_normal_audit_and_unknown_side_audit(self) -> None:
        first = protocol.cargo_audit_frame(
            0x50,
            protocol.CARGO_GREEN,
            protocol.CARGO_NONE,
            1, 0, 0, 10, 1,
        )
        second = protocol.cargo_audit_frame(
            0x51,
            protocol.CARGO_GREEN,
            protocol.CARGO_NONE,
            1, 0, protocol.AUDIT_STABLE, 11, 1,
        )
        first_payload = protocol.parse_frame(first)[2]
        second_payload = protocol.parse_frame(second)[2]
        self.assertEqual(first_payload[6], 10)
        self.assertEqual(second_payload[6], 11)
        self.assertFalse(first_payload[5] & protocol.AUDIT_STABLE)
        self.assertTrue(second_payload[5] & protocol.AUDIT_STABLE)
        self.assertEqual(first_payload[2:5], second_payload[2:5])

        unassigned = protocol.cargo_audit_frame(
            0x52,
            protocol.CARGO_GREEN,
            protocol.CARGO_NONE,
            1, 0, protocol.AUDIT_UNKNOWN_PRESENT, 12, 2,
        )
        unknown_payload = protocol.parse_frame(unassigned)[2]
        self.assertEqual(unknown_payload[7], 2)
        self.assertEqual((unknown_payload[4] & 0x03), 1)
        self.assertTrue(unknown_payload[5] & protocol.AUDIT_UNKNOWN_PRESENT)

    def test_pause_uses_reserved_mission_code(self) -> None:
        pause = protocol.mission_frame(
            0x33, protocol.CMD_PAUSE, protocol.CMD_VALID,
        )
        self.assertEqual(protocol.parse_frame(pause)[2][0], 0x01)

    def test_stm_status_matches_upper_computer(self) -> None:
        frame = protocol.stm_status_frame(9, 0x29, 2, 7350, 8, 0)
        self.assertEqual(
            frame.hex(" ").upper(),
            "A3 B3 17 09 29 02 1C B6 08 00 00 00 82 4C C3",
        )
        status = protocol.parse_stm_status(frame)
        self.assertTrue(status["claw_visible"])
        self.assertTrue(status["auto_approach"])
        self.assertTrue(status["distance_done"])
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
