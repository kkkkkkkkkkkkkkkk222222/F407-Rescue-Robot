import unittest
import math

from tools import vision_protocol as protocol


class VisionProtocolTests(unittest.TestCase):
    def test_uart_baud_matches_firmware(self) -> None:
        self.assertEqual(protocol.UART_BAUD, 115_200)

    def test_config_frame_matches_readme(self) -> None:
        frame = protocol.config_frame(0, 0x11, 1)
        self.assertEqual(
            frame.hex(" ").upper(),
            "A3 B3 11 00 11 01 00 00 00 00 00 00 F0 57 C3",
        )

    def test_every_frame_has_valid_envelope_and_crc(self) -> None:
        frames = (
            protocol.report_frame(
                0x10,
                640,
                512,
                350,
                protocol.pack_counts(1, 0, 0, 0),
                protocol.FLAG_FOUND | protocol.FLAG_CLASS_VALID,
            ),
            protocol.nav_frame(
                0x20,
                protocol.NAV_FORWARD,
                protocol.NAV_EN_ROUTE,
                protocol.DEST_MATERIAL,
            ),
            protocol.stop_frame(0x30),
            protocol.rescue_frame(0x31),
            protocol.build_frame(
                protocol.MSG_STATUS,
                0x41,
                bytes((3, 1, 0, 120, 0x0F, 0, 2, 1)),
            ),
            protocol.build_frame(
                protocol.MSG_ODOM,
                0x42,
                bytes((0, 12, 0xFF, 0xF8, 0, 4, 10, 7)),
            ),
        )
        for frame in frames:
            with self.subTest(frame=frame.hex()):
                self.assertEqual(len(frame), 15)
                self.assertEqual(frame[:2], protocol.FRAME_HEAD)
                self.assertEqual(frame[-1], protocol.FRAME_TAIL)
                self.assertEqual(
                    int.from_bytes(frame[12:14], "little"),
                    protocol.crc16_modbus(frame[2:12]),
                )

    def test_counts_pack_four_two_bit_fields(self) -> None:
        self.assertEqual(protocol.pack_counts(1, 2, 3, 0), 0x39)
        with self.assertRaises(ValueError):
            protocol.pack_counts(4, 0, 0, 0)

    def test_contradictory_reports_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            protocol.report_frame(
                1, 640, 512, 0, 0,
                protocol.FLAG_FOUND | protocol.FLAG_CLASS_VALID,
            )
        with self.assertRaises(ValueError):
            protocol.report_frame(
                2, 0, 0, 0, protocol.pack_counts(1, 0, 0, 0), 0,
            )

    def test_near_safe_requires_hold(self) -> None:
        with self.assertRaises(ValueError):
            protocol.nav_frame(
                1,
                protocol.NAV_FORWARD,
                protocol.NAV_NEAR_SAFE,
                protocol.DEST_MATERIAL,
            )

    def test_rescue_event_value(self) -> None:
        frame = protocol.rescue_frame(0x5A)
        self.assertEqual(frame[2], protocol.MSG_EVENT)
        self.assertEqual(frame[3], 0x5A)
        self.assertEqual(
            frame[4:12], bytes((protocol.EVENT_RESCUE,) + (0,) * 7)
        )

    def test_parse_full_config_ack(self) -> None:
        self.assertEqual(protocol.parse_config_ack(protocol.CONFIG_ACK),
                         {"accepted": True})

    def test_parse_task_status(self) -> None:
        frame = protocol.build_frame(
            protocol.MSG_STATUS,
            10,
            bytes((4, 2, 0, 95, 0x2D, 5, 3, 0x10)),
        )
        status = protocol.parse_status(frame)
        self.assertEqual(status["state"], 4)
        self.assertEqual(status["destination"], 2)
        self.assertEqual(status["remaining_s"], 95)
        self.assertTrue(status["match_started"])
        self.assertTrue(status["grabbed"])
        self.assertTrue(status["cargo_valid"])
        self.assertTrue(status["nav_fresh"])
        self.assertEqual(status["fault"], 5)
        self.assertEqual(status["recovery_count"], 3)

    def test_parse_odometry(self) -> None:
        frame = bytes.fromhex(
            "A3 B3 15 00 12 34 56 78 9A BC 0A 07 90 F6 C3"
        )
        self.assertEqual(
            protocol.parse_odometry(frame),
            {
                "sequence": 0,
                "m1_count": 0x1234,
                "m2_count": 0x5678,
                "m3_count": 0x9ABC,
                "dt_ms": 10,
                "status": 0x07,
            },
        )

        forward, left = protocol.odometry_body_velocity(
            {
                "m1_delta": -10,
                "m2_delta": 0,
                "m3_delta": 10,
                "dt_ms": 20,
            }
        )
        metres_per_count = math.pi * 0.070 / 1768
        self.assertAlmostEqual(
            forward, 20 * metres_per_count / math.sqrt(3) / 0.020
        )
        self.assertAlmostEqual(left, 0.0)


if __name__ == "__main__":
    unittest.main()
