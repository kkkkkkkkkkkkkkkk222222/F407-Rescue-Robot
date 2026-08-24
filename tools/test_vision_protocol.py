import unittest

from tools import vision_protocol as protocol


class VisionProtocolTests(unittest.TestCase):
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
                320,
                240,
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
                1, 320, 240, 0, 0,
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


if __name__ == "__main__":
    unittest.main()
