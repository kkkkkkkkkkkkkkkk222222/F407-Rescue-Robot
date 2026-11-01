"""Offline protocol evidence; no hardware, SDK, or physical timing acceptance."""
import subprocess
import unittest
from tools import f405_timed_protocol as p
from tools import vision_protocol as legacy


class TimedQualityChecks(unittest.TestCase):
    def test_actual_c_frames(self):
        result = subprocess.run(["build/test_timed_telemetry.exe"], capture_output=True, text=True, check=True)
        frames = [bytes.fromhex(line[6:]) for line in result.stdout.splitlines() if line.startswith("FRAME ")]
        self.assertGreaterEqual(len(frames), 25)
        decoded = [p.decode(f) for f in frames]
        pages = [f["page"] for f in decoded if f["type"] == p.QUALITY_REPLY]
        self.assertEqual(pages, [0, 1])
        for f in decoded:
            if f["type"] == p.WHEEL:
                self.assertFalse(f["flags"] & 2)
                self.assertEqual(f["source_uncertainty_us"], 0xffffffff)
            if f["type"] == p.GYRO:
                for source in ("yaw", "rate"):
                    self.assertTrue(f[source+"_flags"] & 512)
                    self.assertEqual(f[source+"_uncertainty_us"], 0xffffffff)
        self.assertTrue(p.inspect_capture(b"".join(frames))["stream_integrity_pass"])

    def test_prefixes_not_hidden(self):
        extended = p.request(1, 123)
        old = legacy.build_frame(0x15, 1, bytes(8))
        for prefix, real in ((b"\xd4\x4d", old), (b"\xa3\xb3", extended), (b"\xa3\xb3", old)):
            report = p.inspect_capture(prefix+real)
            self.assertFalse(report["stream_integrity_pass"])
            self.assertEqual(len(report["rejected_bytes"]), 2)
            self.assertEqual(report["valid_frames"][0]["offset"], 2)

    def test_crc_required(self):
        bad = bytearray(p.request(1, 1)); bad[-3] ^= 1
        report = p.inspect_capture(bytes(bad))
        self.assertFalse(report["stream_integrity_pass"])
        self.assertEqual(report["valid_frames"], [])

    def test_partial_is_not_valid(self):
        raw = p.request(1, 3)
        report = p.inspect_capture(raw[:-2])
        self.assertFalse(report["stream_integrity_pass"])
        self.assertEqual(report["valid_frames"], [])
        self.assertEqual(bytes.fromhex(report["trailing_partial_hex"]), raw[:-2])

    def test_magic_inside_legacy_payload(self):
        old = legacy.build_frame(0x15, 7, b"\xd4\x4d\xa3\xb3\x00\x00\x00\x00")
        report = p.inspect_capture(old+p.request(3, 42))
        self.assertTrue(report["stream_integrity_pass"])
        self.assertEqual([f["family"] for f in report["valid_frames"]], ["legacy", "extended"])

    def test_quality_length_and_schema(self):
        self.assertEqual(len(p.request(11, 1, quality_page=1)), 21)
        with self.assertRaises(ValueError):
            p.request(11, 1, quality_page=2)
        with self.assertRaises(ValueError):
            p.decode(p.packet(12, 1, bytes(160)))

    def test_observed_scale_is_an_interval(self):
        first = dict(boot_epoch="01", response_prepare_us=1000000,
                     host_write_before_ns=1000000000, receive_monotonic_ns=1002000000)
        second = dict(boot_epoch="01", response_prepare_us=11000000,
                      host_write_before_ns=11000000000, receive_monotonic_ns=11002000000)
        low, high = p.observed_scale_interval(first, second)
        self.assertLess(low, 1.0); self.assertGreater(high, 1.0)
        second["boot_epoch"] = "02"
        with self.assertRaises(ValueError):
            p.observed_scale_interval(first, second)


if __name__ == "__main__":
    unittest.main()
