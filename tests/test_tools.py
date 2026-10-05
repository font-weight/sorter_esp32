"""Offline unit tests. No serial port or board is required."""
import io
import json
from pathlib import Path
import unittest
import tempfile
from unittest.mock import patch

import numpy as np

from tools import calibrate
from tools.analyze_run import RunLogError, analyze, duration_stats, read_log
from tools.protocol import (Detection, LineParser, Packet, ProtocolError, Scene, crc16,
                            decode_packet, decode_scene, encode_packet, encode_scene, parse_i32, parse_u32)
from tools.serial_tool import CaptureError, read_capture, rgb565be_to_rgb


ROOT = Path(__file__).resolve().parents[1]


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_protocol_roundtrip_all_types(self):
        scene = Scene(4294967295, 12, (Detection(1, -2147483648, 2147483647, 4294967295),))
        for packet in (Packet("Q", 1, 1, "12"), Packet("D", 17, 4294967295, encode_scene(scene)), Packet("E", 1, 2, "CONFLICT")):
            self.assertEqual(decode_packet(encode_packet(packet)), packet)
        self.assertEqual(decode_scene(encode_scene(scene)), scene)

    def test_empty_scene_and_max_count(self):
        for objects in ((), tuple(Detection(i % 3 + 1, i, -i, 10) for i in range(8))):
            scene = Scene(1, 2, objects)
            self.assertEqual(decode_scene(encode_scene(scene)), scene)
        with self.assertRaises(ProtocolError):
            encode_scene(Scene(1, 2, tuple(Detection(1, 1, 1, 1) for _ in range(9))))

    def test_frame_corruption_is_detected(self):
        frame = encode_packet(Packet("Q", 21, 22, "73"))
        for index in range(1, frame.index(b"*")):
            damaged = bytearray(frame)
            damaged[index] ^= 1
            with self.assertRaises(ProtocolError):
                decode_packet(damaged)

    def test_strict_numeric_parsing(self):
        for value in ("", "-1", "+1", " 1", "1 ", "1x", "4294967296", "1.0", "١"):
            with self.assertRaises(ProtocolError):
                parse_u32(value)
        for value in ("2147483648", "-2147483649", "+1", "--1", "1e0"):
            with self.assertRaises(ProtocolError):
                parse_i32(value)
        self.assertEqual(parse_i32("-2147483648"), -2147483648)
        self.assertEqual(parse_u32("4294967295"), 4294967295)

    def test_payload_syntax_and_counts(self):
        for value in ("1,2,1", "1,2,0;", "1,2,1;4,1,1,20", "1,2,1;1,1,1,0",
                      "0,2,0", "1,0,0", "1,2,1;1,2,3,4,5", "1,2,9"):
            with self.assertRaises(ProtocolError):
                decode_scene(value)
        for packet in (Packet("Q", 1, 1, "0"), Packet("E", 1, 1, "has space"),
                       Packet("E", 1, 1, "BAD@CODE"), Packet("Q", 0, 1, "3")):
            with self.assertRaises(ProtocolError):
                encode_packet(packet)

    def test_stream_fragmentation_and_recovery(self):
        frame = encode_packet(Packet("Q", 1, 1, "42"))
        parser = LineParser()
        received = []
        for byte in b"boot message\n@broken\n" + frame + b"@" + b"x" * 800 + b"\n" + frame:
            received.extend(parser.feed(bytes([byte])))
        self.assertEqual(received, [Packet("Q", 1, 1, "42")] * 2)
        self.assertEqual(parser.errors, 2)

    def test_crlf_and_nul_recovery_match_cpp(self):
        packet = Packet("Q", 1, 1, "42")
        frame = encode_packet(packet)
        self.assertEqual(decode_packet(frame[:-1] + b"\r\n"), packet)
        parser = LineParser()
        self.assertEqual(parser.feed(b"@partial\x00"), [])
        self.assertEqual(parser.errors, 1)
        self.assertEqual(parser.feed(frame), [packet])

    def test_golden_vectors(self):
        data = json.loads((ROOT / "examples/protocol_vectors.json").read_text(encoding="utf-8"))
        for vector in data["vectors"]:
            p = Packet(vector["type"], vector["session"], vector["seq"], vector["payload"])
            self.assertEqual(encode_packet(p).decode("ascii"), vector["frame"])


class CalibrationTests(unittest.TestCase):
    def test_cli_never_overwrites_source_with_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "measured.csv"
            original = b"split,u,v,x_mm,y_mm\n"
            source.write_bytes(original)
            with patch("sys.stderr", new_callable=io.StringIO), self.assertRaises(SystemExit) as result:
                calibrate.main([str(source), "--output", str(source), "--plane-height-mm", "10"])
            self.assertEqual(result.exception.code, 2)
            self.assertEqual(source.read_bytes(), original)

    def test_dimensions_fit_generated_uint16_header(self):
        for width in (True, 65536, 1):
            with self.assertRaises(calibrate.CalibrationError):
                calibrate.build_calibration([], width=width)

    def setUp(self):
        self.h = np.array([[0.42, 0.025, -70], [0.015, 0.4, 55], [0.0006, -0.0004, 1]])
        self.train = np.array([(u, v) for v in (25, 120, 215) for u in (30, 160, 290)])
        self.test = np.array([(95, 72.5), (225, 72.5), (95, 167.5), (225, 167.5)])
        self.rows = []
        for split, points in (("train", self.train), ("test", self.test)):
            for pixel, world in zip(points, calibrate.project_points(self.h, points)):
                self.rows.append({"split": split, "image": pixel.tolist(), "world": world.tolist()})

    def test_recovers_perspective_on_independent_points(self):
        h, condition = calibrate.fit_homography(self.train, calibrate.project_points(self.h, self.train))
        self.assertLess(condition, 100)
        np.testing.assert_allclose(calibrate.project_points(h, self.test), calibrate.project_points(self.h, self.test), atol=1e-10)

    def test_normalization_handles_large_coordinate_offset(self):
        src = self.train + 1000000
        target = src * [0.2, 0.5] + [9000000, -4000000]
        h, _ = calibrate.fit_homography(src, target)
        np.testing.assert_allclose(calibrate.project_points(h, src), target, atol=1e-5)

    def test_degenerate_collinear_and_duplicates(self):
        line = np.array([(i, 2*i) for i in range(9)], dtype=float)
        for src, dst in ((line, line + 3), (self.train, line)):
            with self.assertRaises(calibrate.CalibrationError):
                calibrate.fit_homography(src, dst)
        duplicated = self.train.copy()
        duplicated[-1] = duplicated[0]
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.fit_homography(duplicated, calibrate.project_points(self.h, self.train))

    def test_holdout_errors_cannot_be_hidden_by_refitting(self):
        rows = [dict(row, world=[row["world"][0]+5, row["world"][1]]) if row["split"] == "test" else dict(row)
                for row in self.rows]
        result = calibrate.build_calibration(rows, provenance="measured")
        self.assertLess(result["training"]["max_mm"], 1e-9)
        self.assertAlmostEqual(result["independent_test"]["max_mm"], 5, places=8)
        self.assertFalse(result["numerical_test_passed"])
        self.assertFalse(result["calibration_confirmed"])
        self.assertFalse(result["confirmation_permitted"])

    def test_support_is_limited_and_calibration_stays_unconfirmed(self):
        result = calibrate.build_calibration(self.rows, provenance="synthetic", valid_rect=[40, 35, 280, 205])
        self.assertTrue(result["numerical_test_passed"])
        self.assertFalse(result["confirmation_permitted"])
        self.assertFalse(result["calibration_confirmed"])
        self.assertEqual(result["support_pixel_polygon"], [[40, 35], [280, 35], [280, 205], [40, 205]])
        self.assertIn("CALIBRATION_CONFIRMED = false", calibrate.cpp_header(result))
        for rect in ([0, 0, 319, 239], [40, 35, 70, 50]):
            with self.assertRaises(calibrate.CalibrationError):
                calibrate.build_calibration(self.rows, valid_rect=rect)

    def test_rejects_leaked_holdout_and_frame_mismatch(self):
        leaked = self.rows + [dict(self.rows[0], split="test")]
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.build_calibration(leaked)
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.build_calibration(self.rows, width=160, height=120)
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.build_calibration(self.rows[:9])

    def test_inside_convex_both_windings(self):
        poly = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
        for boundary in (poly, poly[::-1]):
            self.assertEqual(calibrate.inside_convex([[5, 5], [10, 10], [11, 5]], boundary).tolist(), [True, True, False])

    def test_checked_example_is_synthetic(self):
        result = calibrate.build_calibration(calibrate.read_points(ROOT / "examples/synthetic_calibration_points.csv"), provenance="synthetic")
        self.assertLess(result["independent_test"]["max_mm"], 0.2)
        self.assertFalse(result["calibration_confirmed"])


class RunAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.rows = read_log(ROOT / "examples/synthetic_run.csv")

    def test_failures_and_retry_remain_in_denominators(self):
        result = analyze(self.rows, "synthetic", expected_parts=8)
        self.assertEqual(result["all_attempts"], 9)
        self.assertEqual(result["logged_trials"], 8)
        self.assertEqual(result["first_attempt_successes"], 3)
        self.assertEqual(result["first_attempt_success_rate"], 3/8)
        self.assertEqual(result["eventual_successes"], 4)
        self.assertEqual(result["failed_attempts"], 5)
        self.assertEqual(result["all_attempt_durations"]["max_s"], 30)
        self.assertAlmostEqual(result["all_attempt_durations"]["p95_s"], 25.2)

    def test_unlogged_planned_trials_count_against_primary_rate(self):
        result = analyze(self.rows, expected_parts=10)
        self.assertEqual(result["unlogged_planned_trials"], 2)
        self.assertEqual(result["first_attempt_success_rate"], 0.3)

    def test_gapped_duplicate_or_post_success_retry_rejected(self):
        for rows in (self.rows + [self.rows[0]], [dict(self.rows[0], attempt_index=2)],
                     [self.rows[0], dict(self.rows[0], attempt_index=2)]):
            with self.assertRaises(RunLogError):
                analyze(rows)

    def test_inconsistent_success_row_rejected(self):
        content = (ROOT / "examples/synthetic_run.csv").read_text(encoding="utf-8")
        invalid = content.replace("p01,1,1,1,success", "p01,1,1,2,success")
        with patch.object(Path, "open", return_value=io.StringIO(invalid)):
            with self.assertRaises(RunLogError):
                read_log("bad.csv")

    def test_empty_or_no_success_durations(self):
        with self.assertRaises(RunLogError):
            analyze([])
        self.assertIsNone(duration_stats([])["p95_s"])


class CameraCaptureTests(unittest.TestCase):
    def test_rgb565be_primary_colors(self):
        raw = bytes.fromhex("f800 07e0 001f ffff 0000")
        self.assertEqual(rgb565be_to_rgb(raw, 5, 1), bytes([255,0,0, 0,255,0, 0,0,255, 255,255,255, 0,0,0]))

    def test_capture_binary_newlines_do_not_break_framing(self):
        raw = b"\x0a\x00\xff\x0a"
        stream = io.BytesIO(b"camera ready\r\nRGB565BE 2 1 4\n" + raw + b"\nEND\n")
        self.assertEqual(read_capture(stream), (2, 1, raw))

    def test_bad_size_or_terminator_rejected(self):
        for data in (b"RGB565BE 2 1 5\n12345\nEND\n", b"RGB565BE 1 1 2\nab\nBAD\n"):
            with self.assertRaises(CaptureError):
                read_capture(io.BytesIO(data))

    def test_truncated_frame_times_out(self):
        with self.assertRaises(CaptureError):
            read_capture(io.BytesIO(b"RGB565BE 2 1 4\n12"), timeout_s=0.002)

    def test_camera_error_is_reported_without_waiting_for_timeout(self):
        with self.assertRaisesRegex(CaptureError, "UART_DISABLED"):
            read_capture(io.BytesIO(b"ERROR USB_CAMERA_COMMAND_REQUIRES_UART_DISABLED\n"))


if __name__ == "__main__":
    unittest.main()
