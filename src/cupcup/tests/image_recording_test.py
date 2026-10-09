#!/usr/bin/env python3
"""Verify zero-stamp provenance and RGB/padding handling without a running ROS graph."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
from sensor_msgs.msg import Image
from record_images import ImageRecorder


class ImageRecordingTest(unittest.TestCase):
    """Exercise the real callback on synthetic messages, not fake saved metadata."""

    def message(self, stamp=0):
        """Two padded RGB rows representing a saturated red image."""
        msg = Image()
        msg.width, msg.height, msg.step, msg.encoding = 2, 2, 8, "rgb8"
        msg.header.stamp.sec = stamp
        msg.data = [255, 0, 0, 255, 0, 0, 99, 99] * 2
        return msg

    def test_zero_stamp_uses_explicit_receipt_clock_and_rate_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = SimpleNamespace(directory=root, last_stamp={}, started_ns=1_000_000_000)
            for receipt in (2_000_000_000, 2_100_000_000, 2_500_000_000):
                with patch("record_images.time.monotonic_ns", return_value=receipt):
                    ImageRecorder.capture(recorder, "red_1", self.message())
            rows = [json.loads(line) for line in (root / "capture.jsonl").read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["timestamp_basis"], "receipt_monotonic")
            self.assertIsNone(rows[0]["header_stamp_ms"])
            self.assertEqual(rows[0]["receipt_elapsed_ms"], 1000)
            self.assertEqual(rows[0]["image"], "red_1_receipt_000001000.jpg")
            image = cv2.imread(str(root / rows[0]["image"]))
            self.assertGreater(image[:, :, 2].mean(), 250)
            self.assertLess(image[:, :, 0].mean(), 5)

    def test_stamped_and_receipt_clocks_do_not_share_a_rate_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = SimpleNamespace(directory=root, last_stamp={}, started_ns=1_000_000_000)
            with patch("record_images.time.monotonic_ns", return_value=2_000_000_000):
                ImageRecorder.capture(recorder, "blue_1", self.message())
                ImageRecorder.capture(recorder, "blue_1", self.message(1))
            rows = [json.loads(line) for line in (root / "capture.jsonl").read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1]["image"], "blue_1_000001000.jpg")
            self.assertEqual(rows[1]["header_stamp_ms"], 1000)
            self.assertEqual(rows[1]["timestamp_basis"], "capture_ros_header")

    def test_invalid_buffer_is_not_saved_or_counted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = SimpleNamespace(directory=root, last_stamp={}, started_ns=0)
            msg = self.message()
            msg.data = [0]
            ImageRecorder.capture(recorder, "red_1", msg)
            self.assertEqual(list(root.iterdir()), [])
            self.assertEqual(recorder.last_stamp, {})


if __name__ == "__main__":
    unittest.main()
