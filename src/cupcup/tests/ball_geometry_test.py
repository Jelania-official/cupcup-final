#!/usr/bin/env python3
"""Check the evaluation-only camera-pose ray projection."""

from __future__ import annotations

import math
import csv
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from compare_ball_geometry import evaluate, oracle_camera_point
from replay_ball_pattern import truth_pixel


class CameraProjectionTest(unittest.TestCase):
    """Exercise Webots camera axes and malformed trace handling."""

    def setUp(self) -> None:
        # Camera looks toward world -X. Local camera +Z is world up (+Y).
        values = (1.0, 0.6, 0.0, -1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0, 1.0, 0.0)
        names = ("camera_x", "camera_y", "camera_z") + tuple(
            f"camera_r{index}" for index in range(9))
        self.row = {f"red_1_{name}": str(value)
                    for name, value in zip(names, values)}

    def test_ball_view_score_rejects_invalid_height_before_reading_inputs(self) -> None:
        scorer = Path(__file__).with_name("score_ball_views.py")
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing"
            output = Path(directory) / "score.json"
            for height in ("nan", "inf", "-inf", ".24", ".51"):
                with self.subTest(height=height):
                    result = subprocess.run(
                        [sys.executable, str(scorer), str(missing), str(missing),
                         "--camera-binary", str(missing), "--camera-root-height=" + height,
                         "--output", str(output)], capture_output=True, text=True, check=False)
                    self.assertEqual(result.returncode, 2)
                    self.assertIn("camera root height must be finite", result.stderr)
                    self.assertFalse(output.exists())

    def test_centered_downward_ray(self) -> None:
        point = oracle_camera_point(self.row, "red_1", 0.5, 0.75)
        self.assertIsNotNone(point)
        expected_distance = 0.53 / (0.5 * math.tan(1.3613 / 2.0) * 480.0 / 640.0)
        self.assertAlmostEqual(point[0], 1.0 - expected_distance)
        self.assertAlmostEqual(point[1], 0.0)

    def test_near_horizon_and_missing_pose_are_rejected(self) -> None:
        self.assertIsNone(oracle_camera_point(self.row, "red_1", 0.5, 0.5))
        self.assertIsNone(oracle_camera_point({}, "red_1", 0.5, 0.75))

    def test_truth_pixel_is_evaluation_only_and_respects_camera_axes(self) -> None:
        row = dict(self.row, ball_x="-1", ball_y="0.6", ball_z="0")
        u, v = truth_pixel(row, "red_1")
        self.assertAlmostEqual(u, 0.5)
        self.assertAlmostEqual(v, 0.5)
        row["ball_y"] = "0.05"
        self.assertGreater(truth_pixel(row, "red_1")[1], 0.5)
        row["ball_x"] = "2"
        self.assertIsNone(truth_pixel(row, "red_1"))
        self.assertIsNone(truth_pixel({}, "red_1"))

    def test_accepted_ball_timestamp_beats_latest_image_timestamp(self) -> None:
        first = dict(self.row)
        first.update({"time": "1.0", "ball_x": "-0.8", "ball_z": "0",
                      "red_1_true_x": "0", "red_1_true_z": "0",
                      "red_1_vx": "0", "red_1_vz": "0",
                      "red_1_omega_y": "0", "red_1_talk": ""})
        second = dict(first)
        second.update({
            "time": "2.0", "ball_x": "5.0",
            "red_1_talk": "cupcup|bstamp_ms=1000|istamp_ms=2000|age=0.1|"
                          "biha_sim=0.02|biia_sim=0.02|iu=0.5|iv=0.75|"
                          "ir=0.05|bihp=20|biip=0",
        })
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.csv"
            with trace.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=first.keys())
                writer.writeheader()
                writer.writerows((first, second))
            result = evaluate(trace, "red", 0.57, 10.5)["red_1"]
        self.assertEqual(result["samples"], 1)
        self.assertAlmostEqual(result["radius"][0], 0.2)
        self.assertEqual(result["pixel_samples"], 1)
        self.assertEqual(len(result["near_pixel_error_px"]), 1)

    def test_invalid_simple_ray_does_not_hide_valid_oracle_or_fk(self) -> None:
        row = dict(self.row)
        row.update({"time": "1.0", "ball_x": "-0.8", "ball_z": "0",
                    "red_1_true_x": "0", "red_1_true_z": "0",
                    "red_1_vx": "0.1", "red_1_vz": "0",
                    "red_1_omega_y": "0"})
        fields = ("cupcup|bstamp_ms=1000|age=0.1|biha_sim=0.02|biia_sim=0.02|"
                  "iu=0.5|iv=0.75|ir=0.05|bihp=-70|biip=0|"
                  "bgvalid=1|bgx=-0.8|bgz=0|bcx=1|bcy=0.6|bcz=0")
        fields += ''.join(f"|bcr{i}={row[f'red_1_camera_r{i}']}" for i in range(9))
        row["red_1_talk"] = fields
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.csv"
            with trace.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=row.keys())
                writer.writeheader()
                writer.writerow(row)
            result = evaluate(trace, "red", .57, 10.5)["red_1"]
        self.assertEqual(result["invalid_ray"], 1)
        self.assertEqual(len(result["moving_oracle_range"]), 1)
        self.assertEqual(result["moving_fk_range"], [0.0])
        self.assertEqual(result["fk_orientation_deg"], [0.0])

    def test_pixels_do_not_require_reported_posture_and_reject_outside_view(self) -> None:
        row = dict(self.row, time="1.0", ball_x="-0.8", ball_y="0.07", ball_z="0",
                   red_1_true_x="0", red_1_true_z="0")
        u, v = truth_pixel(row, "red_1")
        row["red_1_talk"] = (f"cupcup|bstamp_ms=1000|age=0.1|iu={u + 10/640}|"
                             f"iv={v + 5/480}|state=ALIGN")
        outside = dict(row, time="2.0", ball_z="10")
        outside["red_1_talk"] = "cupcup|bstamp_ms=2000|age=0.1|iu=0.5|iv=0.5"
        stale = dict(row, time="3.0")
        stale["red_1_talk"] = "cupcup|bstamp_ms=3000|age=1.0|iu=0.5|iv=0.5"
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "trace.csv"
            with trace.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=row.keys())
                writer.writeheader()
                writer.writerows((row, outside, stale))
            result = evaluate(trace, "red", .57, 10.5)["red_1"]
        self.assertEqual(result["observations"], 3)
        self.assertEqual(result["pixel_samples"], 1)
        self.assertEqual(result["pixel_unscored"], 1)
        self.assertEqual(result["pose_rejected"], 2)
        self.assertEqual(result["samples"], 0)
        self.assertAlmostEqual(result["near_pixel_du_px"][0], 10)
        self.assertAlmostEqual(result["near_pixel_dv_px"][0], 5)
        self.assertAlmostEqual(result["pixel_by_state"]["ALIGN"][0], math.sqrt(125))


if __name__ == "__main__":
    unittest.main()
