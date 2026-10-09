#!/usr/bin/env python3
"""Keep wall duration, simulation duration and kick requests distinct."""

import contextlib
import io
import json
import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from run_match import binary_matches, seed_provenance, summarize


class MatchSummaryTest(unittest.TestCase):
    """Summary remains usable for failed launches and partial traces."""

    def test_trace_binary_rebuild_is_not_mistaken_for_manifest_version(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "binary"
            path.write_bytes(b"version1")
            expected = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertTrue(binary_matches(path, expected))
            path.write_bytes(b"version2")
            self.assertFalse(binary_matches(path, expected))
            self.assertFalse(binary_matches(path.with_name("missing"), expected))

    def test_original_seed_is_requested_but_not_applied(self):
        self.assertFalse(seed_provenance(7601, True)["localization_seed_applied"])
        self.assertTrue(seed_provenance(7601, False)["localization_seed_applied"])
        self.assertFalse(seed_provenance(None, False)["localization_seed_applied"])
        self.assertFalse(seed_provenance(7601, False)["physics_seed_controlled"])

    def test_original_trace_requires_explicit_writer_before_launch(self):
        runner = Path(__file__).with_name("run_match.py")
        result = subprocess.run(
            [sys.executable, str(runner), "--official-launch-compat", "--trace"],
            capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn("original supervisor has no trace writer", result.stderr)

    def test_trace_writer_cannot_silently_replace_experimental_judge(self):
        runner = Path(__file__).with_name("run_match.py")
        result = subprocess.run([sys.executable, str(runner),
                                 "--trace-judge", "/missing", "--trace"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires --official-launch-compat --trace", result.stderr)

    def test_missing_logs_do_not_invent_a_simulation_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                summarize(Path(directory), 180.0, "red", "cupcup", 1)
        self.assertIn("duration_clock=wall_seconds", output.getvalue())
        self.assertNotIn("simulation_seconds=", output.getvalue())
        self.assertIn("kick_transitions=0", output.getvalue())

    def test_trace_duration_is_not_elapsed_wall_time(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "world_trace.csv").write_text(
                "time,ball_x\nnan,0\n0.02,0\nbroken,0\n93.08,1\ninf,1\n",
                encoding="utf-8")
            (root / "cupcup.log").write_text("blue_1 kick foot=left\n", encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                summarize(root, 192.0, "blue", "cupcup", 4104)
        self.assertIn("match_seconds=192.0", output.getvalue())
        self.assertIn("simulation_seconds=93.06", output.getvalue())
        self.assertIn("kick_transitions=1", output.getvalue())

    def test_defender_explanation_does_not_double_count_kick_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "cupcup.log").write_text(
                "blue_1 kick foot=right lane_offset=0\n"
                "blue_2 kick foot=right lane_offset=12\n"
                "blue_2 defender clear kick foot=right\n", encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                summarize(root, 30, "blue", "unirobot", 1)
        self.assertIn("kick_transitions=2", output.getvalue())

    def test_invalid_duration_is_rejected_before_launch(self):
        runner = Path(__file__).with_name("run_match.py")
        for duration in ("nan", "inf", "0", "-1"):
            with self.subTest(duration=duration):
                result = subprocess.run(
                    [sys.executable, str(runner), "--duration", duration],
                    capture_output=True, text=True, check=False)
                self.assertEqual(result.returncode, 2)
                self.assertIn("finite and positive", result.stderr)

    def test_invalid_settle_evidence_is_rejected_before_launch(self):
        runner = Path(__file__).with_name("run_match.py")
        for frames in ("2", "21", "-1"):
            result = subprocess.run([sys.executable, str(runner), "--kick-settle-frames", frames],
                                    capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 2)
            self.assertIn("must be 3..20", result.stderr)

    def test_released_referee_clock_is_explicit_and_short_run_not_formal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "released_referee.json").write_text(json.dumps({
                "referee_play_seconds": 10, "end_observed": True,
                "formal_900_completed": False}), encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                summarize(root, 30.0, "red", "unirobot", 1)
        self.assertIn("match_seconds=30.0", output.getvalue())
        self.assertIn("referee_play_seconds=10", output.getvalue())
        self.assertIn("formal_900_completed=False", output.getvalue())

    def test_released_referee_requires_original_launch_before_starting(self):
        runner = Path(__file__).with_name("run_match.py")
        result = subprocess.run([sys.executable, str(runner), "--released-referee", "/missing"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires --official-launch-compat", result.stderr)


if __name__ == "__main__":
    unittest.main()
