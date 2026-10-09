#!/usr/bin/env python3
"""Smoke and determinism checks for the visual strategy sandbox."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SANDBOX_BINARY = sys.argv.pop(1)


class TacticalSandboxTest(unittest.TestCase):
    def test_custom_initial_state_and_invalid_layouts(self) -> None:
        values = [1, -.5, 120, 2, .5, 180, -1, .5, -30, -2, -.5, 0,
                  .3, -.2, .6, -.4]
        args = ("--duration", "1", "--initial-state", " ".join(map(str, values)))
        result, data = self.run_simulation(*args)
        repeat, _ = self.run_simulation(*args)
        self.assertEqual(result.stdout, repeat.stdout)
        self.assertEqual(data["parameters"]["initial_state"], values)
        first = data["frames"][0]
        self.assertEqual(first["ball"], values[12:14])
        self.assertEqual(first["ball_velocity"], values[14:16])
        for i, robot in enumerate(first["red"] + first["blue"]):
            self.assertEqual([robot["x"], robot["z"], robot["yaw"]], values[3*i:3*i+3])
            self.assertIsNone(robot["mapped_ball"])
        for invalid in ("1 2", " ".join(map(str, values)) + " 1", "nan " * 16,
                        "0 " * 16):
            failed = subprocess.run([SANDBOX_BINARY, "--initial-state", invalid],
                                    capture_output=True)
            self.assertEqual(failed.returncode, 2)

    def test_final_only_matches_full_recording_without_changing_simulation(self) -> None:
        for mode in ("oracle", "realistic"):
            for color in ("red", "blue"):
                with self.subTest(mode=mode, color=color):
                    args = ("--duration", "30", "--seed", "101", "--mode", mode,
                            "--cupcup-color", color, "--opponent", "shared")
                    _, full = self.run_simulation(*args)
                    _, final = self.run_simulation(*args, "--frames", "final")
                    self.assertEqual(final.pop("frames"), [full.pop("frames")[-1]])
                    self.assertEqual(final.pop("recording_mode"), "final")
                    self.assertEqual(full.pop("recording_mode"), "full")
                    self.assertEqual(full, final)
        invalid = subprocess.run([SANDBOX_BINARY, "--frames", "none"], capture_output=True)
        self.assertNotEqual(invalid.returncode, 0)

    def test_legacy_visibility_is_explicit_and_invalid_models_rejected(self) -> None:
        _, legacy = self.run_simulation("--duration", "1", "--vision-model", "legacy")
        self.assertEqual(legacy["horizontal_vision_revision"], 1)
        self.assertEqual(legacy["parameters"]["horizontal_fov_deg"], 190)
        invalid = subprocess.run([SANDBOX_BINARY, "--vision-model", "magic"],
                                 text=True, capture_output=True)
        self.assertNotEqual(invalid.returncode, 0)

    def test_evaluation_records_selected_source_root(self) -> None:
        evaluator = Path(__file__).with_name("evaluate_tactics.py")
        source = Path(__file__).resolve().parents[1] / "src"
        with tempfile.TemporaryDirectory(prefix="cupcup-source-snapshot-") as directory:
            command = [sys.executable, str(evaluator), SANDBOX_BINARY, "--output",
                       str(Path(directory) / "report.json"), "--source-root", str(source),
                       "--seeds", "1", "--duration", "1", "--split", "development",
                       "--opponents", "block", "--scenarios", "kickoff"]
            subprocess.run(command, capture_output=True, text=True, check=True)
            report = json.loads((Path(directory) / "report.json").read_text())
            self.assertEqual(report["source_root"], str(source))
            self.assertFalse(report["source_changed_during_run"])
            self.assertEqual(len(report["source_sha256"]), 6)
            command[command.index("--source-root") + 1] = directory
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 2)
            self.assertIn("source root must contain", result.stderr)

    def test_evaluation_rejects_invalid_observation_noise(self) -> None:
        evaluator = Path(__file__).with_name("evaluate_tactics.py")
        for noise in ("nan", "inf", "-0.1", "1.1"):
            with self.subTest(noise=noise):
                result = subprocess.run(
                    [sys.executable, str(evaluator), SANDBOX_BINARY,
                     "--output", "/nonexistent/cupcup-invalid-noise.json",
                     "--observation-noise=" + noise],
                    capture_output=True, text=True, check=False)
                self.assertEqual(result.returncode, 2)
                self.assertIn("observation noise must be finite", result.stderr)

    def run_simulation(
        self, *args: str
    ) -> tuple[subprocess.CompletedProcess[str], dict]:
        result = subprocess.run(
            [SANDBOX_BINARY, *args], capture_output=True, text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return result, json.loads(result.stdout)

    def test_scenario_is_reproducible_and_explains_strategy(self) -> None:
        args = ("--duration", "3", "--seed", "29", "--opponent", "shared")
        first_result, first = self.run_simulation(*args)
        second_result, _ = self.run_simulation(*args)
        self.assertEqual(first_result.stdout, second_result.stdout)
        self.assertEqual(first["model"], "bounded-2d-v3")
        self.assertEqual(first["robot_contact_revision"], 2)
        self.assertEqual(first["ball_contact_revision"], 2)
        self.assertEqual(first["observation_execution_revision"], 2)
        self.assertEqual(first["horizontal_vision_revision"], 2)
        self.assertEqual(first["parameters"]["vision_model"], "pan")
        self.assertEqual(first["cupcup_color"], "red")
        self.assertGreater(len(first["frames"]), 5)
        for frame in first["frames"]:
            self.assertEqual(len(frame["red"]), 2)
            self.assertEqual(len(frame["blue"]), 2)
            for robot in frame["red"] + frame["blue"]:
                self.assertLessEqual(abs(robot["head_pan_deg"]), 60)
                if robot["sees_ball"]:
                    self.assertEqual(len(robot["observed_ball_relative"]), 2)
                else:
                    self.assertIsNone(robot["observed_ball_relative"])
                self.assertIn(
                    robot["action"],
                    {"CHASE", "SUPPORT", "DEFEND", "CLEAR", "HOLD"},
                )
                self.assertEqual(len(robot["target"]), 2)
                self.assertLessEqual(robot["speed"], 0.40001)
                self.assertLessEqual(abs(robot["x"]), 4.2501)
                self.assertLessEqual(abs(robot["z"]), 2.7501)
                if robot["sees_ball"]:
                    self.assertEqual(len(robot["observed_ball"]), 2)
                self.assertIn("mapped_ball", robot)
                self.assertIn("mapped_self", robot)
        kickoff = first["frames"][0]
        self.assertGreater(kickoff["red"][0]["yaw"], 90.0)
        self.assertLess(abs(kickoff["blue"][0]["yaw"]), 30.0)

    def test_noise_changes_belief_without_changing_replay_seed(self) -> None:
        _, clean = self.run_simulation(
            "--duration", "1", "--seed", "3", "--noise", "0",
            "--opponent", "press"
        )
        _, noisy = self.run_simulation(
            "--duration", "1", "--seed", "3", "--noise", "0.4",
            "--opponent", "press"
        )
        self.assertNotEqual(
            clean["frames"][1]["red"][0]["observed_ball"],
            noisy["frames"][1]["red"][0]["observed_ball"],
        )
        _, blue = self.run_simulation(
            "--duration", "1", "--seed", "3", "--cupcup-color", "blue"
        )
        self.assertEqual(blue["cupcup_color"], "blue")

    def test_invalid_parameters_fail_without_emitting_scene_data(self) -> None:
        result = subprocess.run(
            [SANDBOX_BINARY, "--duration", "901"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")

    def test_goal_and_sideline_restarts(self) -> None:
        _, match = self.run_simulation(
            "--duration", "5", "--scenario", "shot", "--mode", "oracle"
        )
        final = match["frames"][-1]
        self.assertEqual(final["red_score"], 1)
        self.assertEqual(match["metrics"]["restarts"], 1)
        _, side = self.run_simulation("--duration", "1", "--scenario", "sideline")
        self.assertEqual(side["frames"][-1]["phase"], "PAUSE")
        self.assertEqual(side["frames"][-1]["event"], "sideline")
        self.assertEqual(side["frames"][-1]["ball_velocity"], [0, 0])

    def test_color_swap_rotates_the_same_experiment(self) -> None:
        for mode in ("oracle", "realistic"):
            args = ("--duration", "60", "--seed", "41", "--opponent", "press", "--mode", mode)
            _, red = self.run_simulation(*args, "--cupcup-color", "red")
            _, blue = self.run_simulation(*args, "--cupcup-color", "blue")
            self.assertEqual(red["metrics"]["red_kicks"], blue["metrics"]["blue_kicks"])
            for a, b in zip(red["frames"], blue["frames"]):
                self.assertEqual(a["red_score"], b["blue_score"])
                for team, other in (("red", "blue"), ("blue", "red")):
                    for p, q in zip(a[team], b[other]):
                        self.assertAlmostEqual(p["x"], -q["x"], places=5)
                        self.assertAlmostEqual(p["z"], -q["z"], places=5)

    def test_measured_slow_turn_long_color_pair(self) -> None:
        args = ("--duration", "180", "--seed", "101", "--mode", "realistic",
                "--speed", "0.12113567", "--turn", "15.76969",
                "--opponent-speed", "0.07570979", "--opponent-turn", "15.76969",
                "--opponent-setup", "1.8", "--opponent", "press",
                "--kick-speed", "1.16982", "--ball-decay", "0.696006911")
        _, red = self.run_simulation(*args, "--cupcup-color", "red")
        _, blue = self.run_simulation(*args, "--cupcup-color", "blue")
        for name in ("kicks", "penalties", "mean_ball_attack_m", "robot_blocked_s"):
            self.assertAlmostEqual(red["metrics"]["red_" + name],
                                   blue["metrics"]["blue_" + name], places=5)
        for a, b in zip(red["frames"], blue["frames"]):
            for p, q in zip(a["red"], b["blue"]):
                self.assertAlmostEqual(p["x"], -q["x"], places=5)
                self.assertAlmostEqual(p["z"], -q["z"], places=5)

    def test_capabilities_delay_and_missing_information(self) -> None:
        _, slow = self.run_simulation("--duration", "5", "--speed", "0.15",
                                      "--opponent-speed", "0.7", "--delay", "0.4")
        self.assertEqual(slow["parameters"]["opponent"]["speed_mps"], 0.7)
        for frame in slow["frames"]:
            for p in frame["red"]:
                self.assertLessEqual(p["speed"], 0.15001)
        self.assertGreaterEqual(slow["frames"][-1]["red"][0]["observation_age_s"], 0.39)
        _, blind = self.run_simulation("--duration", "10", "--dropout", "1")
        self.assertEqual(blind["metrics"]["red_kicks"], 0)
        self.assertEqual(blind["metrics"]["blue_kicks"], 0)


if __name__ == "__main__":
    unittest.main()
