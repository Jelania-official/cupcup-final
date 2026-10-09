#!/usr/bin/env python3
"""Synthetic checks for calibration math and safeguards, not robot performance."""

import math
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from analyze_measurement import (
    distribution, fit_rate, prediction, rolling_series,
    sandbox_parameters, score_blocking, score_kick, score_motion, score_timing, target_evidence,
)


class MeasurementAnalysisTest(unittest.TestCase):
    def test_service_discovery_profile_preserves_async_dynamic_defaults(self):
        path = Path(__file__).parent / "measurement/fastdds-udp-service.xml"
        root = ET.parse(path).getroot()
        ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        service = root.find("d:data_writer[@profile_name='service']", ns)
        default = root.find("d:data_writer[@is_default_profile='true']", ns)
        for writer in (service, default):
            self.assertIsNotNone(writer)
            self.assertEqual(writer.findtext("d:historyMemoryPolicy", namespaces=ns), "DYNAMIC")
            self.assertEqual(writer.findtext("d:qos/d:publishMode/d:kind", namespaces=ns),
                             "ASYNCHRONOUS")
        self.assertEqual(service.findtext("d:qos/d:reliability/d:kind", namespaces=ns), "RELIABLE")
        self.assertEqual(service.findtext(
            "d:qos/d:reliability/d:max_blocking_time/d:sec", namespaces=ns), "10")

    def test_target_evidence_excludes_shared_ready_and_requires_distinct_frames(self):
        reference = [{"action": action, "index": str(i), "knee": str(value)}
                     for action, values in (("ready", [0]), ("left_kick", [0, -1, -2]),
                                            ("right_kick", [0, 1, 2, 3, 4]))
                     for i, value in enumerate(values)]
        targets = [{"time": str(i * .02), "knee": str(value)}
                   for i, value in enumerate([0, 0, 1, 2, 3, 4])]
        result = target_evidence(reference, targets, "right_kick", 0, .1)
        self.assertTrue(result["action_target_sequence_observed"])
        self.assertEqual(result["distinct_action_target_frames"], 4)
        self.assertAlmostEqual(result["first_action_target_delay_s"], .04)
        self.assertEqual(result["target_sequence_episodes"], 1)
        result = target_evidence(reference, targets, "left_kick", 0, .1)
        self.assertFalse(result["action_target_sequence_observed"])
        missing = target_evidence(reference, targets[:2], "left_kick", 0, 1)
        self.assertIsNone(missing["action_target_sequence_observed"])
        self.assertFalse(missing["target_history_complete"])
        held = [{"time": str(i * .02), "knee": "1"} for i in range(100)]
        self.assertFalse(target_evidence(reference, held, "right_kick", 0, 2)[
            "action_target_sequence_observed"])

    def timing_fixture(self, moving=True, contact=True):
        summary = {"final_forward": "1" if moving else "0", "final_left": "0",
                   "pre_drift": "0", "max_ball_displacement": "1" if moving else "0",
                   "max_ball_speed": "1" if moving else "0", "fall": "0",
                   "contact_frames": "1" if contact else "0", "pulse": ".12"}
        rows = [{"phase": "ACT" if i < 75 else "MEASURE", "time": str(i * .02),
                 "ball_x": str(max(0, i - 50) * .004 if moving else 0), "ball_z": "0",
                 "ball_vx": "0", "ball_vz": "0", "vx": "0", "vz": "0",
                 "robot_y": ".34", "body_type": "2" if i < 6 else "1",
                 "ball_contact_count": "1" if contact and i == 50 else "0"}
                for i in range(326)]
        return summary, rows

    def test_blocking_scores_contacts_geometry_and_opponent_instability(self):
        summary, rows = self.timing_fixture()
        summary.update({"blocker_forward": ".25", "blocker_left": ".18",
                        "opponent_contact_frames": "1", "opponent_fall": "0",
                        "act_yaw_deg": "180"})
        rows = [r | {"opponent_contact_count": "1" if i == 80 else "0",
                     "opponent_x": "-.25", "opponent_z": ".18",
                     "opponent_y": ".34", "opponent_fall": "0"}
                for i, r in enumerate(rows)]
        result = score_blocking(summary, rows)
        self.assertTrue(result["sampled_contact"])
        self.assertTrue(result["sampled_opponent_contact"])
        self.assertFalse(result["opponent_fall"])
        self.assertAlmostEqual(result["actual_blocker_forward_m"], .25)
        self.assertAlmostEqual(result["actual_blocker_left_m"], .18)
        self.assertEqual(result["opponent_root_drift_m"], 0)
        rows[100]["opponent_y"] = ".20"
        self.assertTrue(score_blocking(summary, rows)["opponent_fall"])
        with self.assertRaisesRegex(ValueError, "opponent contact summary"):
            score_blocking(summary | {"opponent_contact_frames": "2"}, rows)

    def test_timing_distinguishes_physical_contact_and_movement(self):
        for moving, contact in ((True, True), (True, False), (False, True)):
            summary, rows = self.timing_fixture(moving, contact)
            result = score_timing(summary, rows)
            self.assertEqual(result["clean_forward_motion"], moving and contact)
            self.assertEqual(result["sampled_contact"], contact)
            self.assertAlmostEqual(result["published_pulse_sim_s"], .12)

    def test_timing_rejects_incomplete_or_mismatched_contact_trace(self):
        summary, rows = self.timing_fixture()
        with self.assertRaisesRegex(ValueError, "complete"):
            score_timing(summary, rows[:100])
        with self.assertRaisesRegex(ValueError, "contact summary"):
            score_timing(summary | {"contact_frames": "2"}, rows)
        with self.assertRaisesRegex(ValueError, "missing samples"):
            score_timing(summary, rows[:50] + rows[60:])

    def test_timing_rejects_wrong_task_type_and_unbounded_pulse(self):
        summary, rows = self.timing_fixture()
        with self.assertRaisesRegex(ValueError, "no ACT"):
            score_timing(summary, [row | {"body_type": "1"} for row in rows])
        with self.assertRaisesRegex(ValueError, "unbounded"):
            score_timing(summary, [row | {"body_type": "2"} for row in rows])
        with self.assertRaisesRegex(ValueError, "differs"):
            score_timing(summary | {"pulse": ".8"}, rows)

    def test_distribution_interpolates_small_sample(self):
        self.assertEqual(distribution([]), {"n": 0})
        self.assertEqual(distribution([1, 3])["median"], 2)
        self.assertAlmostEqual(distribution([1, 3])["p95"], 2.9)

    def test_exponential_decay_recovered_at_two_speeds(self):
        trials = [[(i * .02, *prediction("exponential", .7, speed, i * .02))
                   for i in range(241)] for speed in (.9, 1.5)]
        self.assertAlmostEqual(fit_rate("exponential", trials), .7, places=5)

    def test_constant_deceleration_recovered_and_stops(self):
        samples = [(i * .02, *prediction("constant_deceleration", .3, .9, i * .02))
                   for i in range(241)]
        self.assertAlmostEqual(fit_rate("constant_deceleration", [samples]), .3, places=5)
        speed, position = prediction("constant_deceleration", .3, .9, 100)
        self.assertEqual(speed, 0)
        self.assertAlmostEqual(position, 1.35)

    def test_walking_uses_net_displacement_not_body_sway_peak(self):
        rows = [{"phase": "ACT", "phase_time": str(i), "time": str(i),
                 "robot_x": str(-.12 * i), "robot_z": str(.03 * math.sin(i)),
                 "robot_y": ".34", "yaw_deg": "180", "fall": "0"} for i in range(9)]
        result = score_motion(rows, -1)
        self.assertAlmostEqual(result["forward_speed_m_s"], .12)
        self.assertFalse(result["fall"])

    def test_yaw_crossing_not_a_full_rotation(self):
        rows = [{"phase": "ACT", "phase_time": str(i), "time": str(i),
                 "robot_x": "0", "robot_z": "0", "robot_y": ".34",
                 "yaw_deg": str((170 + 5 * i + 180) % 360 - 180), "fall": "0"}
                for i in range(9)]
        self.assertAlmostEqual(score_motion(rows, 1)["net_turn_deg_s"], 5)

    def test_early_push_fall_and_backwards_kick_do_not_pass(self):
        summary = {"final_forward": "1", "final_left": "0", "pre_drift": "0",
                   "max_ball_displacement": "1", "max_ball_speed": "1", "fall": "0"}
        self.assertTrue(score_kick(summary, [])["strict_success"])
        for overrides in ({"pre_drift": ".04"}, {"fall": "1"}, {"final_forward": "-1"}):
            self.assertFalse(score_kick(summary | overrides, [])["strict_success"])

    def test_kick_motion_latency_uses_continuous_time_across_phases(self):
        summary = {"final_forward": "1", "final_left": "0", "pre_drift": "0",
                   "max_ball_displacement": "1", "max_ball_speed": "1", "fall": "0"}
        rows = [{"phase": phase, "time": str(t), "ball_x": str(x), "ball_z": "0",
                 "robot_y": ".34"}
                for phase, t, x in [("ACT", 10, 0), ("MEASURE", 11.5, .1)]]
        self.assertEqual(score_kick(summary, rows)["command_to_ball_movement_s"], 1.5)

    def test_roll_rejects_large_lateral_motion(self):
        rows = [{"phase": "ACT", "time": str(.02 * i), "ball_x": str(.01 * i),
                 "ball_vx": ".5", "ball_vz": ".2"} for i in range(80)]
        with self.assertRaisesRegex(ValueError, "contamination"):
            rolling_series(rows, 1)

    def test_motion_does_not_score_short_or_nonfinite_samples(self):
        with self.assertRaises(ValueError):
            score_motion([], 1)
        rows = [{"phase": "ACT", "phase_time": "nan"}]
        with self.assertRaises(ValueError):
            score_motion(rows, 1)

    def test_nominal_parameters_keep_unsupported_models_explicit(self):
        report = {"schema": "cupcup-controlled-calibration-v1", "rolling": {
            "preferred_model_on_heldout": "constant_deceleration"}}
        with self.assertRaisesRegex(ValueError, "exponential"):
            sandbox_parameters(report)

    def test_saved_report_has_reproducible_partial_parameters(self):
        report = json.loads((Path(__file__).parent / "data/measurement-20261007.json").read_text())
        params = sandbox_parameters(report)
        self.assertEqual(params, report["nominal_sandbox_parameters"])
        self.assertLess(params["speed"], .15)
        self.assertGreater(params["speed"], .1)
        self.assertEqual(report["rolling"]["training_trials"], 4)
        self.assertEqual(report["rolling"]["heldout_trials"], 8)
        self.assertEqual(report["kicks"]["left_kick:0.18:0.08"]["strict_successes"], 6)
        self.assertEqual(report["kicks"]["right_kick:0.18:-0.04"]["strict_successes"], 0)
        self.assertEqual(report["kicks"]["right_kick:0.22:-0.04"]["strict_successes"], 6)


if __name__ == "__main__":
    unittest.main()
