#!/usr/bin/env python3
"""Synthetic contact-scoring safeguards, not evidence of robot performance."""
import unittest
import tempfile
from pathlib import Path

from analyze_kick_contacts import (
    ROBOTS, analyze, preparation_coverage, read_trace, request_scene, segment_geometry)


def fixture(contact=True):
    rows = []
    for i in range(301):
        t = i * .02
        x = min(.4, max(0, (t - 1.5) * .2)) if contact else 0
        row = {"time": str(t), "state": "2", "ball_x": str(x), "ball_z": "0",
               "ball_vx": ".2" if 1.5 < t < 3.5 and contact else "0", "ball_vz": "0"}
        for r in ROBOTS:
            row.update({f"{r}_true_x": "-.2", f"{r}_true_z": "0", f"{r}_yaw": "0",
                        f"{r}_kick_seq": "1" if r == "blue_1" and t >= 1 else "0",
                        f"{r}_kick_time": "1" if r == "blue_1" else "-1",
                        f"{r}_kick_name": "left_kick",
                        f"{r}_body_type": "2" if r == "blue_1" and 1 <= t < 1.5 else "1",
                        f"{r}_actname": "left_kick" if r == "blue_1" and 1 <= t < 1.5 else "",
                        f"{r}_ball_contact_count": "1" if
                        r == "blue_1" and 1.5 <= t <= 1.52 and contact else "0"})
        rows.append(row)
    return rows


class KickContactsTest(unittest.TestCase):
    def test_preparation_without_requests_deduplicates_and_preserves_unknown(self):
        rows = fixture(False)
        for row in rows:
            row["red_1_talk_seq"] = "1"
            row["red_1_talk"] = "cupcup|state=ALIGN"
        coverage = preparation_coverage(rows, "red_1")
        self.assertEqual(coverage["align_messages"], 1)
        self.assertIsNone(coverage["max_published_preparation_frames"])
        rows[1].update(red_1_talk_seq="2", red_1_talk=(
            "cupcup|state=SETTLE|prep_frames=3|lgdx=.3|lgdz=.4|lg_age=.1"))
        rows[2].update(red_1_talk_seq="3", red_1_talk=(
            "cupcup|state=ALIGN|prep_frames=0|lgdx=.3|lgdz=.4|lg_age=.3"))
        coverage = preparation_coverage(rows, "red_1")
        self.assertEqual(coverage["settle_messages"], 1)
        self.assertEqual(coverage["ground_messages"], 1)
        self.assertEqual(coverage["max_published_preparation_frames"], 3)
        self.assertEqual(coverage["published_ground_range_m"]["mean"], .5)
        self.assertEqual(preparation_coverage(fixture(), "red_1")["align_messages"], 0)

    def test_request_scene_keeps_truth_separate_and_missing_inputs_unknown(self):
        row = fixture()[50]
        scene = request_scene(row, "blue_1")
        self.assertIsNone(scene["published_target"])
        self.assertIsNone(scene["published_peer_obstacles"])
        self.assertEqual(scene["published_local_tracks"], [])
        self.assertEqual(len(scene["truth_opponents"]), 2)
        row["blue_1_talk"] = ("cupcup|kx=1|kz=0|bx=0|bz=0|peer_obstacles=1|"
                              "r0x=0.5|r0z=0.1|r0age=0.1|r0team=unknown|"
                              "r0source=number_square|r0conf=0.25|r0unc=1|"
                              "r1x=0|r1z=0|r1age=0.7|"
                              "pobs0x=0.8|pobs0z=0|pobs0age=0.2|pobs0team=red")
        scene = request_scene(row, "blue_1")
        self.assertEqual(scene["published_peer_obstacles"], 1)
        self.assertEqual(len(scene["published_local_tracks"]), 1)
        self.assertEqual(len(scene["published_peer_tracks"]), 1)
        self.assertEqual(scene["published_peer_tracks"][0]["source"],
                         "teammate_direct_number_square")
        track = scene["published_local_tracks"][0]
        self.assertEqual(track["team"], "unknown")  # truth does not assign identity
        self.assertAlmostEqual(track["published_ball_to_target"]["centre_distance_m"], .1)
        self.assertEqual(track["published_ball_to_target"]["projection_fraction"], .5)
        self.assertIsNone(segment_geometry((1, 0), (0, 0), (0, 0)))
        self.assertEqual(segment_geometry((-1, 0), (0, 0), (1, 0))["projection_fraction"], -1)

    def test_preparation_time_is_separate_from_command_latency_and_resets_on_align(self):
        rows = fixture()
        for row in rows:
            t = float(row["time"])
            row["blue_1_talk"] = "cupcup|state=" + (
                "ALIGN" if t < .2 else "SETTLE" if t < 1 else "KICK")
        e = analyze(rows)["events"][0]
        self.assertAlmostEqual(e["observed_settle_to_request_s"], .8)
        self.assertAlmostEqual(e["observed_settle_to_contact_s"], 1.3)
        rows[40]["blue_1_talk"] = "cupcup|state=ALIGN"
        e = analyze(rows)["events"][0]
        self.assertAlmostEqual(e["observed_settle_to_request_s"], .18)
        self.assertIsNone(analyze(fixture())["events"][0]["observed_settle_to_contact_s"])

    def test_only_partial_trailing_record_can_be_discarded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            path.write_text("time,state,ball_x\n1,2,0\n2,2,0\n3,", encoding="utf-8")
            rows, dropped = read_trace(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(dropped, 1)
            path.write_text("time,state,ball_x\n1,\n2,2,0\n", encoding="utf-8")
            rows, dropped = read_trace(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(dropped, 0)
            with self.assertRaises(ValueError):
                analyze(rows)

    def test_contact_motion_is_not_just_a_request(self):
        report = analyze(fixture())
        e = report["events"][0]
        self.assertAlmostEqual(e["first_contact_delay_s"], .5)
        self.assertAlmostEqual(e["command_pulse_sim_s"], .5)
        self.assertTrue(e["forward_contact_motion_observed"])
        self.assertAlmostEqual(e["initial_ball_forward_m"], .2)
        self.assertEqual(report["robots"]["blue_1"]["observed_requests"], 1)
        self.assertFalse(analyze(fixture(False))["events"][0]["forward_contact_motion_observed"])

    def test_motion_without_contact_is_not_scored(self):
        rows = fixture()
        for row in rows:
            row["blue_1_ball_contact_count"] = "0"
        self.assertFalse(analyze(rows)["events"][0]["forward_contact_motion_observed"])

    def test_phase_reset_and_trace_end_are_censored(self):
        for reason in ("phase-change", "ball-reset-or-unmodelled-jump", "trace-ended"):
            rows = fixture()
            if reason == "phase-change":
                rows[100]["state"] = "3"
            elif reason == "ball-reset-or-unmodelled-jump":
                rows[100]["ball_x"] = "3"
            else:
                rows = rows[:100]
            e = analyze(rows)["events"][0]
            self.assertEqual(e["censored"], reason)
            self.assertFalse(e["forward_contact_motion_observed"])

    def test_other_robot_touch_or_initial_motion_prevents_clean_label(self):
        rows = fixture()
        rows[76]["red_1_ball_contact_count"] = "1"
        self.assertFalse(analyze(rows)["events"][0]["forward_contact_motion_observed"])
        rows = fixture()
        rows[50]["ball_vx"] = ".1"
        self.assertFalse(analyze(rows)["events"][0]["forward_contact_motion_observed"])

    def test_reverse_time_missing_data_and_nan_rejected(self):
        for rows in ([], list(reversed(fixture())), [{"time": "nan"}]):
            with self.assertRaises(ValueError):
                analyze(rows)

    def test_gap_and_overlapping_requests_censor_old_window(self):
        rows = fixture()
        e = analyze(rows[:80] + rows[100:])["events"][0]
        self.assertEqual(e["censored"], "sampling-gap")
        rows = fixture()
        for row in rows[100:]:
            row["blue_1_kick_seq"] = "2"
            row["blue_1_kick_time"] = "2"
        self.assertEqual(analyze(rows)["events"][0]["censored"], "next-request")


if __name__ == "__main__":
    unittest.main()
