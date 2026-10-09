#!/usr/bin/env python3
"""Unit checks for offline Webots trace evaluation."""

import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_world_trace import analyze_rows, parse_talk  # noqa: E402


def sample_row(sequence: int, talk: str,
               teammate_talk: str = "") -> dict[str, str]:
    row = {"ball_x": "1", "ball_z": "0", "ball_vx": "0.5", "ball_vz": "0"}
    for name in ("red_1", "red_2", "blue_1", "blue_2"):
        row.update({
            f"{name}_true_x": "0",
            f"{name}_true_z": "0",
            f"{name}_obs_x": "0.3",
            f"{name}_obs_z": "0.4",
            f"{name}_vx": "0.3",
            f"{name}_vz": "0.4",
            f"{name}_omega_y": "-1.0",
            f"{name}_talk_seq": str(sequence),
            f"{name}_talk": (
                talk if name == "red_1" else
                teammate_talk if name == "red_2" else ""
            ),
        })
    return row


class WorldTraceTest(unittest.TestCase):
    def test_arrived_observer_uses_only_fresh_nonowner_estimates_and_unique_messages(self):
        base = ("cupcup|tac=DEFEND|healthy=1|px=1|pz=0|tx=1.01|tz=0|pyaw=0|"
                "pose_age=.1|bx=0|bz=0|bmap_age=.1|hy=55|robots=1|"
                "r0source=number_square|r0x=0|r0z=0|r0age=.1|r0conf=.25")
        wires = (base, base, base + "|tac=CHASE", base + "|bmap_age=.7",
                 base + "|tx=2", base + "|pose_age=-.1", base + "|healthy=0",
                 base + "|pyaw=180|hy=0")
        rows = [dict(sample_row(i if i else 1, wire), state="2")
                for i, wire in enumerate(wires)]
        robot = analyze_rows(rows)["robots"]["red_1"]
        counts = robot["belief_coverage"]
        self.assertEqual(counts["arrived_nonowner_messages"], 2)
        self.assertEqual(counts["arrived_nonowner_number_square_messages"], 2)
        self.assertEqual(counts["arrived_nonowner_head_saturated_messages"], 1)
        angles = robot["arrived_nonowner_mapped_body_ball_angle_deg"]
        self.assertEqual(angles["n"], 2)
        self.assertEqual(angles["mean"], 90)

    def test_heading_diagnostics_wrap_deduplicate_and_keep_unknown_unknown(self):
        base = "cupcup|bx=0|bz=0|kx=-1|kz=0|bmap_age=.1|aim_yaw=-179|aim_offset=-20"
        rows = [dict(sample_row(1, base), state="2"),
                dict(sample_row(1, base), state="2"),
                dict(sample_row(2, base + "|aim_yaw=nan"), state="2"),
                dict(sample_row(3, base + "|bmap_age=-.1"), state="2"),
                dict(sample_row(4, base + "|bmap_age=.7"), state="2"),
                dict(sample_row(5, base + "|kx=0"), state="2"),
                dict(sample_row(6, "cupcup|bx=0|bz=0|kx=-1|kz=0|bmap_age=.1"), state="2"),
                dict(sample_row(7, base), state="1")]
        red = analyze_rows(rows)["robots"]["red_1"]
        self.assertEqual(red["controller_vs_policy_heading_abs_deg"]["n"], 1)
        self.assertAlmostEqual(red["controller_vs_policy_heading_abs_deg"]["mean"], 1)
        self.assertEqual(red["legacy_heading_offset_abs_deg"]["mean"], 20)
        self.assertEqual(analyze_rows([sample_row(1, "")])["robots"]["red_1"][
            "controller_vs_policy_heading_abs_deg"]["n"], 0)

    def test_coverage_keeps_missing_fields_and_deduplicates_play_messages(self):
        base = "cupcup|ball=1|age=.1|fa=.1|robots=1"
        marker = "|r0source=number_square|r0x=1|r0z=0|r0age=.1|r0conf=.25"

        def row(sequence, talk, state=2):
            return sample_row(sequence, talk) | {"state": str(state)}
        report = analyze_rows([
            row(1, base + marker, 1), row(2, base + marker), row(2, base + marker),
            row(3, base + marker + "|r0age=-.1"), row(4, base + marker + "|r0x=nan"),
            row(5, "cupcup|ball=0"), row(6, "cupcup|robots=0|ball=1|age=-.1|fa=.1"),
            row(7, "cupcup|robots=1"),
        ])
        counts = report["robots"]["red_1"]["belief_coverage"]
        self.assertEqual(counts["play_talk_messages"], 6)
        self.assertEqual(counts["fresh_ball_messages"], 3)
        self.assertEqual(counts["number_square_messages"], 1)
        self.assertEqual(counts["number_square_candidates"], 1)
        self.assertEqual(counts["robot_fields_missing_messages"], 2)

    def test_runtime_local_xy_is_not_reconstructed_from_range_or_global_map(self) -> None:
        base = "cupcup|ball=1|age=0.05|fa=0.05|bdist=1|ball_range_source=ground_ray"
        point = "|lgdx=0.8|lgdz=0.3|lg_age=0.05"
        report = analyze_rows([
            sample_row(1, base + point), sample_row(1, base + point),
            sample_row(2, base),
            sample_row(3, base + point + "|lg_age=-.1"),
            sample_row(4, base + point + "|lg_age=.3"),
            sample_row(5, base + point + "|lgdx=nan"),
            sample_row(6, base + point + "|ball_range_source=radius"),
        ])
        local = report["robots"]["red_1"]["local_ground_xy_error_by_true_range_m"]["1-2m"]
        self.assertEqual(local["n"], 1)
        self.assertAlmostEqual(local["mean"], (0.2 ** 2 + 0.3 ** 2) ** .5)

    def test_negative_map_pose_and_pair_ages_are_not_fresh(self) -> None:
        bad = "cupcup|ball=1|bmap_age=-.1|bx=1|bz=0|pose_age=-.1|px=0|pz=0"
        good = "cupcup|ball=1|bmap_age=.05|bx=1|bz=0"
        report = analyze_rows([sample_row(1, bad, good)])
        red = report["robots"]["red_1"]
        self.assertEqual(red["talk_pose_error_m"]["n"], 0)
        self.assertEqual(red["talk_ball_error_m"]["n"], 0)
        self.assertEqual(red["talk_ball_error_by_true_range_m"]["1-2m"]["n"], 0)
        self.assertEqual(report["paired_ball_fusion"]["red"]["midpoint_fused_error_m"]["n"], 0)

    def test_play_occupancy_distinguishes_document_from_released_tolerance(self) -> None:
        row = sample_row(1, "")
        row.update({"state": "2", "red_1_true_x": "2.6", "red_1_true_z": "2.2",
                    "blue_1_true_x": "-2.6", "blue_1_true_z": "-2.2",
                    "red_2_true_x": "-0.1", "blue_2_true_x": "0.1"})
        ready = dict(row, state="1")
        report = analyze_rows([row, ready])
        for name in ("red_1", "blue_1", "red_2", "blue_2"):
            occupancy = report["robots"][name]["role_region_occupancy"]
            self.assertEqual(occupancy["play_position_samples"], 1)
            self.assertEqual(occupancy["document_role_region_samples"], 1)
            self.assertEqual(occupancy["released_stop_region_samples"], 0)
            self.assertAlmostEqual(occupancy["max_document_role_penetration_m"], 0.1)
        row.update({"red_1_true_x": "2.8", "red_1_true_z": "1.9",
                    "red_2_true_x": "-0.3"})
        report = analyze_rows([row])
        for name in ("red_1", "red_2"):
            self.assertEqual(report["robots"][name]["role_region_occupancy"][
                "released_stop_region_samples"], 1)

    def test_range_source_split_retains_unknown_and_rejects_negative_ages(self) -> None:
        base = "cupcup|ball=1|age=0.05|fa=0.05|bdist=0.8"
        report = analyze_rows([
            sample_row(1, base + "|ball_range_source=ground_ray"),
            sample_row(1, base + "|ball_range_source=ground_ray"),
            sample_row(2, base + "|ball_range_source=radius"),
            sample_row(3, base),
            sample_row(4, base + "|ball_range_source=invalid"),
            sample_row(5, base + "|age=-0.1|ball_range_source=ground_ray"),
            sample_row(6, base + "|fa=-0.1|ball_range_source=ground_ray"),
        ])
        red = report["robots"]["red_1"]
        split = red["camera_range_abs_error_by_source_and_true_range_m"]
        self.assertEqual(split["ground_ray"]["1-2m"]["n"], 1)
        self.assertEqual(split["radius"]["1-2m"]["n"], 1)
        self.assertEqual(split["unknown"]["1-2m"]["n"], 2)
        self.assertEqual(red["camera_range_abs_error_by_true_range_m"]["1-2m"]["n"], 4)

    def test_only_fresh_new_talk_messages_are_scored(self) -> None:
        fresh = (
            "cupcup|ball=1|age=0.05|bmap_age=0.05|bx=1.3|bz=0|"
            "px=0.3|pz=0.4|pose_age=0.05"
        )
        stale = (
            "cupcup|ball=1|age=0.01|bmap_age=0.5|bx=9|bz=9|"
            "px=9|pz=9|pose_age=0.5"
        )
        report = analyze_rows([
            sample_row(1, fresh),
            sample_row(2, stale),
        ])
        red = report["robots"]["red_1"]
        self.assertEqual(red["reported_location_error_m"]["n"], 2)
        self.assertAlmostEqual(red["reported_location_error_m"]["median"], 0.5)
        self.assertEqual(red["talk_pose_error_m"]["n"], 1)
        self.assertAlmostEqual(red["talk_pose_error_m"]["mean"], 0.5)
        self.assertEqual(red["talk_ball_error_m"]["n"], 1)
        self.assertAlmostEqual(red["talk_ball_error_m"]["mean"], 0.3)

    def test_non_cupcup_messages_are_not_treated_as_estimates(self) -> None:
        self.assertEqual(parse_talk("unirobot|ball=1|bx=1|bz=2"), {})

    def test_partial_final_sample_is_discarded(self) -> None:
        row = sample_row(2, "")
        row["blue_2_talk_seq"] = None
        report = analyze_rows([sample_row(1, ""), row])
        self.assertEqual(report["complete_rows"], 1)
        self.assertEqual(report["incomplete_rows"], 1)
        self.assertEqual(
            report["robots"]["red_1"]["reported_location_error_m"]["n"],
            1,
        )

    def test_pair_estimates_use_their_midpoint(self) -> None:
        first = "cupcup|ball=1|bmap_age=0.02|bx=0.8|bz=0"
        second = "cupcup|ball=1|bmap_age=0.04|bx=1.2|bz=0"
        report = analyze_rows([sample_row(1, first, second)])
        paired = report["paired_ball_fusion"]["red"]
        self.assertEqual(paired["individual_mean_error_m"]["n"], 1)
        self.assertAlmostEqual(paired["individual_mean_error_m"]["mean"], 0.2)
        self.assertAlmostEqual(
            paired["midpoint_fused_error_m"]["mean"], 0.0
        )

    def test_range_error_uses_fresh_unique_detector_samples(self) -> None:
        fresh = (
            "cupcup|ball=1|age=0.05|fa=0.05|bdist=0.8|"
            "iha=0.03|iia=0.02|bmap_age=0.05|bx=1.3|bz=0"
        )
        stale_frame = "cupcup|ball=1|age=0.05|fa=0.5|bdist=7.9"
        report = analyze_rows([
            sample_row(1, fresh),
            # Same Talk message sampled again by the supervisor.
            sample_row(1, fresh),
            sample_row(2, stale_frame),
        ])
        red = report["robots"]["red_1"]
        one_to_two = red["camera_range_abs_error_by_true_range_m"]["1-2m"]
        self.assertEqual(one_to_two["n"], 1)
        self.assertAlmostEqual(one_to_two["median"], 0.2)
        self.assertAlmostEqual(
            red["observation_age_s"]["iha"]["median"], 0.03
        )
        mapped = red["talk_ball_error_by_true_range_m"]["1-2m"]
        self.assertEqual(mapped["n"], 1)
        self.assertAlmostEqual(mapped["median"], 0.3)


if __name__ == "__main__":
    unittest.main()
