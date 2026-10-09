#!/usr/bin/env python3
"""Exercise label scoring and safeguards without asserting robot performance."""

import copy
import math
import unittest

from score_robot_labels import (
    evaluate, evaluate_number_landmarks, iou, match_boxes, validate_labels,
)
from score_robot_replay import (
    compare_bottom_range, compare_number_patch, ground_range_proxy, number_patch_proxy,
    score_static_view,
)


class RobotLabelsTest(unittest.TestCase):
    def fixture(self):
        labels = {"schema": "cupcup-visible-robot-labels-v1", "width": 100, "height": 100,
                  "frames": [{"image": "a.jpg", "split": "development", "robots": [
                      {"xyxy": [10, 10, 30, 50], "team": "red", "visibility": "full"},
                      {"xyxy": [60, 10, 80, 50], "team": "blue", "visibility": "occluded"}]},
                      {"image": "b.jpg", "split": "holdout", "robots": []}]}
        replay = [{"image": "/images/a.jpg", "robots": [
            {"x": .2, "y": .3, "width": .2, "height": .4, "team": "unknown"}]},
            {"image": "/images/b.jpg", "robots": [
                {"x": .8, "y": .8, "width": .1, "height": .1, "team": "red"}]}]
        return labels, replay

    def test_misses_false_positives_and_unknown_color_are_distinct(self):
        labels, replay = self.fixture()
        result = evaluate(labels, replay, .5)
        self.assertEqual((result["true_positives"], result["false_positives"],
                          result["false_negatives"]), (1, 1, 1))
        self.assertEqual(result["precision"], .5)
        self.assertEqual(result["recall"], .5)
        self.assertEqual(result["wrong_team"], 0)
        self.assertEqual(result["unclassified_known_team"], 1)
        self.assertEqual(result["correct_known_team_detection_recall"], 0)
        self.assertEqual(evaluate(labels, replay, .5, "holdout")["false_positives"], 1)

    def test_wrong_color_not_confused_with_unknown(self):
        labels, replay = self.fixture()
        replay[0]["robots"][0]["team"] = "blue"
        self.assertEqual(evaluate(labels, replay, .5)["wrong_team"], 1)

    def test_matching_is_one_to_one_not_greedy_and_bounded(self):
        labels = [[0, 0, 10, 10], [8, 0, 18, 10]]
        predictions = [[1, 0, 17, 10], [0, 0, 8, 10]]
        pairs = match_boxes(predictions, labels, .3)
        self.assertEqual(len(pairs), 2)
        self.assertEqual({(i, j) for i, j, _ in pairs}, {(0, 1), (1, 0)})
        self.assertEqual(len(match_boxes([labels[0]] * 2, labels[:1], .5)), 1)
        with self.assertRaises(ValueError):
            match_boxes(predictions * 3, labels, .3)
        for threshold in (0, 1.1, math.nan):
            with self.assertRaises(ValueError):
                match_boxes(predictions, labels, threshold)
        self.assertEqual(iou([0, 0, 10, 10], [10, 10, 20, 20]), 0)

    def test_missing_duplicate_or_corrupt_replay_fails(self):
        labels, replay = self.fixture()
        for invalid in (replay[:1], replay + replay[:1]):
            with self.assertRaises(ValueError):
                evaluate(labels, invalid, .5)
        replay[0]["robots"][0]["width"] = math.nan
        with self.assertRaises(ValueError):
            evaluate(labels, replay, .5)

    def test_invalid_labels_not_silently_dropped(self):
        labels, _ = self.fixture()
        for override in ({"team": "enemy"}, {"xyxy": [30, 10, 10, 50]},
                         {"xyxy": [0, 0, math.nan, 50]}):
            invalid = copy.deepcopy(labels)
            invalid["frames"][0]["robots"][0].update(override)
            with self.assertRaises(ValueError):
                validate_labels(invalid)
        labels["frames"] *= 2
        with self.assertRaises(ValueError):
            validate_labels(labels)

    def test_number_landmarks_do_not_claim_body_iou_or_hide_duplicates(self):
        labels, frames = self.fixture()
        patch = {"x": .2, "y": .3, "width": .05, "height": .05, "team": "blue"}
        frames[0]["number_patches"] = [patch, patch, dict(patch, x=.95, y=.95)]
        result = evaluate_number_landmarks(labels, frames)
        self.assertEqual((result["assigned"], result["duplicate"],
                          result["outside_labelled_robots"], result["wrong_team"]), (1, 1, 1, 1))
        labels["frames"][0]["robots"][1]["xyxy"] = [10, 10, 30, 50]
        self.assertEqual(evaluate_number_landmarks(labels, frames)["ambiguous"], 2)
        frames[0]["number_patches"][0] = dict(patch, x=math.nan)
        with self.assertRaises(ValueError):
            evaluate_number_landmarks(labels, frames)


class RobotGeometryTest(unittest.TestCase):
    def test_ground_proxy_rejects_missing_pose_horizon_and_outside_image(self):
        name = "red_1"
        row = {name + "_camera_" + key: value
               for key, value in zip("xyz", (0, 1, 0))}
        matrix = (1, 0, 0, 0, 0, 1, 0, -1, 0)
        row.update({name + "_camera_r" + str(i): value for i, value in enumerate(matrix)})
        row.update({name + "_true_x": 0, name + "_true_z": 0})
        expected = 1 / (.25 * .75 * 2 * math.tan(1.3613 / 2))
        self.assertAlmostEqual(ground_range_proxy(row, name, .5, .75), expected)
        self.assertIsNone(ground_range_proxy(row, name, .5, .5))
        self.assertIsNone(ground_range_proxy(row, name, .5, 1.1))
        self.assertIsNone(ground_range_proxy({}, name, .5, .75))
        row[name + "_camera_y"] = math.nan
        self.assertIsNone(ground_range_proxy(row, name, .5, .75))

    def test_bottom_comparison_uses_identical_subset_not_cherry_picked_denominators(self):
        matched = {"clipped": False, "ambiguous": False, "range_error": .2,
                   "bbox_bottom_range_error_m": .1}
        cases = [matched, dict(matched, clipped=True), dict(matched, ambiguous=True),
                 dict(matched, bbox_bottom_range_error_m=None)]
        result = compare_bottom_range(cases)
        self.assertEqual(result["matched_subset_n"], 1)
        self.assertEqual(result["height_error_m"]["n"], result["bbox_bottom_error_m"]["n"])
        self.assertFalse(result["deployable"])

    def test_number_patch_coordinate_convention_missing_truth_and_sparse_coverage(self):
        row = {"blue_1_camera_x": 0, "blue_1_camera_y": 1, "blue_1_camera_z": 0,
               "blue_1_true_x": 0, "blue_1_true_z": 0,
               "red_1_true_x": 2, "red_1_true_z": -0.2}
        matrix = (1, 0, 0, 0, 0, 1, 0, -1, 0)
        row.update({"blue_1_camera_r" + str(i): v for i, v in enumerate(matrix)})
        patch = {"translation": [-0.2, 0.5, 2]}
        proxy = number_patch_proxy(row, "blue_1", "red_1", patch)
        self.assertAlmostEqual(proxy["position_error_m"], 0)
        self.assertAlmostEqual(proxy["estimated_range_m"], math.hypot(2, .2))
        for invalid in (None, {"translation": [0, 0, 0]}, {"translation": [0, math.nan, 2]},
                        {"translation": [0, 2]}):
            self.assertIsNone(number_patch_proxy(row, "blue_1", "red_1", invalid))
        self.assertIsNone(number_patch_proxy({}, "blue_1", "red_1", patch))
        replay = {"frames": [{"robots": [{"number_patch": patch}, {"number_patch": None}]}]}
        matched = {"ambiguous": False, "range_error": .3, "number_patch_proxy": proxy}
        result = compare_number_patch([matched, dict(matched, ambiguous=True)], replay)
        self.assertEqual((result["detector_boxes"], result["accepted_patch_boxes"],
                          result["unambiguous_scored_patches"]), (2, 1, 1))
        self.assertEqual(result["height_error_m"]["n"], result["number_patch_range_error_m"]["n"])
        self.assertFalse(result["deployable"])

    def test_static_views_measure_original_sensor_geometry_and_reject_bad_samples(self):
        sample = {"observer": "blue_1", "target": "red_1", "observer_x": 0, "observer_z": 0,
                  "target_x": 2, "target_z": 0, "target_y": .365, "image_saved": "1", "fall": 0,
                  "image_age": 0, "imu_age": 0, "head_age": 0, "target_vx": 0, "target_vz": 0,
                  "image": "a.ppm", "name": "robot-front", "imu_yaw": 0, "head_yaw": 0}
        pose = {"origin": [0, .5, 0], "rotation": [1, 0, 0, 0, 0, 1, 0, -1, 0]}
        marker = {"x": .5, "y": .5, "width": .1, "height": .1,
                  "translation": [0, .02, 2], "team": "red"}
        frame = {"robots": [], "number_patches": [marker]}
        result = score_static_view(sample, frame, pose)
        self.assertTrue(result["sample_valid"])
        self.assertEqual(result["body_matches"], 0)
        self.assertEqual(result["marker_matches"], 1)
        self.assertAlmostEqual(result["marker_position_error_m"], 0)
        self.assertTrue(result["marker_team_correct"])
        moving_sample = sample | {"name": "robot-walk-fast", "observer_vx": ".12",
                                  "observer_omega_y": ".2", "phase_time": "2.5"}
        dynamic = score_static_view(moving_sample, frame, pose)
        self.assertTrue(dynamic["sample_valid"])
        self.assertEqual(dynamic["observer_motion"]["observer_vx"], .12)
        self.assertEqual(dynamic["observer_motion"]["phase_time"], 2.5)
        self.assertIsNone(result["observer_motion"]["observer_vx"])
        frame["number_patches"] *= 2
        self.assertIsNone(score_static_view(sample, frame, pose)["marker_position_error_m"])
        for override in ({"fall": 1}, {"image_age": .5}, {"head_age": -1},
                         {"target_vx": .1}, {"image_saved": "0"}):
            self.assertFalse(score_static_view(sample | override, frame, pose)["sample_valid"])


if __name__ == "__main__":
    unittest.main()
