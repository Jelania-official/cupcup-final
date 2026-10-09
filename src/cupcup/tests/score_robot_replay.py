#!/usr/bin/env python3
"""Evaluate recorded robot boxes against trace-only projected robot-centre proxies."""

import argparse
import bisect
import collections
import csv
import hashlib
import json
import math
import re
import statistics
import subprocess
from pathlib import Path

from analyze_world_trace import ROBOTS, distribution, number


def project_robot(row: dict, observer: str, target: str, height: float = .365) -> tuple | None:
    origin = [number(row, observer + "_camera_" + axis) for axis in "xyz"]
    matrix = [number(row, observer + "_camera_r" + str(i)) for i in range(9)]
    point = [number(row, target + "_true_x"), height, number(row, target + "_true_z")]
    if None in origin + matrix + point:
        return None
    delta = [point[i] - origin[i] for i in range(3)]
    local = [sum(matrix[k * 3 + i] * delta[k] for k in range(3)) for i in range(3)]
    if local[0] <= 0.1:
        return None
    scale = 2 * math.tan(1.3613 / 2)
    u, v = 0.5 - local[1] / (local[0] * scale), 0.5 - local[2] / (local[0] * scale * 0.75)
    return (u, v) if 0 <= u <= 1 and 0 <= v <= 1 else None


def ground_range_proxy(row: dict, observer: str, u: float, v: float) -> float | None:
    """Probe box-bottom IPM with evaluation-only camera truth, never online state."""
    origin = [number(row, observer + "_camera_" + axis) for axis in "xyz"]
    matrix = [number(row, observer + "_camera_r" + str(i)) for i in range(9)]
    self_position = [number(row, observer + "_true_x"), number(row, observer + "_true_z")]
    if None in origin + matrix + self_position or not 0 <= u <= 1 or not 0 <= v <= 1:
        return None
    scale = 2 * math.tan(1.3613 / 2)
    ray = [1, (.5 - u) * scale, (.5 - v) * scale * .75]
    direction = [sum(matrix[i * 3 + k] * ray[k] for k in range(3)) for i in range(3)]
    if direction[1] >= -.05:
        return None
    travel = -origin[1] / direction[1]
    if not 0 < travel <= 12:
        return None
    return math.hypot(origin[0] + travel * direction[0] - self_position[0],
                      origin[2] + travel * direction[2] - self_position[1])


def number_patch_proxy(row: dict, observer: str, target: str, patch: dict | None) -> dict | None:
    """Isolate known-size marker geometry with evaluation-only camera extrinsics."""
    if patch is None:
        return None
    t = patch.get("translation", [])
    origin = [number(row, observer + "_camera_" + axis) for axis in "xyz"]
    matrix = [number(row, observer + "_camera_r" + str(i)) for i in range(9)]
    own = [number(row, observer + "_true_x"), number(row, observer + "_true_z")]
    other = [number(row, target + "_true_x"), number(row, target + "_true_z")]
    if len(t) != 3 or not all(math.isfinite(v) for v in t) or t[2] <= 0:
        return None
    if None in origin + matrix + own + other:
        return None
    local = [t[2], -t[0], -t[1]]  # OpenCV right/down/forward -> Webots forward/left/up
    point = [origin[i] + sum(matrix[i * 3 + k] * local[k] for k in range(3))
             for i in range(3)]
    distance = math.hypot(point[0] - own[0], point[2] - own[1])
    truth = math.hypot(other[0] - own[0], other[1] - own[1])
    return {"estimated_range_m": distance, "range_error_m": abs(distance - truth),
            "position_error_m": math.hypot(point[0] - other[0], point[2] - other[1]),
            "estimated_x": point[0], "estimated_z": point[2],
            "camera_pose_source": "evaluation_only_truth"}


def evaluate(replay: dict) -> dict:
    cache = {}
    matches = []
    independent_markers = []
    unmatched_markers = unscored_markers = 0
    unmatched = unscored = 0
    for frame in replay["frames"]:
        image = Path(frame["image"])
        parsed = re.fullmatch(r"((?:red|blue)_[12])_(\d+)\.jpg", image.name)
        if parsed is None:
            unscored += len(frame["robots"])
            unscored_markers += len(frame.get("number_patches", []))
            continue
        observer, millis = parsed.groups()
        trace = image.parent.parent / "world_trace.csv"
        if trace not in cache:
            with trace.open() as stream:
                rows = list(csv.DictReader(stream))
            cache[trace] = ([float(row["time"]) for row in rows], rows)
        times, rows = cache[trace]
        if not rows:
            unscored += len(frame["robots"])
            unscored_markers += len(frame.get("number_patches", []))
            continue
        stamp = int(millis) / 1000
        index = min(bisect.bisect_left(times, stamp), len(times) - 1)
        if index and abs(times[index - 1] - stamp) < abs(times[index] - stamp):
            index -= 1
        if abs(times[index] - stamp) > 0.04:
            unscored += len(frame["robots"])
            unscored_markers += len(frame.get("number_patches", []))
            continue
        row = rows[index]
        projected = {name: project_robot(row, observer, name)
                     for name in ROBOTS if name != observer}
        used = set()
        for box in frame["robots"]:
            candidates = []
            for target, pixel in projected.items():
                if pixel is None or target in used:
                    continue
                dx = abs(box["x"] - pixel[0]) / max(0.01, box["width"])
                dy = abs(box["y"] - pixel[1]) / max(0.01, box["height"])
                if dx <= 0.7 and dy <= 0.7:
                    candidates.append((dx + dy, target))
            if not candidates:
                unmatched += 1
                continue
            _, target = min(candidates)
            used.add(target)
            true_range = math.hypot(float(row[target + "_true_x"]) -
                                    float(row[observer + "_true_x"]),
                                    float(row[target + "_true_z"]) -
                                    float(row[observer + "_true_z"]))
            estimate = min(6.0, max(0.45, 0.68 / (
                2 * math.tan(1.3613 / 2) * 0.75 * box["height"])))
            bottom = ground_range_proxy(row, observer, box["x"], box["y"] + box["height"] / 2)
            patch = number_patch_proxy(row, observer, target, box.get("number_patch"))
            matches.append({"image": str(image), "observer": observer, "target": target,
                            "expected_team": target.split("_")[0], "team": box["team"],
                            "true_range": true_range, "estimated_range": estimate,
                            "range_error": abs(estimate - true_range),
                            "bbox_bottom_range_proxy_m": bottom,
                            "bbox_bottom_range_error_m": abs(bottom - true_range)
                            if bottom is not None else None,
                            "detector_box": box,
                            "number_patch_proxy": patch,
                            "box_height": box["height"],
                            "clipped": (box["x"] - box["width"] / 2 < .01 or
                                        box["x"] + box["width"] / 2 > .99 or
                                        box["y"] - box["height"] / 2 < .01 or
                                        box["y"] + box["height"] / 2 > .99),
                            "ambiguous": len(candidates) > 1})
        marker_pixels = {target: project_robot(row, observer, target, .480)
                         for target in ROBOTS if target != observer}
        used = set()
        for patch in frame.get("number_patches", []):
            candidates = [target for target, p in marker_pixels.items() if p is not None
                          and target not in used and abs(patch["x"] - p[0]) <= .7 * patch["width"]
                          and abs(patch["y"] - p[1]) <= .7 * patch["height"]]
            if len(candidates) != 1:
                unmatched_markers += 1
                continue
            target = candidates[0]
            proxy = number_patch_proxy(row, observer, target, patch)
            if proxy is None:
                unscored_markers += 1
                continue
            used.add(target)
            independent_markers.append({"image": str(image), "target": target, "patch": patch,
                                        "expected_team": target.split("_")[0], "geometry": proxy,
                                        "body_box_present": any(
                                            abs(patch["x"] - b["x"]) <= b["width"] / 2 and
                                            abs(patch["y"] - b["y"]) <= b["height"] / 2
                                            for b in frame["robots"])})
    confusion = collections.Counter(row["expected_team"] + ":" + row["team"] for row in matches)
    return {
        "matched_proxies": len(matches), "unmatched_boxes": unmatched,
        "unscored_boxes": unscored, "team_confusion": dict(confusion),
        "range_errors_m": distribution([row["range_error"] for row in matches]),
        "by_observer_team": {team: distribution([row["range_error"] for row in matches
                                                if row["observer"].startswith(team)])
                             for team in ("red", "blue")},
        "caveat": ("Trace camera truth is evaluation-only. Matching uses projected centres at "
                   "fixed height 0.365 m; falls, overlap, occlusion and timing affect labels. "
                   "Spatial proxies are not a manually labelled precision/recall benchmark."),
        "matches": matches,
        "range_calibration_probe": calibrate_range(matches),
        "bbox_bottom_probe": compare_bottom_range(matches),
        "number_patch_probe": compare_number_patch(matches, replay),
        "independent_number_patch_probe": {
            "landmarks": sum(len(f.get("number_patches", [])) for f in replay["frames"]),
            "scored": len(independent_markers), "unmatched_or_ambiguous": unmatched_markers,
            "unscored": unscored_markers,
            "without_body_box": sum(not r["body_box_present"] for r in independent_markers),
            "position_errors_m": distribution(
                [r["geometry"]["position_error_m"] for r in independent_markers]),
            "deployable_as_precise_tactical_input": False,
            "caveat": "Fixed .480m chest projection is an evaluation proxy, not a labelled marker",
            "matches": independent_markers},
        "trace_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in cache},
        "replay_binary_sha256": replay.get("binary_sha256"),
        "replay_model_sha256": replay.get("model_sha256"),
    }


def compare_number_patch(matches, replay):
    """Report sparse marker coverage as well as same-subset errors, never just best frames."""
    patches = sum(box.get("number_patch") is not None for frame in replay["frames"]
                  for box in frame["robots"])
    usable = [r for r in matches if not r["ambiguous"] and r["number_patch_proxy"] is not None]
    return {"detector_boxes": sum(len(frame["robots"]) for frame in replay["frames"]),
            "accepted_patch_boxes": patches, "unambiguous_scored_patches": len(usable),
            "height_error_m": distribution([r["range_error"] for r in usable]),
            "number_patch_range_error_m": distribution(
                [r["number_patch_proxy"]["range_error_m"] for r in usable]),
            "number_patch_position_error_m": distribution(
                [r["number_patch_proxy"]["position_error_m"] for r in usable]),
            "deployable": False,
            "limits": ["Only a complete sufficiently large FRONT white number square is supported",
                       "Receipt-clock recordings cannot be scored as capture time",
                       "Marker/body offset, overlap and centre matching affect proxy truth",
                       "Truth rotation isolates geometry, not original head-target error",
                       "No pose estimate is not a detector false negative or precise obstacle"]}


def compare_bottom_range(matches):
    """Compare both estimates on the same unambiguous, unclipped proxy subset."""
    usable = [r for r in matches if not r["clipped"] and not r["ambiguous"]
              and r["bbox_bottom_range_error_m"] is not None]
    return {"matched_subset_n": len(usable),
            "height_error_m": distribution([r["range_error"] for r in usable]),
            "bbox_bottom_error_m": distribution([r["bbox_bottom_range_error_m"] for r in usable]),
            "deployable": False,
            "limits": ["Ground-truth camera pose only isolates pixel/geometry errors",
                       "Box bottom is not a learned base footprint or guaranteed visible feet",
                       "Root ground position is a proxy, not a labelled footprint",
                       "Original target head angles are not calibrated actual camera extrinsics"]}


def calibrate_range(matches):
    """Test a single scale correction before considering complex depth models."""
    scale = 2 * math.tan(1.3613 / 2) * .75
    training = [r for r in matches if r["observer"].startswith("red")
                and not r["clipped"] and not r["ambiguous"]]
    heldout = [r for r in matches if r["observer"].startswith("blue")]
    if len(training) < 8 or not heldout:
        return {"usable": False, "reason": "too few training or heldout proxies"}
    height = statistics.median(r["true_range"] * scale * r["box_height"] for r in training)
    baseline = [r["range_error"] for r in heldout]
    corrected = [abs(min(6, max(.45, height / (scale * r["box_height"]))) - r["true_range"])
                 for r in heldout]
    before, after = distribution(baseline), distribution(corrected)
    return {"training_n": len(training), "heldout_n": len(heldout),
            "effective_height_m": height, "baseline_heldout_error_m": before,
            "corrected_heldout_error_m": after,
            "improves_median_and_p95": after["median"] < before["median"]
            and after["p95"] < before["p95"],
            "deployable": False,
            "reason": "spatial proxies, one dataset; independent labels needed"}


def score_static_view(sample, frame, pose):
    """Score body boxes and independent marker ROIs with sensor-only FK."""
    name, target = sample["observer"], sample["target"]
    dx = float(sample["target_x"]) - float(sample["observer_x"])
    dz = float(sample["target_z"]) - float(sample["observer_z"])
    row = {name + "_camera_" + k: v for k, v in zip("xyz", pose["origin"])}
    row.update({name + "_camera_r" + str(i): v for i, v in enumerate(pose["rotation"])})
    row.update({name + "_true_x": 0, name + "_true_z": 0,
                target + "_true_x": dx, target + "_true_z": dz})
    valid = (sample["image_saved"] == "1" and int(sample["fall"]) == 0
             and all(0 <= float(sample[k]) <= .25 for k in ("image_age", "imu_age", "head_age"))
             and math.hypot(float(sample["target_vx"]), float(sample["target_vz"])) <= .02)
    result = {"image": sample["image"], "scenario": sample["name"], "sample_valid": valid,
              "true_range_m": math.hypot(dx, dz), "body_matches": 0, "marker_matches": 0,
              "height_position_error_m": None, "bottom_position_error_m": None,
              "marker_position_error_m": None, "marker_range_error_m": None}
    result["observer_motion"] = {key: number(sample, key) for key in (
        "observer_vx", "observer_vz", "observer_omega_y", "phase_time")}
    if not valid:
        return result

    def candidates(boxes, height):
        pixel = project_robot(row, name, target, height)
        if pixel is None:
            return []
        return [b for b in boxes if abs(b["x"] - pixel[0]) <= .7 * b["width"]
                and abs(b["y"] - pixel[1]) <= .7 * b["height"]]

    boxes = candidates(frame["robots"], float(sample["target_y"]))
    markers = candidates(frame.get("number_patches", []), float(sample["target_y"]) + .115)
    result["body_matches"], result["marker_matches"] = len(boxes), len(markers)
    if len(boxes) == 1:
        box = boxes[0]
        scale = 2 * math.tan(1.3613 / 2)
        distance = min(6, max(.45, .68 / (.75 * scale * box["height"])))
        angle = (float(sample["imu_yaw"]) + float(sample["head_yaw"])) * math.pi / 180
        angle += math.atan((.5 - box["x"]) * scale)
        result["height_online_eligible"] = .04 < box["height"] < .95
        result["height_position_error_m"] = math.hypot(
            distance * math.cos(angle) - dx, -distance * math.sin(angle) - dz)
        bottom = box["y"] + box["height"] / 2
        if 0 <= bottom <= 1:
            local = [1, (.5 - box["x"]) * scale, (.5 - bottom) * scale * .75]
            direction = [sum(pose["rotation"][i * 3 + k] * local[k] for k in range(3))
                         for i in range(3)]
            travel = -pose["origin"][1] / direction[1] if direction[1] < -.05 else -1
            if 0 < travel <= 12:
                result["bottom_position_error_m"] = math.hypot(
                    pose["origin"][0] + travel * direction[0] - dx,
                    pose["origin"][2] + travel * direction[2] - dz)
    if len(markers) == 1:
        proxy = number_patch_proxy(row, name, target, markers[0])
        if proxy:
            result["marker_position_error_m"] = proxy["position_error_m"]
            result["marker_range_error_m"] = proxy["range_error_m"]
            result["marker_team_correct"] = markers[0]["team"] == target.split("_")[0]
    return result


def evaluate_static_views(replay, directory, camera_binary):
    result = json.loads((directory / "result.json").read_text())
    manifest = json.loads((directory / "manifest.json").read_text())
    profile = manifest["arguments"]["profile"]
    if (not result["complete"]
            or profile not in ("robot-views", "robot-dynamic", "robot-ball-views")):
        raise ValueError("complete controlled robot measurement required")
    with (directory / "robot_views.csv").open() as stream:
        samples = list(csv.DictReader(stream))
    frames = {Path(f["image"]).name: f for f in replay["frames"]}
    if len(frames) != len(replay["frames"]) or set(frames) != {s["image"] for s in samples}:
        raise ValueError("replay must cover the controlled samples exactly once")
    before = hashlib.sha256(camera_binary.read_bytes()).hexdigest()
    views = []
    for sample in samples:
        frame = frames[sample["image"]]
        path = directory / "images" / sample["image"]
        image_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        if replay.get("image_sha256", {}).get(str(path)) != image_hash:
            raise ValueError("controlled image changed or replay image hash missing")
        command = [str(camera_binary), "--pose"] + [sample[k] for k in (
            "imu_yaw", "imu_pitch", "imu_roll", "head_yaw", "head_pitch")]
        pose = json.loads(subprocess.check_output(command, text=True))
        views.append(score_static_view(sample, frame, pose))
    if before != hashlib.sha256(camera_binary.read_bytes()).hexdigest():
        raise ValueError("sensor camera FK binary changed during evaluation")
    return {"schema": "cupcup-controlled-robot-views-v1", "views": views, "profile": profile,
            "samples": len(samples), "valid_samples": sum(v["sample_valid"] for v in views),
            "body_target_matches": sum(v["body_matches"] == 1 for v in views),
            "marker_target_matches": sum(v["marker_matches"] == 1 for v in views),
            "position_error_m": {method: distribution([
                v[method + "_position_error_m"] for v in views
                if v[method + "_position_error_m"] is not None])
                for method in ("height", "bottom", "marker")},
            "camera_pose_source": "original_IMU_and_head_targets_using_production_FK",
            "camera_root_height_m": .365,
            "camera_fk_binary_sha256": before, "measurement_manifest": manifest,
            "input_sha256": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                             for name in ("robot_views.csv", "manifest.json", "result.json")},
            "deployable": False,
            "limits": ["Receipt-time proxies, no exposure or actual neck-feedback synchronization",
                       "CLI nominal root height .365m, not runtime ball projection height .345m",
                       "Target/root truth only for offline association and error",
                       "Per-method coverage differs; these means are not paired comparisons",
                       "Relative geometry only, excludes noisy global self localization",
                       "Other visible robots unlabelled; unmatched is not a false positive"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--static-views", type=Path,
                        help="controlled robot-views/robot-dynamic directory")
    parser.add_argument("--camera-binary", type=Path,
                        help="camera_geometry_test providing production sensor-only FK")
    args = parser.parse_args()
    replay_bytes = args.replay.read_bytes()
    if args.static_views and not args.camera_binary:
        parser.error("--static-views requires --camera-binary")
    replay = json.loads(replay_bytes)
    report = (evaluate_static_views(replay, args.static_views, args.camera_binary)
              if args.static_views else evaluate(replay))
    report["replay_sha256"] = hashlib.sha256(replay_bytes).hexdigest()
    report["analyzer_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("matches", "views")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
