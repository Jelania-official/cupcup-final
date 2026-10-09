#!/usr/bin/env python3
"""Score controlled original-input ball views without assuming capture timestamps."""

import argparse
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

from analyze_world_trace import distribution


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurement", type=Path)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--camera-binary", type=Path, required=True)
    parser.add_argument("--camera-root-height", type=float, default=.365,
                        help="nominal root height in metres; use .345 for current runtime config")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.camera_root_height) or not .25 <= args.camera_root_height <= .50:
        parser.error("camera root height must be finite and between .25 and .50 metres")
    root = args.measurement.resolve()
    manifest_path, result_path = root / "manifest.json", root / "result.json"
    manifest = json.loads(manifest_path.read_text())
    result = json.loads(result_path.read_text())
    profile = manifest["arguments"]["profile"]
    if profile not in ("ball-views", "ball-dynamic", "robot-ball-views") or not result["complete"]:
        parser.error("complete ball view fixture required")
    csv_path = root / ("robot_views.csv" if profile == "robot-ball-views" else "ball_views.csv")
    with csv_path.open() as stream:
        samples = list(csv.DictReader(stream))
    if len(samples) != 8 * int(manifest["arguments"]["repeats"]):
        parser.error("all eight predeclared views per repeat required")
    replay = json.loads(args.replay.read_text())
    frames = {Path(frame["image"]).resolve(): frame for frame in replay["frames"]}
    images = [(root / "images" / sample["image"]).resolve() for sample in samples]
    if (len(frames) != len(replay["frames"]) or set(frames) != set(images)
            or len(images) != len(set(images))):
        parser.error("exact replay coverage, without duplicate frames, required")
    artifacts = [manifest_path, result_path, csv_path, args.replay, args.camera_binary, *images]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in artifacts}
    for image in images:
        if replay.get("image_sha256", {}).get(str(image)) != hashes[str(image)]:
            parser.error("replay image hash mismatch")
    scored = []
    for sample, image in zip(samples, images):
        if profile == "robot-ball-views":
            # Relabel only the evaluation target; keep the original CSV/hash.
            sample = sample | {"target": "ball"} | {
                "target_" + axis: sample["ball_" + axis] for axis in ("x", "z", "vx", "vz")}
        row = {"trial": int(sample["trial"]), "image": str(image), "name": sample["name"]}
        numeric = {key: float(sample[key]) for key in (
            "image_age", "imu_age", "head_age", "observer_x", "observer_z",
            "target_x", "target_z", "target_vx", "target_vz",
            "imu_yaw", "imu_pitch", "imu_roll", "head_yaw", "head_pitch")}
        row["head_target_degrees"] = {
            "yaw": numeric["head_yaw"], "pitch": numeric["head_pitch"]}
        usable = (sample["target"] == "ball" and sample["fall"] == "0"
                  and all(math.isfinite(v) for v in numeric.values())
                  and all(0 <= numeric[k] <= .25 for k in ("image_age", "imu_age", "head_age"))
                  and math.hypot(numeric["target_vx"], numeric["target_vz"]) <= .02)
        row["usable_fixture"] = usable
        if "observer_vx" in sample:
            row["observer_speed_m_s"] = math.hypot(
                float(sample["observer_vx"]), float(sample["observer_vz"]))
            row["observer_turn_rate_rad_s"] = float(sample["observer_omega_y"])
            row["phase_time_s"] = float(sample["phase_time"])
        ball = frames[image].get("ball", {})
        row["detected"] = bool(ball.get("valid"))
        if usable and row["detected"]:
            x, y, radius = (float(ball[k]) for k in ("x", "y", "radius"))
            if (not all(math.isfinite(v) for v in (x, y, radius)) or radius <= 0
                    or not 0 <= x <= 1 or not 0 <= y <= 1):
                parser.error("invalid decoded ball")
            angles = [numeric[k] for k in (
                "imu_yaw", "imu_pitch", "imu_roll", "head_yaw", "head_pitch")]
            point = json.loads(subprocess.check_output(
                [str(args.camera_binary), "--project", *map(str, angles), str(x), str(y),
                 str(args.camera_root_height)],
                text=True))
            if point.get("root_height_m") != args.camera_root_height:
                parser.error("camera binary did not confirm the requested root height")
            dx = numeric["target_x"] - numeric["observer_x"]
            dz = numeric["target_z"] - numeric["observer_z"]
            row.update({"ball": ball, "true_range_m": math.hypot(dx, dz),
                        "radius_range_error_m": abs(.05 / radius - math.hypot(dx, dz)),
                        "ground_projection_valid": point["valid"]})
            if point["valid"]:
                row["ground_xy_error_m"] = math.hypot(point["x"] - dx, point["z"] - dz)
        scored.append(row)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != hashes[str(p)] for p in artifacts):
        parser.error("measurement, replay or camera binary changed during scoring")
    report = {
        "schema": "cupcup-ball-view-score-v2", "samples": len(scored),
        "measurement_profile": profile,
        "camera_root_height_m": args.camera_root_height,
        "comparison_clock": "settled-receipt" if profile == "ball-views" else "receipt-only",
        "usable_fixtures": sum(r["usable_fixture"] for r in scored),
        "detected_usable_fixtures": sum(r["usable_fixture"] and r["detected"] for r in scored),
        "ground_xy_error_m": distribution([
            r["ground_xy_error_m"] for r in scored if "ground_xy_error_m" in r]),
        "radius_range_error_m": distribution([
            r["radius_range_error_m"] for r in scored if "radius_range_error_m" in r]),
        "artifact_sha256": hashes, "frames": scored,
        "limits": "Decoded output counts are candidate counts, not verified ball detections. "
                  "Received RGB and head targets; no exposure synchronization. "
                  "Dynamic cases are receipt-time geometry proxies, not capture-time calibration. "
                  "Production decoder without temporal prior; nominal sensor FK, no camera truth. "
                  "Offline true target/observer positions score relative geometry only. "
                  "Not manual precision/recall, global map accuracy or fitted noise sigma.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in (
        "frames", "artifact_sha256")}, indent=2))


if __name__ == "__main__":
    main()
