#!/usr/bin/env python3
"""Compare ball range estimators on moving-match traces at image capture time."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path

from analyze_perception_calibration import projected_range
from analyze_world_trace import distribution, number, parse_talk
from replay_ball_pattern import truth_pixel


def oracle_camera_point(row: dict[str, str], robot: str,
                        u: float, v: float,
                        plane_height: float = 0.07) -> tuple[float, float] | None:
    """Intersect a detected pixel ray with ball-center height using trace-only pose."""
    origin = [number(row, f"{robot}_camera_{axis}") for axis in "xyz"]
    matrix = [number(row, f"{robot}_camera_r{index}") for index in range(9)]
    if any(value is None for value in origin + matrix):
        return None
    horizontal_tan = math.tan(1.3613 / 2.0)
    local = (1.0, (0.5 - u) * 2.0 * horizontal_tan,
             (0.5 - v) * 2.0 * horizontal_tan * 480.0 / 640.0)
    direction = [sum(matrix[3 * row_index + column] * local[column]
                     for column in range(3)) for row_index in range(3)]
    if direction[1] >= -0.01:
        return None
    # Actual height is legitimate only for the oracle upper bound. The robot's
    # FK result was already recorded online using its own nominal ground plane.
    truth_height = number(row, "ball_y")
    plane = plane_height if truth_height is None else truth_height
    distance = (plane - origin[1]) / direction[1]
    if distance <= 0.0 or distance > 30.0:
        return None
    return (origin[0] + distance * direction[0],
            origin[2] + distance * direction[2])


def nearest_pose(samples: list[tuple[int, float]],
                 target: float) -> float | None:
    """Find a reported sensor value no more than 40 ms from image time."""
    if not samples:
        return None
    stamps = [item[0] for item in samples]
    index = bisect.bisect_left(stamps, target)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    chosen = min(candidates, key=lambda item: abs(item[0] - target))
    return chosen[1] if abs(chosen[0] - target) <= 40 else None


def evaluate(trace: Path, color: str, height: float,
             offset: float, plane_height: float = 0.07) -> dict[str, dict[str, object]]:
    """Match each accepted ball observation to its own capture-time truth."""
    with trace.open(newline="", encoding="utf-8") as trace_file:
        rows = list(csv.DictReader(trace_file))
    times = [number(row, "time") for row in rows]
    if not rows or any(value is None for value in times):
        return {}
    robots = (f"{color}_1", f"{color}_2")
    posture: dict[str, dict[str, list[tuple[int, float]]]] = {}
    for robot in robots:
        heads: dict[int, float] = {}
        imus: dict[int, float] = {}
        for row in rows:
            fields = parse_talk(row.get(f"{robot}_talk", ""))
            head_stamp = number(fields, "htstamp_ms")
            imu_stamp = number(fields, "mtstamp_ms")
            head_pitch = number(fields, "hp")
            imu_pitch = number(fields, "ip")
            if head_stamp is not None and head_stamp > 0 and head_pitch is not None:
                heads[int(head_stamp)] = head_pitch
            if imu_stamp is not None and imu_stamp > 0 and imu_pitch is not None:
                imus[int(imu_stamp)] = imu_pitch
        posture[robot] = {
            "head": sorted(heads.items()), "imu": sorted(imus.items()),
        }
    errors: dict[str, dict[str, object]] = {
        name: {"observations": 0, "pose_rejected": 0, "samples": 0, "invalid_ray": 0,
               "radius": [], "ray": [],
               "moving_radius": [], "moving_ray": [],
               "aligned_radius": [], "aligned_ray": [],
               "moving_aligned_radius": [], "moving_aligned_ray": [],
               "oracle_radius": [], "oracle_range": [],
               "oracle_position": [], "camera_pitch_residual": [],
               "moving_camera_pitch_residual": [], "moving_oracle_radius": [],
               "moving_oracle_range": [], "moving_oracle_position": [],
               "fk_radius": [], "fk_range": [], "fk_position": [],
               "moving_fk_radius": [], "moving_fk_range": [], "moving_fk_position": [],
               "fk_orientation_deg": [], "fk_origin_m": [], "fk_height_m": [],
               "pixel_samples": 0, "pixel_unscored": 0,
               "pixel_error_px": [], "near_pixel_error_px": [],
               "far_pixel_error_px": [], "near_pixel_du_px": [],
               "near_pixel_dv_px": [], "pixel_by_state": {}}
        for name in robots
    }
    last_ball_stamp = {name: -1 for name in robots}
    for row in rows:
        for robot in robots:
            fields = parse_talk(row.get(f"{robot}_talk", ""))
            # Latest-image time is not the ball image's time: the tracker can
            # retain a previous detection through a dropped/rejected frame.
            image_stamp = number(fields, "bstamp_ms")
            if image_stamp is None or image_stamp <= 0:
                continue
            if image_stamp <= last_ball_stamp[robot]:
                continue
            last_ball_stamp[robot] = image_stamp
            result = errors[robot]
            result["observations"] += 1
            observed_age = number(fields, "age")
            if observed_age is None or not 0 <= observed_age <= 0.25:
                continue
            index = bisect.bisect_left(times, image_stamp / 1000.0)
            if index >= len(rows):
                index = len(rows) - 1
            if index > 0 and abs(times[index - 1] - image_stamp / 1000.0) < abs(
                    times[index] - image_stamp / 1000.0):
                index -= 1
            if abs(times[index] - image_stamp / 1000.0) > 0.04:
                continue
            truth = rows[index]
            ball_x = number(truth, "ball_x")
            ball_z = number(truth, "ball_z")
            robot_x = number(truth, f"{robot}_true_x")
            robot_z = number(truth, f"{robot}_true_z")
            velocity_x = number(truth, f"{robot}_vx")
            velocity_z = number(truth, f"{robot}_vz")
            angular_rate = number(truth, f"{robot}_omega_y")
            u = number(fields, "iu")
            v = number(fields, "iv")
            # Image control does not depend on the FK/range estimator. Score
            # pixels independently, even if reported posture is missing/stale.
            # This remains a simulator projection proxy, not semantic labels.
            projected = truth_pixel(truth, robot, plane_height)
            if (None not in (ball_x, ball_z, robot_x, robot_z, u, v) and
                    projected is not None and
                    all(0 <= coordinate <= 1 for coordinate in (*projected, u, v))):
                du, dv = (u - projected[0]) * 640, (v - projected[1]) * 480
                error = math.hypot(du, dv)
                result["pixel_samples"] += 1
                result["pixel_error_px"].append(error)
                near = math.hypot(ball_x - robot_x, ball_z - robot_z) < 1.0
                result["near_pixel_error_px" if near else "far_pixel_error_px"].append(error)
                if near:
                    result["near_pixel_du_px"].append(abs(du))
                    result["near_pixel_dv_px"].append(abs(dv))
                state = fields.get("state", "UNKNOWN")
                result["pixel_by_state"].setdefault(state, []).append(error)
            else:
                result["pixel_unscored"] += 1
            head_age = number(fields, "biha_sim")
            imu_age = number(fields, "biia_sim")
            if (None in (head_age, imu_age) or abs(head_age) > 0.15 or
                    abs(imu_age) > 0.15):
                result["pose_rejected"] += 1
                continue
            radius = number(fields, "ir")
            head_pitch = number(fields, "bihp")
            imu_pitch = number(fields, "biip")
            if (None in (ball_x, ball_z, robot_x, robot_z, velocity_x,
                         velocity_z, angular_rate, u, v, radius,
                         head_pitch, imu_pitch) or radius <= 0.005):
                continue
            true_range = math.hypot(ball_x - robot_x, ball_z - robot_z)
            sample = (true_range, u, v, head_pitch, imu_pitch, radius)
            result["samples"] += 1
            ray_range = projected_range(sample, height, offset)
            radius_error = abs(0.050 / radius - true_range)
            moving = math.hypot(velocity_x, velocity_z) > 0.08 or abs(angular_rate) > 0.1
            if ray_range is None:
                result["invalid_ray"] += 1
            else:
                ray_error = abs(ray_range - true_range)
                result["radius"].append(radius_error)
                result["ray"].append(ray_error)
                if moving:
                    result["moving_radius"].append(radius_error)
                    result["moving_ray"].append(ray_error)
            # Evaluate each alternative on its own paired valid subset. An
            # invalid simplified ray must NOT erase valid FK/oracle samples.
            if abs(head_age) <= 0.04 and abs(imu_age) <= 0.04:
                predicted = [number(fields, f"bcr{i}") for i in range(9)]
                actual = [number(truth, f"{robot}_camera_r{i}") for i in range(9)]
                if None not in predicted + actual:
                    cosine = (sum(a * b for a, b in zip(predicted, actual)) - 1.0) / 2.0
                    result["fk_orientation_deg"].append(math.degrees(math.acos(
                        max(-1.0, min(1.0, cosine)))))
                origin = [number(fields, f"bc{axis}") for axis in "xyz"]
                camera_origin = [number(truth, f"{robot}_camera_{axis}") for axis in "xyz"]
                if None not in origin + camera_origin:
                    origin_error = math.sqrt((origin[0] + robot_x - camera_origin[0]) ** 2
                                             + (origin[1] - camera_origin[1]) ** 2
                                             + (origin[2] + robot_z - camera_origin[2]) ** 2)
                    result["fk_origin_m"].append(origin_error)
                    result["fk_height_m"].append(abs(origin[1] - camera_origin[1]))
                gx, gz = number(fields, "bgx"), number(fields, "bgz")
                if number(fields, "bgvalid") == 1 and gx is not None and gz is not None:
                    position_error = math.hypot(gx + robot_x - ball_x, gz + robot_z - ball_z)
                    range_error = abs(math.hypot(gx, gz) - true_range)
                    result["fk_radius"].append(radius_error)
                    result["fk_range"].append(range_error)
                    result["fk_position"].append(position_error)
                    if moving:
                        result["moving_fk_radius"].append(radius_error)
                        result["moving_fk_range"].append(range_error)
                        result["moving_fk_position"].append(position_error)
            point = oracle_camera_point(truth, robot, u, v, plane_height)
            if point is not None:
                vertical = number(truth, f"{robot}_camera_r3")
                if vertical is not None:
                    actual_pitch = math.degrees(math.asin(max(-1.0, min(1.0, -vertical))))
                    pitch_residual = abs(actual_pitch - (head_pitch + imu_pitch + offset))
                    result["camera_pitch_residual"].append(pitch_residual)
                    if moving:
                        result["moving_camera_pitch_residual"].append(pitch_residual)
                position_error = math.hypot(point[0] - ball_x, point[1] - ball_z)
                oracle_error = abs(math.hypot(point[0] - robot_x,
                                              point[1] - robot_z) - true_range)
                result["oracle_radius"].append(radius_error)
                result["oracle_range"].append(oracle_error)
                result["oracle_position"].append(position_error)
                if moving:
                    result["moving_oracle_radius"].append(radius_error)
                    result["moving_oracle_range"].append(oracle_error)
                    result["moving_oracle_position"].append(position_error)
            aligned_head = nearest_pose(posture[robot]["head"], image_stamp)
            aligned_imu = nearest_pose(posture[robot]["imu"], image_stamp)
            if aligned_head is None or aligned_imu is None:
                continue
            aligned = projected_range(
                (true_range, u, v, aligned_head, aligned_imu, radius),
                height, offset,
            )
            if aligned is None:
                continue
            result["aligned_radius"].append(radius_error)
            result["aligned_ray"].append(abs(aligned - true_range))
            if moving:
                result["moving_aligned_radius"].append(radius_error)
                result["moving_aligned_ray"].append(abs(aligned - true_range))
    return errors


def main() -> int:
    """Print paired error distributions for each robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--color", choices=("red", "blue"), required=True)
    parser.add_argument("--height", type=float, default=0.57)
    parser.add_argument("--offset", type=float, default=10.5)
    parser.add_argument("--plane-height", type=float, default=0.07,
                        help="ball-center height: normal world 0.07, calibration fixture 0.05")
    parser.add_argument("--output", type=Path, help="save distributions and provenance as JSON")
    args = parser.parse_args()
    try:
        errors = evaluate(args.trace, args.color, args.height, args.offset, args.plane_height)
    except OSError as error:
        parser.error(str(error))
    if args.output:
        def summarize(value):
            if isinstance(value, list):
                return distribution(value)
            if isinstance(value, dict):
                return {key: summarize(item) for key, item in value.items()}
            return value
        report = {
            "schema": "cupcup-ball-geometry-diagnostic-v1",
            "trace": str(args.trace),
            "trace_sha256": hashlib.sha256(args.trace.read_bytes()).hexdigest(),
            "analysis_sha256": {name: hashlib.sha256(
                Path(__file__).with_name(name).read_bytes()).hexdigest() for name in
                ("compare_ball_geometry.py", "replay_ball_pattern.py", "analyze_world_trace.py")},
            "color": args.color, "height": args.height, "offset": args.offset,
            "plane_height": args.plane_height,
            "results": summarize(errors),
            "caveat": "Accepted detections only, paired to capture-time truth within 40 ms. "
                      "Pixels use measured camera pose as an offline projection proxy; "
                      "not manual labels, precision/recall, or original-sensor validation. "
                      "Near means actual robot-ball range <1 m. Signed bias is not reported; "
                      "du/dv are absolute errors. Local pixel and FK subsets differ. "
                      "State groups use the reporting Talk state, not an image-time state label.",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
    for robot, result in errors.items():
        print(f"{robot} images={result['samples']} invalid_ray={result['invalid_ray']}")
        print(f"  accepted_observations={result['observations']} "
              f"pose_rejected={result['pose_rejected']}")
        for key in ("pixel_error_px", "near_pixel_error_px", "far_pixel_error_px",
                    "near_pixel_du_px", "near_pixel_dv_px"):
            print(f"  {key}: {distribution(result[key])}")
        for label, radius_key, ray_key in (
                ("all", "radius", "ray"),
                ("moving", "moving_radius", "moving_ray"),
                ("time_aligned", "aligned_radius", "aligned_ray"),
                ("moving_time_aligned", "moving_aligned_radius",
                 "moving_aligned_ray")):
            radius = distribution(result[radius_key])
            ray = distribution(result[ray_key])
            if radius["n"]:
                print(f"  {label}: n={radius['n']} "
                      f"radius_median={radius['median']:.3f} "
                      f"ray_median={ray['median']:.3f} "
                      f"radius_p95={radius['p95']:.3f} "
                      f"ray_p95={ray['p95']:.3f}")
            else:
                print(f"  {label}: n=0")
        for label, key in (("camera_pitch_residual_deg", "camera_pitch_residual"),
                           ("moving_camera_pitch_residual_deg",
                            "moving_camera_pitch_residual"),
                           ("fk_orientation_deg", "fk_orientation_deg"),
                           ("fk_origin_m", "fk_origin_m"),
                           ("fk_height_m", "fk_height_m")):
            values = distribution(result[key])
            if values["n"]:
                print(f"  {label}: n={values['n']} "
                      f"median={values['median']:.2f} p95={values['p95']:.2f}")
        for label, radius_key, oracle_key, position_key in (
                ("oracle_camera", "oracle_radius", "oracle_range",
                 "oracle_position"),
                ("moving_oracle_camera", "moving_oracle_radius",
                 "moving_oracle_range", "moving_oracle_position"),
                ("sensor_fk", "fk_radius", "fk_range", "fk_position"),
                ("moving_sensor_fk", "moving_fk_radius", "moving_fk_range",
                 "moving_fk_position")):
            radius = distribution(result[radius_key])
            oracle = distribution(result[oracle_key])
            position = distribution(result[position_key])
            if radius["n"]:
                print(f"  {label}: n={radius['n']} "
                      f"radius_median={radius['median']:.3f} "
                      f"radius_p95={radius['p95']:.3f} "
                      f"estimated_range_median={oracle['median']:.3f} "
                      f"estimated_range_p95={oracle['p95']:.3f} "
                      f"estimated_position_median={position['median']:.3f} "
                      f"estimated_position_p95={position['p95']:.3f}")
            else:
                print(f"  {label}: n=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
