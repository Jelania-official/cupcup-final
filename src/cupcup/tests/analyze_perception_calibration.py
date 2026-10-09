#!/usr/bin/env python3
"""Score fixed-pose Webots ball detections by range and head orientation."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

from analyze_world_trace import distribution, number, parse_talk


RANGES = (1.5, 2.5, 3.5)
YAWS = (-20.0, 0.0, 20.0)
PITCHES = (20.0, 40.0)
STAGE_SECONDS = 2.0
HORIZONTAL_FOV = 1.3613
HORIZONTAL_HALF_TAN = math.tan(HORIZONTAL_FOV / 2.0)
VERTICAL_HALF_TAN = HORIZONTAL_HALF_TAN * 480.0 / 640.0


def projected_range(sample: tuple[float, ...], height: float,
                    pitch_offset: float) -> float | None:
    """Use an effective camera height and pitch offset for the ball-center ray."""
    _, u, v, head_pitch, imu_pitch, _ = sample
    vertical_ray = math.atan((v - 0.5) * 2.0 * VERTICAL_HALF_TAN)
    horizontal_ray = math.atan((u - 0.5) * 2.0 * HORIZONTAL_HALF_TAN)
    down_angle = math.radians(head_pitch + imu_pitch + pitch_offset) + vertical_ray
    if down_angle < 0.03 or down_angle > 1.35:
        return None
    return height / math.tan(down_angle) / math.cos(horizontal_ray)


def fit_geometry(stages: list[dict[str, object]]) -> tuple[float, float] | None:
    """Fit on straight-ahead stages; left/right stages remain untouched."""
    training = [stages[index]["samples"] for index in (2, 3, 8, 9, 14, 15)]
    training = [group for group in training if group]
    if len(training) < 3:
        return None
    best: tuple[float, float, float] | None = None
    for height_step in range(25, 81):
        height = height_step / 100.0
        for offset_step in range(-10, 51):
            offset = offset_step / 2.0
            stage_errors = []
            for group in training:
                errors = []
                for sample in group:
                    estimate = projected_range(sample, height, offset)
                    errors.append(5.0 if estimate is None else
                                  abs(estimate - sample[0]))
                stage_errors.append(statistics.median(errors))
            loss = statistics.fmean(stage_errors)
            if best is None or loss < best[0]:
                best = (loss, height, offset)
    return (best[1], best[2]) if best is not None else None


def compare_geometry(stages: list[dict[str, object]], height: float,
                     offset: float) -> None:
    """Print front-fit and held-out side-view errors separately."""
    for name, indices in (("front_fit", (2, 3, 8, 9, 14, 15)),
                          ("side_holdout", (0, 1, 4, 5, 6, 7, 10, 11,
                                            12, 13, 16, 17))):
        radius_errors = []
        geometric_errors = []
        for index in indices:
            for sample in stages[index]["samples"]:
                estimate = projected_range(sample, height, offset)
                if estimate is None:
                    continue
                radius_errors.append(abs(0.050 / sample[5] - sample[0]))
                geometric_errors.append(abs(estimate - sample[0]))
        radius = distribution(radius_errors)
        geometric = distribution(geometric_errors)
        if radius["n"]:
            print(f"{name}: n={radius['n']} radius_median={radius['median']:.3f} "
                  f"ray_median={geometric['median']:.3f} "
                  f"radius_p95={radius['p95']:.3f} "
                  f"ray_p95={geometric['p95']:.3f}")
        else:
            print(f"{name}: n=0")


def score(trace: Path, color: str = "red") -> list[dict[str, object]]:
    """Keep distinct images after the fixture has settled in each stage."""
    stages: list[dict[str, object]] = [
        {"images": 0, "settled": 0, "detected": 0,
         "errors": [], "samples": []}
        for _ in range(18)
    ]
    previous_image_stamp = -1
    previous_ball_stamp = -1
    robot = f"{color}_1"
    expected_x = 1.5 if color == "red" else -1.5
    with trace.open(newline="", encoding="utf-8") as trace_file:
        for row in csv.DictReader(trace_file):
            fields = parse_talk(row.get(f"{robot}_talk", ""))
            if fields.get("sim_stamp_valid") != "1":
                continue
            image_stamp = number(fields, "istamp_ms")
            if image_stamp is None or image_stamp <= previous_image_stamp:
                continue
            previous_image_stamp = image_stamp
            phase = (image_stamp / 1000.0) % STAGE_SECONDS
            if phase < 1.20 or phase >= 1.95:
                continue
            stage = int(image_stamp / (STAGE_SECONDS * 1000)) % 18
            result = stages[stage]
            true_x = number(row, f"{robot}_true_x")
            true_z = number(row, f"{robot}_true_z")
            ball_x = number(row, "ball_x")
            ball_z = number(row, "ball_z")
            velocity_x = number(row, f"{robot}_vx")
            velocity_z = number(row, f"{robot}_vz")
            if None in (true_x, true_z, ball_x, ball_z,
                        velocity_x, velocity_z):
                continue
            expected_range = RANGES[stage // 6]
            expected_yaw = YAWS[(stage // 2) % 3]
            expected_ball_x = expected_x + (
                -1.0 if color == "red" else 1.0
            ) * expected_range * math.cos(math.radians(expected_yaw))
            expected_ball_z = (
                1.0 if color == "red" else -1.0
            ) * expected_range * math.sin(math.radians(expected_yaw))
            true_range = math.hypot(ball_x - true_x, ball_z - true_z)
            if any((
                math.hypot(true_x - expected_x, true_z) > 0.12,
                math.hypot(velocity_x, velocity_z) > 0.08,
                abs(true_range - expected_range) > 0.12,
                math.hypot(ball_x - expected_ball_x,
                           ball_z - expected_ball_z) > 0.12,
            )):
                continue
            result["images"] += 1
            head_yaw = number(fields, "ihy")
            head_pitch = number(fields, "ihp")
            expected_pitch = PITCHES[stage % 2]
            if head_yaw is None or head_pitch is None:
                continue
            if any((abs(head_yaw - expected_yaw) > 5.0,
                    abs(head_pitch - expected_pitch) > 5.0)):
                continue
            result["settled"] += 1
            seen_age = number(fields, "age")
            radius = number(fields, "ir")
            ball_stamp = number(fields, "bstamp_ms")
            if seen_age is None or radius is None or ball_stamp is None:
                continue
            if (seen_age > 0.25 or radius <= 0.005
                    or ball_stamp <= previous_ball_stamp
                    or int(ball_stamp / (STAGE_SECONDS * 1000)) % 18 != stage):
                continue
            previous_ball_stamp = ball_stamp
            result["detected"] += 1
            result["errors"].append(abs(0.050 / radius - true_range))
            u = number(fields, "iu")
            v = number(fields, "iv")
            imu_pitch = number(fields, "biip")
            head_pitch = number(fields, "bihp")
            if None not in (u, v, imu_pitch) and all((
                    0.0 <= u <= 1.0, 0.0 <= v <= 1.0)):
                result["samples"].append(
                    (true_range, u, v, head_pitch, imu_pitch, radius)
                )
    return stages


def main() -> int:
    """Print coverage and distance error for each range and head pose."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--color", choices=("red", "blue"), default="red")
    parser.add_argument("--height", type=float)
    parser.add_argument("--offset", type=float)
    args = parser.parse_args()
    if (args.height is None) != (args.offset is None):
        parser.error("--height and --offset must be supplied together")
    try:
        stages = score(args.trace, args.color)
    except OSError as error:
        parser.error(str(error))
    print("range_m,yaw_deg,pitch_deg,images,head_settled,ball_detected,"
          "median_range_error_m,p95_range_error_m")
    for index, stage in enumerate(stages):
        errors = distribution(stage["errors"])
        median = f"{errors['median']:.3f}" if errors["n"] else ""
        p95 = f"{errors['p95']:.3f}" if errors["n"] else ""
        print(f"{RANGES[index // 6]:.1f},{YAWS[(index // 2) % 3]:.0f},"
              f"{PITCHES[index % 2]:.0f},{stage['images']},"
              f"{stage['settled']},{stage['detected']},{median},{p95}")
    fitted = ((args.height, args.offset) if args.height is not None
              else fit_geometry(stages))
    if fitted is not None:
        height, offset = fitted
        print(f"effective_height_m={height:.2f} pitch_offset_deg={offset:.1f}")
        compare_geometry(stages, height, offset)
    else:
        print("geometry_fit=insufficient_front_samples")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
