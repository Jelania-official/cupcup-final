#!/usr/bin/env python3
"""Compare Webots truth with the observations exposed to the strategy."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path


ROBOTS = ("red_1", "red_2", "blue_1", "blue_2")
RANGE_BANDS = (("<1m", 0.0, 1.0), ("1-2m", 1.0, 2.0),
               ("2-3m", 2.0, 3.0), ("3m+", 3.0, math.inf))
RANGE_SOURCES = ("ground_ray", "radius", "unknown")


def parse_talk(message: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    parts = message.split("|")
    if not parts or parts[0] != "cupcup":
        return fields
    for part in parts[1:]:
        key, separator, value = part.partition("=")
        if separator:
            fields[key] = value
    return fields


def number(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def distribution(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"n": 0}
    ordered = sorted(values)

    def percentile(fraction: float) -> float:
        index = (len(ordered) - 1) * fraction
        lower = math.floor(index)
        upper = math.ceil(index)
        return (
            ordered[lower]
            + (ordered[upper] - ordered[lower]) * (index - lower)
        )

    return {
        "n": len(ordered),
        "mean": statistics.fmean(ordered),
        "median": statistics.median(ordered),
        "p90": percentile(0.90),
        "p95": percentile(0.95),
        "max": ordered[-1],
    }


def analyze_rows(rows: list[dict[str, str]]) -> dict[str, object]:
    location_errors: dict[str, list[float]] = {name: [] for name in ROBOTS}
    pose_errors: dict[str, list[float]] = {name: [] for name in ROBOTS}
    ball_errors: dict[str, list[float]] = {name: [] for name in ROBOTS}
    heading_residuals = {name: [] for name in ROBOTS}
    heading_offsets = {name: [] for name in ROBOTS}
    robot_speeds: dict[str, list[float]] = {name: [] for name in ROBOTS}
    robot_turn_rates: dict[str, list[float]] = {name: [] for name in ROBOTS}
    range_errors: dict[str, dict[str, list[float]]] = {
        name: {band[0]: [] for band in RANGE_BANDS} for name in ROBOTS
    }
    source_range_errors = {
        name: {source: {band[0]: [] for band in RANGE_BANDS}
               for source in RANGE_SOURCES} for name in ROBOTS
    }
    local_xy_errors = {
        name: {band[0]: [] for band in RANGE_BANDS} for name in ROBOTS
    }
    sensor_ages: dict[str, dict[str, list[float]]] = {
        name: {key: [] for key in (
            "fa", "iha", "iia", "iha_sim", "iia_sim", "ila_sim"
        )} for name in ROBOTS
    }
    ranged_map_errors: dict[str, dict[str, list[float]]] = {
        name: {band[0]: [] for band in RANGE_BANDS} for name in ROBOTS
    }
    ball_speeds: list[float] = []
    previous_talk_sequence = {name: -1 for name in ROBOTS}
    arrived_view_angles = {name: [] for name in ROBOTS}
    coverage = {name: {"play_talk_messages": 0, "fresh_ball_messages": 0,
                       "robot_fields_missing_messages": 0, "number_square_messages": 0,
                       "number_square_candidates": 0,
                       "arrived_nonowner_messages": 0,
                       "arrived_nonowner_head_saturated_messages": 0,
                       "arrived_nonowner_number_square_messages": 0} for name in ROBOTS}
    previous_pair_sequence = {"red": None, "blue": None}
    pair_fusion_errors = {"red": [], "blue": []}
    pair_single_errors = {"red": [], "blue": []}
    role_occupancy = {name: {
        "play_position_samples": 0, "document_role_region_samples": 0,
        "released_stop_region_samples": 0, "outside_document_field_samples": 0,
        "max_document_role_penetration_m": 0.0,
    } for name in ROBOTS}
    required_columns = ["ball_x", "ball_z", "ball_vx", "ball_vz"]
    for name in ROBOTS:
        required_columns.extend((
            f"{name}_true_x", f"{name}_true_z", f"{name}_vx", f"{name}_vz",
            f"{name}_omega_y", f"{name}_obs_x", f"{name}_obs_z",
            f"{name}_talk_seq", f"{name}_talk",
        ))
    complete_rows = [
        row for row in rows
        if all(row.get(column) is not None for column in required_columns)
    ]

    for row in complete_rows:
        ball_vx = number(row, "ball_vx")
        ball_vz = number(row, "ball_vz")
        if ball_vx is not None and ball_vz is not None:
            ball_speeds.append(math.hypot(ball_vx, ball_vz))

        for name in ROBOTS:
            true_x = number(row, f"{name}_true_x")
            true_z = number(row, f"{name}_true_z")
            observed_x = number(row, f"{name}_obs_x")
            observed_z = number(row, f"{name}_obs_z")
            if number(row, "state") == 2 and None not in (true_x, true_z):
                # Offline geometric evidence only: judge relocation/penalty
                # state is unavailable, so occupancy is not a penalty verdict.
                own_x = true_x if name.startswith("red") else -true_x
                forward = name.endswith("1")
                document_depth = min(own_x - 2.5, 2.5 - abs(true_z)) if forward else -own_x
                released_depth = min(own_x - 2.7, 2.0 - abs(true_z)) if forward else -own_x - .2
                occupancy = role_occupancy[name]
                occupancy["play_position_samples"] += 1
                occupancy["document_role_region_samples"] += int(document_depth > 0)
                occupancy["released_stop_region_samples"] += int(released_depth > 0)
                occupancy["outside_document_field_samples"] += int(
                    abs(true_x) > 4.5 or abs(true_z) > 3.0)
                occupancy["max_document_role_penetration_m"] = max(
                    occupancy["max_document_role_penetration_m"], document_depth)
            if None not in (true_x, true_z, observed_x, observed_z):
                location_errors[name].append(
                    math.hypot(observed_x - true_x, observed_z - true_z)
                )

            vx = number(row, f"{name}_vx")
            vz = number(row, f"{name}_vz")
            omega = number(row, f"{name}_omega_y")
            if vx is not None and vz is not None:
                robot_speeds[name].append(math.hypot(vx, vz))
            if omega is not None:
                robot_turn_rates[name].append(abs(omega))

            try:
                sequence = int(row.get(f"{name}_talk_seq", "-1"))
            except (TypeError, ValueError):
                sequence = -1
            if sequence <= previous_talk_sequence[name]:
                continue
            previous_talk_sequence[name] = sequence
            fields = parse_talk(row.get(f"{name}_talk", ""))
            if fields and number(row, "state") == 2:
                counts = coverage[name]
                counts["play_talk_messages"] += 1
                bx, bz, kx, kz, aim, map_age = (
                    number(fields, key) for key in
                    ("bx", "bz", "kx", "kz", "aim_yaw", "bmap_age"))
                px, pz, tx, tz, yaw, pose_age = (
                    number(fields, key) for key in ("px", "pz", "tx", "tz", "pyaw", "pose_age"))
                arrived = (fields.get("healthy") == "1" and
                           fields.get("tac") in {"DEFEND", "SUPPORT"} and
                           None not in (px, pz, tx, tz, yaw, pose_age, bx, bz, map_age) and
                           0 <= pose_age <= 2 and 0 <= map_age <= .65 and
                           math.hypot(px - tx, pz - tz) < .12 and
                           math.hypot(px - bx, pz - bz) > .20)
                if arrived:
                    counts["arrived_nonowner_messages"] += 1
                    desired = math.degrees(math.atan2(pz - bz, bx - px))
                    arrived_view_angles[name].append(abs((yaw - desired + 180) % 360 - 180))
                    head_yaw = number(fields, "hy")
                    counts["arrived_nonowner_head_saturated_messages"] += int(
                        head_yaw is not None and abs(head_yaw) >= 50)
                if (None not in (bx, bz, kx, kz, aim, map_age) and
                        0 <= map_age <= .65 and math.hypot(kx - bx, kz - bz) > 1e-6):
                    policy_yaw = math.degrees(math.atan2(bz - kz, kx - bx))
                    heading_residuals[name].append(abs((aim - policy_yaw + 180) % 360 - 180))
                    offset = number(fields, "aim_offset")
                    if offset is not None:
                        heading_offsets[name].append(abs(offset))
                ball_age, frame_age = number(fields, "age"), number(fields, "fa")
                if (fields.get("ball") == "1" and None not in (ball_age, frame_age)
                        and 0 <= ball_age <= .25 and 0 <= frame_age <= .25):
                    counts["fresh_ball_messages"] += 1
                count = number(fields, "robots")
                if count is None or not 0 <= count <= 4 or int(count) != count:
                    counts["robot_fields_missing_messages"] += 1
                else:
                    markers = 0
                    missing = False
                    for i in range(int(count)):
                        prefix = f"r{i}"
                        missing |= any(prefix + key not in fields
                                       for key in ("source", "x", "z", "age", "conf"))
                        values = [number(fields, prefix + key)
                                  for key in ("x", "z", "age", "conf")]
                        if (fields.get(prefix + "source") == "number_square"
                                and None not in values and 0 <= values[2] <= .65
                                and 0 < values[3] <= 1):
                            markers += 1
                    counts["robot_fields_missing_messages"] += int(missing)
                    counts["number_square_messages"] += int(markers > 0)
                    counts["number_square_candidates"] += markers
                    counts["arrived_nonowner_number_square_messages"] += int(
                        arrived and markers > 0)
            for key, values in sensor_ages[name].items():
                value = number(fields, key)
                if value is not None and 0.0 <= value < 99.0:
                    values.append(value)

            truth_ball_x = number(row, "ball_x")
            truth_ball_z = number(row, "ball_z")
            if None not in (true_x, true_z, truth_ball_x, truth_ball_z):
                true_range = math.hypot(
                    truth_ball_x - true_x, truth_ball_z - true_z
                )
                band_name = next(
                    band for band, low, high in RANGE_BANDS
                    if low <= true_range < high
                )
                detection_age = number(fields, "age")
                image_age = number(fields, "fa")
                estimated_range = number(fields, "bdist")
                if (fields.get("ball") == "1" and detection_age is not None and
                        0.0 <= detection_age <= 0.25 and image_age is not None and
                        0.0 <= image_age <= 0.25 and estimated_range is not None and
                        0.0 <= estimated_range < 8.0):
                    error = abs(estimated_range - true_range)
                    range_errors[name][band_name].append(error)
                    source = fields.get("ball_range_source", "unknown")
                    if source not in RANGE_SOURCES:
                        source = "unknown"
                    source_range_errors[name][source][band_name].append(error)
                    dx, dz = number(fields, "lgdx"), number(fields, "lgdz")
                    local_age = number(fields, "lg_age")
                    if (source == "ground_ray" and None not in (dx, dz, local_age) and
                            0.0 <= local_age <= .25 and math.hypot(dx, dz) <= 8.0):
                        local_xy_errors[name][band_name].append(math.hypot(
                            dx - (truth_ball_x - true_x), dz - (truth_ball_z - true_z)))

                map_age = number(fields, "bmap_age")
                map_x = number(fields, "bx")
                map_z = number(fields, "bz")
                if (fields.get("ball") == "1" and map_age is not None and
                        0.0 <= map_age <= 0.25 and None not in (map_x, map_z)):
                    ranged_map_errors[name][band_name].append(
                        math.hypot(map_x - truth_ball_x, map_z - truth_ball_z)
                    )

            pose_age = number(fields, "pose_age")
            pose_x = number(fields, "px")
            pose_z = number(fields, "pz")
            if (pose_age is not None and 0.0 <= pose_age <= 0.25 and
                    None not in (pose_x, pose_z, true_x, true_z)):
                pose_errors[name].append(
                    math.hypot(pose_x - true_x, pose_z - true_z)
                )

            ball_age = number(fields, "bmap_age")
            ball_x = number(fields, "bx")
            ball_z = number(fields, "bz")
            truth_x = number(row, "ball_x")
            truth_z = number(row, "ball_z")
            if (fields.get("ball") == "1" and ball_age is not None and
                    0.0 <= ball_age <= 0.25 and
                    None not in (ball_x, ball_z, truth_x, truth_z)):
                ball_errors[name].append(
                    math.hypot(ball_x - truth_x, ball_z - truth_z)
                )

        for team in ("red", "blue"):
            pair = (f"{team}_1", f"{team}_2")
            observations = []
            sequences = []
            for name in pair:
                try:
                    sequences.append(int(row[f"{name}_talk_seq"]))
                except (KeyError, TypeError, ValueError):
                    sequences.append(-1)
                fields = parse_talk(row.get(f"{name}_talk", ""))
                age = number(fields, "bmap_age")
                x = number(fields, "bx")
                z = number(fields, "bz")
                if (fields.get("ball") != "1" or age is None or not 0.0 <= age <= 0.25 or
                        x is None or z is None):
                    observations.append(None)
                else:
                    observations.append((x, z, age))
            sequence_pair = tuple(sequences)
            if (sequence_pair == previous_pair_sequence[team] or
                    None in observations or
                    abs(observations[0][2] - observations[1][2]) > 0.10):
                continue
            previous_pair_sequence[team] = sequence_pair
            truth_x = number(row, "ball_x")
            truth_z = number(row, "ball_z")
            if truth_x is None or truth_z is None:
                continue
            first, second = observations
            first_error = math.hypot(first[0] - truth_x, first[1] - truth_z)
            second_error = math.hypot(second[0] - truth_x, second[1] - truth_z)
            fused_error = math.hypot(
                0.5 * (first[0] + second[0]) - truth_x,
                0.5 * (first[1] + second[1]) - truth_z,
            )
            pair_single_errors[team].append(0.5 * (first_error + second_error))
            pair_fusion_errors[team].append(fused_error)

    robots = {}
    for name in ROBOTS:
        robots[name] = {
            "belief_coverage": coverage[name],
            "arrived_nonowner_mapped_body_ball_angle_deg": distribution(arrived_view_angles[name]),
            # Filter lag and plan changes contribute; not measured body yaw
            # error, a physical shot trajectory, or an automatic failure gate.
            "controller_vs_policy_heading_abs_deg": distribution(heading_residuals[name]),
            "legacy_heading_offset_abs_deg": distribution(heading_offsets[name]),
            "role_region_occupancy": role_occupancy[name],
            "reported_location_error_m": distribution(location_errors[name]),
            "talk_pose_error_m": distribution(pose_errors[name]),
            "talk_ball_error_m": distribution(ball_errors[name]),
            "camera_range_abs_error_by_true_range_m": {
                band: distribution(errors)
                for band, errors in range_errors[name].items()
            },
            "camera_range_abs_error_by_source_and_true_range_m": {
                source: {band: distribution(errors) for band, errors in bands.items()}
                for source, bands in source_range_errors[name].items()
            },
            "local_ground_xy_error_by_true_range_m": {
                band: distribution(errors) for band, errors in local_xy_errors[name].items()
            },
            "talk_ball_error_by_true_range_m": {
                band: distribution(errors)
                for band, errors in ranged_map_errors[name].items()
            },
            "observation_age_s": {
                key: distribution(values)
                for key, values in sensor_ages[name].items()
            },
            "speed_mps": distribution(robot_speeds[name]),
            "absolute_turn_rate_radps": distribution(robot_turn_rates[name]),
        }
    return {
        "rows": len(rows),
        "complete_rows": len(complete_rows),
        "incomplete_rows": len(rows) - len(complete_rows),
        "robots": robots,
        "ball_speed_mps": distribution(ball_speeds),
        "paired_ball_fusion": {
            team: {
                "individual_mean_error_m": distribution(
                    pair_single_errors[team]
                ),
                "midpoint_fused_error_m": distribution(
                    pair_fusion_errors[team]
                ),
            }
            for team in ("red", "blue")
        },
    }


def format_distribution(label: str, values: dict[str, float | int]) -> str:
    if values["n"] == 0:
        return f"  {label}: n=0"
    return (
        f"  {label}: n={values['n']} mean={values['mean']:.3f}"
        f" median={values['median']:.3f} p90={values['p90']:.3f}"
        f" p95={values['p95']:.3f} max={values['max']:.3f}"
    )


def print_report(report: dict[str, object]) -> None:
    print(
        f"trace_rows={report['rows']} complete={report['complete_rows']}"
        f" incomplete_dropped={report['incomplete_rows']}"
    )
    print(
        "Position errors (m); same-frame truth comparison, "
        "not a localization guarantee:"
    )
    for name, metrics in report["robots"].items():
        print(name)
        print("  PLAY unique Talk belief coverage (not detection PR): "
              f"{metrics['belief_coverage']}")
        print(format_distribution("controller_vs_policy_heading_abs_deg",
                                  metrics["controller_vs_policy_heading_abs_deg"]))
        print(format_distribution("legacy_heading_offset_abs_deg",
                                  metrics["legacy_heading_offset_abs_deg"]))
        print(format_distribution("arrived_nonowner_mapped_body_ball_angle_deg",
                                  metrics["arrived_nonowner_mapped_body_ball_angle_deg"]))
        print("  PLAY geometric occupancy (not referee verdict): "
              f"{metrics['role_region_occupancy']}")
        print(format_distribution(
            "published_location_error", metrics["reported_location_error_m"]
        ))
        print(format_distribution(
            "Talk_pose_error_age_le_0.25s", metrics["talk_pose_error_m"]
        ))
        print(format_distribution(
            "Talk_ball_error_age_le_0.25s", metrics["talk_ball_error_m"]
        ))
        print(
            "  Fresh camera range proxy absolute error "
            "by Webots true range (m):"
        )
        for band, values in metrics[
                "camera_range_abs_error_by_true_range_m"].items():
            print(format_distribution(f"    {band}", values))
        print("  Local range source split; unique Talk messages, not independent exposures:")
        for source, bands in metrics[
                "camera_range_abs_error_by_source_and_true_range_m"].items():
            for band, values in bands.items():
                print(format_distribution(f"    {source} {band}", values))
        print("  Runtime local ground XY; relative field axes, receipt-time proxy (m):")
        for band, values in metrics["local_ground_xy_error_by_true_range_m"].items():
            print(format_distribution(f"    {band}", values))
        print("  Fresh Talk ball-map error by Webots true range (m):")
        for band, values in metrics["talk_ball_error_by_true_range_m"].items():
            print(format_distribution(f"    {band}", values))
        print("  Image-to-sensor observation ages (s):")
        for sensor, values in metrics["observation_age_s"].items():
            print(format_distribution(f"    {sensor}", values))
        print(format_distribution("robot_speed_mps", metrics["speed_mps"]))
        print(format_distribution(
            "abs_turn_rate_radps", metrics["absolute_turn_rate_radps"]
        ))
    print(format_distribution("ball_speed_mps", report["ball_speed_mps"]))
    print(
        "Paired estimates (distinct, fresh messages; "
        "ages differ by <=0.10s):"
    )
    for team, metrics in report["paired_ball_fusion"].items():
        print(team)
        print(format_distribution(
            "individual_mean_error_m", metrics["individual_mean_error_m"]
        ))
        print(format_distribution(
            "midpoint_fused_error_m", metrics["midpoint_fused_error_m"]
        ))
    print(
        "Use these traces to calibrate ranges; peaks in an uncontrolled match "
        "are not physical limits."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    try:
        with args.trace.open(newline="", encoding="utf-8") as trace_file:
            report = analyze_rows(list(csv.DictReader(trace_file)))
    except OSError as error:
        print(f"cannot read trace: {error}", file=sys.stderr)
        return 2
    if report["rows"] == 0:
        print("trace contains no samples", file=sys.stderr)
        return 2
    print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
