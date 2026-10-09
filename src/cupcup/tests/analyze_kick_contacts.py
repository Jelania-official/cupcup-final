#!/usr/bin/env python3
"""Offline contact evidence, not a strategy input or causal kick-success oracle."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from analyze_world_trace import ROBOTS, distribution, number, parse_talk


def segment_geometry(point, start, end):
    """Return centre distance and unbounded projection; never assume a body radius."""
    dx, dz = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dz * dz
    if length2 <= 1e-12:
        return None
    along = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dz) / length2
    clipped = max(0.0, min(1.0, along))
    return {"centre_distance_m": math.hypot(point[0] - start[0] - clipped * dx,
                                            point[1] - start[1] - clipped * dz),
            "projection_fraction": along}


def request_scene(row, robot):
    """Offline truth comparison only; cannot reconstruct missing peer coordinates."""
    fields = parse_talk(row.get(f"{robot}_talk", ""))
    target = [number(fields, k) for k in ("kx", "kz")]
    mapped_ball = [number(fields, k) for k in ("bx", "bz")]
    ball = [float(row["ball_x"]), float(row["ball_z"])]
    usable_target = None not in target and all(abs(v) <= 6 for v in target)
    enemies = [r for r in ROBOTS if r.split("_")[0] != robot.split("_")[0]]
    opponents = []
    for enemy in enemies:
        position = [float(row[f"{enemy}_true_x"]), float(row[f"{enemy}_true_z"])]
        opponents.append({"robot": enemy, "truth_position": position,
                          "true_ball_range_m": math.dist(position, ball),
                          "truth_ball_to_published_target": segment_geometry(
                              position, ball, target) if usable_target else None})
    tracks, peer_tracks = [], []
    for prefix in [f"r{i}" for i in range(4)] + [f"pobs{i}" for i in range(4)]:
        shared = prefix.startswith("pobs")
        x, z, age = (number(fields, prefix + k) for k in ("x", "z", "age"))
        if x is None or z is None or age is None or not 0 <= age <= .65:
            continue
        (peer_tracks if shared else tracks).append({
                       "team": fields.get(prefix + "team", "unknown"),
                       "source": "teammate_direct_number_square" if shared else
                       fields.get(prefix + "source", "unknown"),
                       "x": x, "z": z, "age_s": age,
                       "confidence": number(fields, prefix + "conf"),
                       "uncertainty": number(fields, prefix + "unc"),
                       "nearest_truth_opponent_m": min(
                           math.dist([x, z], e["truth_position"]) for e in opponents),
                       "published_ball_to_target": segment_geometry(
                           [x, z], mapped_ball, target)
                       if usable_target and None not in mapped_ball else None})
    peer_count = number(fields, "peer_obstacles")
    return {"published_target": target if usable_target else None,
            "published_ball": mapped_ball if None not in mapped_ball else None,
            "published_peer_obstacles": peer_count,
            "published_local_tracks": tracks, "published_peer_tracks": peer_tracks,
            "truth_opponents": opponents}


def observed_preparation(rows, index, robot, request_time):
    """Measure the preceding SETTLE episode from received state, not exposure time."""
    start = None
    for row in reversed(rows[:index + 1]):
        if int(row["state"]) != 2:
            break
        state = parse_talk(row.get(f"{robot}_talk", "")).get("state")
        if state == "SETTLE":
            start = float(row["time"])
        elif state == "KICK" and start is None:
            continue
        else:
            break
    return max(0, request_time - start) if start is not None else None


def preparation_coverage(rows, robot):
    """Count published preparation evidence even when no kick was requested."""
    counts = {"align_messages": 0, "settle_messages": 0,
              "ground_messages": 0, "max_published_preparation_frames": None}
    ranges, seen = [], set()
    for row in rows:
        sequence = row.get(f"{robot}_talk_seq")
        # Without a sequence, repeated CSV rows cannot be independent messages.
        if int(row["state"]) != 2 or sequence is None or sequence in seen:
            continue
        seen.add(sequence)
        fields = parse_talk(row.get(f"{robot}_talk", ""))
        state = fields.get("state")
        if state not in ("ALIGN", "SETTLE"):
            continue
        counts[state.lower() + "_messages"] += 1
        frames = number(fields, "prep_frames")
        if frames is not None and frames >= 0:
            counts["max_published_preparation_frames"] = max(
                counts["max_published_preparation_frames"] or 0, frames)
        dx, dz, age = (number(fields, key) for key in ("lgdx", "lgdz", "lg_age"))
        if None not in (dx, dz, age) and 0 <= age <= .25:
            counts["ground_messages"] += 1
            ranges.append(math.hypot(dx, dz))
    counts["published_ground_range_m"] = distribution(ranges)
    return counts


def read_trace(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    dropped = 0
    # A forced process shutdown can interrupt the last buffered CSV record.
    # Only that boundary record may be dropped; corrupt interior data fails.
    if rows and (None in rows[-1] or any(v is None for v in rows[-1].values())):
        rows.pop()
        dropped = 1
    return rows, dropped


def analyze(rows, window=4.0):
    if not rows or not math.isfinite(window) or window <= 0:
        raise ValueError("nonempty trace and positive finite window required")
    required = ("time", "state", "ball_x", "ball_z", "ball_vx", "ball_vz")
    required += tuple(f"{r}_{suffix}" for r in ROBOTS for suffix in (
        "true_x", "true_z", "yaw", "kick_seq", "kick_time", "kick_name", "ball_contact_count"))
    if any(number(row, key) is None for row in rows for key in required
           if not key.endswith("kick_name")):
        raise ValueError("trace lacks finite contact instrumentation columns")
    times = [float(row["time"]) for row in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("trace simulation time must strictly increase")
    events = []
    previous = {r: 0 for r in ROBOTS}
    for i, row in enumerate(rows):
        for robot in ROBOTS:
            seq = int(row[f"{robot}_kick_seq"])
            if seq <= previous[robot]:
                continue
            skipped = seq - previous[robot] - 1
            previous[robot] = seq
            stamp = float(row[f"{robot}_kick_time"])
            # The callback clock is a receipt clock. Reject missing history or
            # cross-phase receipt instead of inventing a complete window.
            if stamp < times[max(0, i - 1)] - .021 or stamp > times[i] + 1e-6:
                events.append({"robot": robot, "sequence": seq,
                               "censored": "request-history-missing", "skipped_requests": skipped})
                continue
            yaw = float(row[f"{robot}_yaw"])
            bx, bz = float(row["ball_x"]), float(row["ball_z"])
            dx, dz = bx - float(row[f"{robot}_true_x"]), bz - float(row[f"{robot}_true_z"])
            event = {"robot": robot, "sequence": seq, "time": stamp,
                     "request_scene": request_scene(row, robot),
                     "action": row[f"{robot}_kick_name"], "skipped_requests": skipped,
                     "initial_ball_range_m": math.hypot(dx, dz),
                     "initial_ball_forward_m": dx * math.cos(yaw) - dz * math.sin(yaw),
                     "initial_ball_left_m": -dx * math.sin(yaw) - dz * math.cos(yaw),
                     "initial_ball_speed_m_s": math.hypot(
                         float(row["ball_vx"]), float(row["ball_vz"])),
                     "contact_frames": 0, "first_contact_delay_s": None,
                     "command_pulse_sim_s": None,
                     "observed_settle_to_request_s": observed_preparation(rows, i, robot, stamp),
                     "observed_settle_to_contact_s": None,
                     "other_robot_contact": False, "max_displacement_m": 0.0,
                     "max_ball_speed_m_s": 0.0, "censored": None}
            last, prior = row, row
            if int(row["state"]) != 2:
                event["censored"] = "request-outside-PLAY"
            else:
                for sample in rows[i:]:
                    t = float(sample["time"])
                    if t > stamp + window + 1e-6:
                        break
                    if int(sample["state"]) != 2:
                        event["censored"] = "phase-change"
                        break
                    if t - float(prior["time"]) > .1:
                        event["censored"] = "sampling-gap"
                        break
                    sx, sz = float(sample["ball_x"]), float(sample["ball_z"])
                    if math.hypot(sx - float(prior["ball_x"]), sz - float(prior["ball_z"])) > .5:
                        event["censored"] = "ball-reset-or-unmodelled-jump"
                        break
                    if int(sample[f"{robot}_kick_seq"]) != seq:
                        event["censored"] = "next-request"
                        break
                    body_type = number(sample, f"{robot}_body_type")
                    if (event["command_pulse_sim_s"] is None and body_type is not None and
                            (body_type != 2 or sample.get(f"{robot}_actname") != event["action"])):
                        event["command_pulse_sim_s"] = max(0, t - stamp)
                    if int(sample[f"{robot}_ball_contact_count"]) > 0:
                        event["contact_frames"] += 1
                        if event["first_contact_delay_s"] is None:
                            event["first_contact_delay_s"] = max(0, t - stamp)
                    event["other_robot_contact"] |= any(
                        int(sample[f"{other}_ball_contact_count"]) > 0
                        for other in ROBOTS if other != robot)
                    event["max_displacement_m"] = max(
                        event["max_displacement_m"], math.hypot(sx - bx, sz - bz))
                    event["max_ball_speed_m_s"] = max(event["max_ball_speed_m_s"], math.hypot(
                        float(sample["ball_vx"]), float(sample["ball_vz"])))
                    last = prior = sample
                if float(last["time"]) - stamp < window - .021 and event["censored"] is None:
                    event["censored"] = "trace-ended"
            fx, fz = float(last["ball_x"]) - bx, float(last["ball_z"]) - bz
            event["observed_window_s"] = max(0, float(last["time"]) - stamp)
            if (event["observed_settle_to_request_s"] is not None and
                    event["first_contact_delay_s"] is not None):
                event["observed_settle_to_contact_s"] = (
                    event["observed_settle_to_request_s"] + event["first_contact_delay_s"])
            event["final_forward_m"] = fx * math.cos(yaw) - fz * math.sin(yaw)
            event["final_left_m"] = -fx * math.sin(yaw) - fz * math.cos(yaw)
            event["forward_contact_motion_observed"] = (
                event["censored"] is None and event["contact_frames"] > 0 and
                not event["other_robot_contact"] and event["initial_ball_speed_m_s"] <= .08 and
                event["final_forward_m"] >= .25 and
                abs(event["final_left_m"]) <=
                math.tan(math.radians(20)) * event["final_forward_m"])
            # Walking/pushing after a command can also move the ball. Even the
            # stricter label above is observed motion, not causal foot success.
            events.append(event)
    aggregate = {}
    for robot in ROBOTS:
        selected = [e for e in events if e["robot"] == robot]
        aggregate[robot] = {
            "preparation_coverage": preparation_coverage(rows, robot),
            "observed_requests": len(selected),
            "uncensored_windows": sum(e.get("censored") is None for e in selected),
            "requests_with_sampled_contact": sum(e.get("contact_frames", 0) > 0 for e in selected),
            "forward_contact_motion_observed": sum(
                e.get("forward_contact_motion_observed", False) for e in selected),
            "request_range_m": distribution([
                e["initial_ball_range_m"] for e in selected if "initial_ball_range_m" in e]),
            "contact_delay_s": distribution([
                e["first_contact_delay_s"] for e in selected
                if e.get("first_contact_delay_s") is not None]),
            "observed_settle_to_contact_s": distribution([
                e["observed_settle_to_contact_s"] for e in selected
                if e.get("observed_settle_to_contact_s") is not None]),
            "all_PLAY_contact_frames": sum(
                int(row["state"]) == 2 and int(row[f"{robot}_ball_contact_count"]) > 0
                for row in rows),
        }
    return {"schema": "cupcup-sampled-contact-evidence-v1", "rows": len(rows),
            "simulation_seconds": times[-1] - times[0], "window_seconds": window,
            "robots": aggregate, "events": events,
            "limits": ["20ms sampling may miss contacts; absence is not proof of no touch",
                       "contact identifies robot body, not a particular foot or action causality",
                       "command time is supervisor receipt, not sensor exposure or motor start",
                       "SETTLE duration is state-receipt approximation, not exact entry time",
                       "preparation coverage counts unique Talk sequences, not camera exposures",
                       "ground availability is published evidence, not every execution gate",
                       "request scene uses latest received Talk, not synchronized strategy inputs",
                       "truth centre distance is not a collision radius or identity match",
                       "old logs lack peer coordinates; count alone cannot replay shared lane",
                       "resets, phase changes, new requests and truncated traces censor scoring"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--window", type=float, default=4)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows, dropped = read_trace(args.trace)
    report = analyze(rows, args.window)
    report["incomplete_trailing_rows_dropped"] = dropped
    report["trace_sha256"] = hashlib.sha256(args.trace.read_bytes()).hexdigest()
    report["analyzer_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    metadata = Path(str(args.trace) + ".meta.json")
    if metadata.is_file():
        report["instrumentation"] = json.loads(metadata.read_text(encoding="utf-8"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["robots"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
