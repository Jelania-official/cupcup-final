#!/usr/bin/env python3
"""Score controlled Webots trials; truth is used only by this offline evaluator."""

import argparse
import csv
import hashlib
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def distribution(values):
    values = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not values:
        return {"n": 0}

    def percentile(p):
        index = p * (len(values) - 1)
        lo = int(index)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (index - lo)
    return {"n": len(values), "min": values[0], "median": percentile(.5),
            "mean": statistics.mean(values), "p95": percentile(.95), "max": values[-1]}


def read_csv(path):
    with path.open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if any(None in row or any(v is None for v in row.values()) for row in rows):
        raise ValueError(f"incomplete CSV rows in {path}; run must finish before scoring")
    return rows


def number(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def wrap(angle):
    return (angle + 180) % 360 - 180


def score_motion(rows, attack):
    act = [r for r in rows if r["phase"] == "ACT"]
    stable = [r for r in act if number(r, "phase_time") >= 2.0]
    if len(stable) < 2 or number(stable[-1], "time") - number(stable[0], "time") < 4:
        raise ValueError("motion trial lacks a four-second stable window")
    duration = number(stable[-1], "time") - number(stable[0], "time")
    dx = number(stable[-1], "robot_x") - number(stable[0], "robot_x")
    dz = number(stable[-1], "robot_z") - number(stable[0], "robot_z")
    turn = sum(wrap(number(b, "yaw_deg") - number(a, "yaw_deg"))
               for a, b in zip(stable, stable[1:]))
    fallen = any(number(r, "fall") != 0 or number(r, "robot_y") < .25 for r in act)
    # One net-displacement speed per trial, not torso-sway peak treated as locomotion.
    return {"duration_s": duration, "net_speed_m_s": math.hypot(dx, dz) / duration,
            "forward_speed_m_s": attack * dx / duration,
            "lateral_speed_m_s": -attack * dz / duration,
            "net_turn_deg_s": turn / duration, "fall": fallen}


def score_kick(summary, rows):
    forward, left = number(summary, "final_forward"), number(summary, "final_left")
    angle = math.degrees(math.atan2(left, forward))
    fallen = (number(summary, "fall") != 0 or
              any(number(r, "robot_y") < .25 for r in rows
                  if r["phase"] in ("ACT", "MEASURE")))
    distance = number(summary, "max_ball_displacement")
    clean = number(summary, "pre_drift") <= .03
    active = [r for r in rows if r["phase"] in ("ACT", "MEASURE")]
    onset = None
    if active:
        start = active[0]
        for r in active:
            if math.hypot(number(r, "ball_x") - number(start, "ball_x"),
                          number(r, "ball_z") - number(start, "ball_z")) >= .03:
                onset = number(r, "time") - number(start, "time")
                break
    return {"distance_m": distance, "forward_m": forward, "left_m": left,
            "direction_deg": angle, "peak_speed_m_s": number(summary, "max_ball_speed"),
            "pre_drift_m": number(summary, "pre_drift"), "fall": fallen,
            "contact": distance >= .25, "clean_start": clean,
            "strict_success": distance >= .25 and forward > 0 and abs(angle) <= 20
            and not fallen and clean, "command_to_ball_movement_s": onset}


def score_timing(summary, rows):
    """Score timing using physical touches, not the old movement proxy."""
    active = [row for row in rows if row["phase"] in ("ACT", "MEASURE")]
    if len(active) < 2 or number(active[-1], "time") - number(active[0], "time") < 6.4:
        raise ValueError("timing trial lacks its complete observation window")
    times = [number(row, "time") for row in rows]
    if any(not 0 < b - a <= .101 for a, b in zip(times, times[1:])):
        raise ValueError("timing trial has nonmonotonic or missing samples")
    touches = [row for row in active if number(row, "ball_contact_count") > 0]
    if len(touches) != int(number(summary, "contact_frames")):
        raise ValueError("contact summary does not match trace")
    request = [row for row in active if number(row, "body_type") == 2]
    if not request:
        raise ValueError("timing trial has no ACT publication")
    first = request[0]
    stopped = next((row for row in active if number(row, "time") > number(first, "time")
                    and number(row, "body_type") != 2), None)
    if stopped is None:
        raise ValueError("unbounded ACT publication")
    result = score_kick(summary, rows)
    result["movement_proxy"] = result.pop("contact")
    result["sampled_contact"] = bool(touches)
    result["clean_forward_motion"] = result.pop("strict_success") and bool(touches)
    result["contact_frames"] = len(touches)
    result["first_contact_delay_s"] = (number(touches[0], "time") - number(first, "time")
                                       if touches else None)
    result["published_pulse_sim_s"] = number(stopped, "time") - number(first, "time")
    if abs(result["published_pulse_sim_s"] - number(summary, "pulse")) > .041:
        raise ValueError("published pulse differs from declared timing case")
    previous = [row for row in rows if number(row, "time") < number(first, "time")]
    result["pre_request_body_speed_m_s"] = (
        math.hypot(number(previous[-1], "vx"), number(previous[-1], "vz"))
        if previous else None)
    result["initial_ball_speed_m_s"] = math.hypot(
        number(first, "ball_vx"), number(first, "ball_vz"))
    return result


def target_evidence(reference, targets, action, start, end):
    """
    Match original-action target signatures, excluding other actions/ready.

    Evidence of targets received by the normal controller, NOT measured joints,
    physical completion or a new action acknowledgement interface.
    """
    fields = tuple(key for key in reference[0] if key not in ("action", "index"))

    def signature(row):
        return tuple(round(number(row, key), 6) for key in fields)

    common = {signature(row) for row in reference if row["action"] != action}
    unique = {signature(row): int(row["index"]) for row in reference
              if row["action"] == action and signature(row) not in common}
    window = [row for row in targets if start <= number(row, "time") <= end]
    times = [number(row, "time") for row in window]
    complete = bool(times) and times[0] - start <= .101 and end - times[-1] <= .101
    complete &= all(0 < b - a <= .101 for a, b in zip(times, times[1:]))
    matches = [(number(row, "time"), unique[signature(row)]) for row in window
               if signature(row) in unique]
    distinct = len({index for _, index in matches})
    episodes = int(bool(matches)) + sum(b[1] < a[1] - 5 for a, b in zip(matches, matches[1:]))
    observed = True if distinct >= 3 else (False if complete else None)
    return {"action_target_sequence_observed": observed, "target_history_complete": complete,
            "distinct_action_target_frames": distinct, "matching_target_samples": len(matches),
            "first_action_target_delay_s": matches[0][0] - start if matches else None,
            "target_sequence_episodes": episodes,
            "target_index_range": [min(i for _, i in matches), max(i for _, i in matches)]
            if matches else None}


def score_blocking(summary, rows):
    result = score_timing(summary, rows)
    active = [r for r in rows if r["phase"] in ("ACT", "MEASURE")]
    contacts = [r for r in active if number(r, "opponent_contact_count") > 0]
    if len(contacts) != int(number(summary, "opponent_contact_frames")):
        raise ValueError("opponent contact summary does not match trace")
    first = active[0]
    dx = number(first, "opponent_x") - number(first, "ball_x")
    dz = number(first, "opponent_z") - number(first, "ball_z")
    yaw = math.radians(number(summary, "act_yaw_deg"))
    result.update({
        "blocker_present": number(summary, "blocker_forward") >= 0,
        "actual_blocker_forward_m": math.cos(yaw) * dx - math.sin(yaw) * dz,
        "actual_blocker_left_m": -math.sin(yaw) * dx - math.cos(yaw) * dz,
        "sampled_opponent_contact": bool(contacts),
        "opponent_contact_frames": len(contacts),
        "first_opponent_contact_radius_m": (math.hypot(
            number(contacts[0], "opponent_x") - number(contacts[0], "ball_x"),
            number(contacts[0], "opponent_z") - number(contacts[0], "ball_z"))
            if contacts else None),
        "first_opponent_contact_delay_s": (
            number(contacts[0], "time") - number(first, "time")) if contacts else None,
        "opponent_fall": number(summary, "opponent_fall") != 0 or any(
            number(r, "opponent_fall") != 0 or number(r, "opponent_y") < .25 for r in active),
        "opponent_root_drift_m": max(math.hypot(
            number(r, "opponent_x") - number(first, "opponent_x"),
            number(r, "opponent_z") - number(first, "opponent_z")) for r in active),
    })
    return result


def analyze_timing(directories, blocking=False):
    sources, groups = [], defaultdict(list)
    for directory in directories:
        manifest = json.loads((directory / "manifest.json").read_text())
        result = json.loads((directory / "result.json").read_text())
        profile = "kick-blocking" if blocking else "kick-timing"
        if (manifest["arguments"].get("profile") != profile or not result["complete"]
                or result.get("changed_artifacts")):
            raise ValueError(f"not a completed immutable timing experiment: {directory}")
        summaries = read_csv(directory / "trials.csv")
        trace = read_csv(directory / "motion_trace.csv")
        expected = int(manifest["arguments"]["repeats"]) * 8
        if (len(summaries) != expected
                or [int(row["trial"]) for row in summaries] != list(range(expected))):
            raise ValueError("missing, duplicated or unfinished timing trials")
        grouped = defaultdict(list)
        for row in trace:
            grouped[int(row["trial"])].append(row)
        reference = read_csv(directory / "action_templates.csv") if (
            directory / "action_templates.csv").is_file() else []
        targets = read_csv(directory / "motor_targets.csv") if (
            directory / "motor_targets.csv").is_file() else []
        if reference and manifest["arguments"].get("controller_adapter") and not targets:
            raise ValueError("requested natural motor-target recording is missing")
        hashes = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                  for name in ("trials.csv", "motion_trace.csv", "action_templates.csv",
                               "motor_targets.csv") if (directory / name).is_file()}
        sources.append({"directory": str(directory), "manifest": manifest, "csv_sha256": hashes})
        for row in summaries:
            fields = ("forward", "left", "warmup_step", "stop_wait", "pulse")
            if blocking:
                fields += ("blocker_forward", "blocker_left")
            key = ":".join([row["name"]] + [f"{number(row, field):g}" for field in fields])
            rows = grouped[int(row["trial"])]
            evidence = {"action_target_sequence_observed": None}
            if reference and targets:
                requested = [r for r in rows if number(r, "body_type") == 2]
                evidence = target_evidence(reference, targets, row["name"],
                                           number(requested[0], "time"), number(rows[-1], "time"))
            scored = score_blocking(row, rows) if blocking else score_timing(row, rows)
            sequence = evidence["action_target_sequence_observed"]
            scored["action_contact_forward_observed"] = (
                scored["clean_forward_motion"] and sequence if sequence is not None else None)
            groups[key].append({"source": str(directory), "color": manifest["arguments"]["color"],
                                "repeat": int(row["repeat"]),
                                **scored, **evidence})
    return {"schema": "cupcup-kick-blocking-v1" if blocking else "cupcup-kick-timing-v1",
            "sources": sources,
            "analyzer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "group_key": "action:forward:left:warmup_step:stop_wait:pulse" + (
                ":blocker_forward:blocker_left" if blocking else ""),
            "limitations": ["test controller scheduling differs when adapter explicitly requested",
                            "passive opponent placed at ACT, not an active duel" if blocking else
                            "unobstructed timing experiment",
                            "ball placed at ACT to isolate gait history from ball geometry",
                            "sampled contact can miss short touches; not action acknowledgement",
                            "action signature is commanded angles, not measured joint motion",
                            "forward contact movement includes walking pushes, not just kicks",
                            "small repeated cases, not match success probability"],
            "groups": {key: {"trials": trials,
                             "sampled_contacts": sum(t["sampled_contact"] for t in trials),
                             "clean_forward_motions": sum(
                                 t["clean_forward_motion"] for t in trials),
                             "action_contact_forward_observed": sum(
                                 t["action_contact_forward_observed"] is True for t in trials),
                             "action_target_evidence_trials": sum(
                                 t["action_target_sequence_observed"] is not None for t in trials),
                             "falls": sum(t["fall"] for t in trials),
                             **({"sampled_opponent_contacts": sum(
                                 t["sampled_opponent_contact"] for t in trials),
                                 "opponent_falls": sum(t["opponent_fall"] for t in trials)}
                                if blocking else {})}
                       for key, trials in groups.items()}}


def rolling_series(rows, attack):
    active = [r for r in rows if r["phase"] in ("ACT", "MEASURE")]
    if not active:
        raise ValueError("rolling trial has no active samples")
    origin = number(active[0], "time")
    # Drop the injection/contact transient, retain position as well as speed.
    active = [r for r in active if .2 <= number(r, "time") - origin <= 5.0]
    if len(active) < 20:
        raise ValueError("rolling trial too short")
    start = active[0]
    series = []
    for row in active:
        velocity = attack * number(row, "ball_vx")
        if velocity < -.02 or abs(number(row, "ball_vz")) > .12:
            raise ValueError("rolling trial has lateral/contact contamination")
        series.append((number(row, "time") - number(start, "time"),
                       max(0.0, velocity),
                       attack * (number(row, "ball_x") - number(start, "ball_x"))))
    return series


def prediction(model, rate, initial_speed, elapsed):
    if model == "exponential":
        attenuation = math.exp(-rate * elapsed)
        return initial_speed * attenuation, initial_speed * (1 - attenuation) / rate
    stop = min(elapsed, initial_speed / rate)
    return max(0.0, initial_speed - rate * elapsed), initial_speed * stop - .5 * rate * stop * stop


def model_error(model, rate, series):
    errors = []
    for samples in series:
        speed = samples[0][1]
        # Average within each trial first: a longer trial must not dominate fitting.
        errors.append(statistics.mean((prediction(model, rate, speed, t)[0] - v) ** 2
                                      for t, v, _ in samples))
    return statistics.mean(errors)


def fit_rate(model, series):
    if not series:
        raise ValueError("no training rolling trials")
    low, high = .005, 5.0
    for _ in range(65):
        a, b = low + (high - low) / 3, high - (high - low) / 3
        if model_error(model, a, series) < model_error(model, b, series):
            high = b
        else:
            low = a
    return (low + high) / 2


def validate_model(model, rate, trials):
    results = []
    for trial in trials:
        series = trial["series"]
        initial = series[0][1]
        results.append({"source": trial["source"], "repeat": trial["repeat"],
                        "command_m_s": trial["command"],
                        "speed_rmse_m_s": math.sqrt(statistics.mean(
                            (prediction(model, rate, initial, t)[0] - v) ** 2
                            for t, v, _ in series)),
                        "position_rmse_m": math.sqrt(statistics.mean(
                            (prediction(model, rate, initial, t)[1] - x) ** 2
                            for t, _, x in series))})
    return {"trials": results,
            "speed_rmse_m_s": distribution(r["speed_rmse_m_s"] for r in results),
            "position_rmse_m": distribution(r["position_rmse_m"] for r in results)}


def analyze(directories):
    sources, motion, kicks, rolling = [], defaultdict(list), defaultdict(list), []
    for directory in directories:
        manifest = json.loads((directory / "manifest.json").read_text())
        if manifest["arguments"].get("profile", "calibration") != "calibration":
            raise ValueError("only the calibration profile may fit nominal motion parameters")
        color = manifest["arguments"]["color"]
        attack = 1 if color == "blue" else -1
        summaries = read_csv(directory / "trials.csv")
        trace = read_csv(directory / "motion_trace.csv")
        if len(summaries) != int(manifest["arguments"]["repeats"]) * 8:
            raise ValueError(f"unfinished experiment {directory}")
        if sorted(int(row["trial"]) for row in summaries) != list(range(len(summaries))):
            raise ValueError(f"missing/duplicate trial IDs in {directory}")
        grouped = defaultdict(list)
        for row in trace:
            grouped[int(row["trial"])].append(row)
        sources.append({"directory": str(directory), "color": color,
                        "manifest": manifest,
                        "csv_sha256": {name: hashlib.sha256(
                            (directory / name).read_bytes()).hexdigest()
                                       for name in ("trials.csv", "motion_trace.csv")}})
        for summary in summaries:
            rows = grouped[int(summary["trial"])]
            identity = {"source": str(directory), "color": color, "repeat": int(summary["repeat"])}
            name = summary["name"]
            if name in ("walk", "turn"):
                key = f"{name}:{number(summary, 'command'):g}"
                motion[key].append(identity | score_motion(rows, attack))
            elif name.endswith("kick"):
                key = f"{name}:{number(summary, 'forward'):g}:{number(summary, 'left'):g}"
                kicks[key].append(identity | score_kick(summary, rows))
            elif name == "roll":
                rolling.append(identity | {"command": number(summary, "command"),
                                           "series": rolling_series(rows, attack)})
    # Fit only earliest source's first two repeats; later repeat and all other
    # color/source runs are held out. Never refit on the validation set.
    first = str(directories[0])
    training = [t for t in rolling if t["source"] == first and t["repeat"] < 2]
    heldout = [t for t in rolling if t not in training]
    if not heldout:
        raise ValueError("need held-out rolling trials (third repeat or another source)")
    models = {}
    for model in ("exponential", "constant_deceleration"):
        rate = fit_rate(model, [t["series"] for t in training])
        models[model] = {"rate": rate, "rate_unit": "1/s" if model == "exponential" else "m/s^2",
                         "training": validate_model(model, rate, training),
                         "heldout": validate_model(model, rate, heldout)}
    chosen = min(models, key=lambda name: models[name]["heldout"]["position_rmse_m"]["mean"])
    report = {
            "schema": "cupcup-controlled-calibration-v1", "sources": sources,
            "limitations": ["small repeated samples, not population confidence intervals",
                            "legacy contacts are displacement proxies, not physical touches",
                            "body-turn command 10 degrees only; not maximum turn capability",
                            "kick onset omits approach/alignment/stabilization time",
                            "acceleration, latency, collision and occlusion uncalibrated",
                            "bounded waits change scheduling, not sensor or physics sources"],
            "motion": {key: {"trials": values,
                             "stable_net_speed_m_s": distribution(
                                 v["net_speed_m_s"] for v in values),
                             "stable_forward_m_s": distribution(
                                 v["forward_speed_m_s"] for v in values),
                             "stable_turn_deg_s": distribution(
                                 v["net_turn_deg_s"] for v in values),
                             "falls": sum(v["fall"] for v in values)}
                       for key, values in motion.items()},
            "kicks": {key: {"trials": values, "contacts": sum(v["contact"] for v in values),
                            "strict_successes": sum(v["strict_success"] for v in values),
                            "falls": sum(v["fall"] for v in values),
                            **{field: distribution(v[field] for v in values
                                                   if v[field] is not None)
                               for field in ("distance_m", "direction_deg", "peak_speed_m_s",
                                             "pre_drift_m", "command_to_ball_movement_s")}}
                      for key, values in kicks.items()},
            "rolling": {"training_trials": len(training), "heldout_trials": len(heldout),
                        "models": models, "preferred_model_on_heldout": chosen,
                        "selection_is_not_an_independent_final_test": True}}
    report["nominal_sandbox_parameters"] = (
        sandbox_parameters(report)
        if chosen == "exponential" and not any(g["falls"] for g in report["motion"].values())
        else {})
    return report


def sandbox_parameters(report):
    """Nominal measured inputs, not proof of calibrated complete simulator physics."""
    if report["schema"] != "cupcup-controlled-calibration-v1":
        raise ValueError("unsupported calibration schema")
    rolling = report["rolling"]
    if rolling["preferred_model_on_heldout"] != "exponential":
        raise ValueError("sandbox only supports exponential decay; do not silently approximate")
    if any(group["falls"] for group in report["motion"].values()):
        raise ValueError("inspect fallen motion samples before nominal capability selection")
    walking = report["motion"]["walk:0.05"]["stable_forward_m_s"]["median"]
    turning = statistics.median(abs(t["net_turn_deg_s"])
                                for t in report["motion"]["turn:10"]["trials"])
    kick = report["kicks"]["left_kick:0.18:0.08"]["peak_speed_m_s"]["median"]
    if not all(math.isfinite(v) and v > 0 for v in (walking, turning, kick)):
        raise ValueError("invalid measured capabilities")
    return {"speed": walking, "turn": turning, "kick-speed": kick,
            "ball-decay": rolling["models"]["exponential"]["rate"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        profiles = {json.loads((directory / "manifest.json").read_text())["arguments"].get(
            "profile", "calibration") for directory in args.directories}
        if len(profiles) != 1:
            raise ValueError("do not mix timing and calibration profiles")
        report = (analyze_timing(args.directories, profiles == {"kick-blocking"})
                  if profiles <= {"kick-timing", "kick-blocking"} else analyze(args.directories))
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if report["schema"] in ("cupcup-kick-timing-v1", "cupcup-kick-blocking-v1"):
        fields = ("sampled_contacts", "clean_forward_motions", "action_contact_forward_observed",
                  "action_target_evidence_trials", "falls")
        print(json.dumps({key: {field: group[field] for field in fields}
                          for key, group in report["groups"].items()}, indent=2))
        return
    print(json.dumps({"motion": {k: v["stable_net_speed_m_s"]
                                 for k, v in report["motion"].items()},
                      "kicks": {k: {"contacts": v["contacts"], "strict": v["strict_successes"],
                                    "distance": v["distance_m"], "falls": v["falls"]}
                                for k, v in report["kicks"].items()},
                      "rolling": report["rolling"]}, indent=2))


if __name__ == "__main__":
    main()
