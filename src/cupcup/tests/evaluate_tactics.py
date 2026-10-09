#!/usr/bin/env python3
"""Save paired-color, capability-stratified tactical experiments with held-out seeds."""

import argparse
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path

from analyze_measurement import sandbox_parameters

SUM_METRICS = ("mean_ball_attack_m", "opponent_half_s", "own_box_s",
               "uncontested_ball_access_s", "contested_ball_s", "robot_blocked_s",
               "kick_reach_s", "kick_facing_s", "kick_aim_s", "kick_ready_s",
               "kick_setup_resets")
MAX_METRICS = ("max_kick_preparing_s",)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--source-root", type=Path,
                        help="isolated candidate source snapshot; not a build attestation")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--split", choices=("development", "holdout", "both"), default="both")
    parser.add_argument("--development-seed", type=int, default=101)
    parser.add_argument("--holdout-seed", type=int, default=90001)
    parser.add_argument("--kick-hysteresis", type=float, default=0.25)
    parser.add_argument("--defense-mode", choices=("line", "fixed"), default="line")
    parser.add_argument("--vision-model", choices=("pan", "legacy"), default="pan")
    parser.add_argument("--frames", choices=("final", "full"), default="final",
                        help="full also supports archived binaries without --frames")
    parser.add_argument("--calibration", type=Path,
                        help="measurement report for nominal speed/turn/kick/ball-decay")
    parser.add_argument("--setup", type=float,
                        help="Own near-ball preparation/contact time for model sensitivity tests")
    parser.add_argument("--observation-noise", type=float,
                        help="Provisional relative-vision sigma in metres; not a calibration")
    parser.add_argument("--opponents", nargs="+", default=["press", "block", "keeper", "shared"],
                        choices=("press", "block", "keeper", "shared"))
    parser.add_argument("--scenarios", nargs="+", default=["kickoff", "own-half", "incoming"],
                        choices=("kickoff", "own-half", "incoming"))
    args = parser.parse_args()
    if not 1 <= args.seeds <= 100 or not 1 <= args.duration <= 900:
        parser.error("seeds must be 1..100 and duration 1..900")
    if any(not 0 <= seed <= 4294967295 - args.seeds + 1
           for seed in (args.development_seed, args.holdout_seed)):
        parser.error("seed range must fit unsigned 32-bit")
    if abs(args.development_seed - args.holdout_seed) < args.seeds:
        parser.error("development and holdout seed ranges must not overlap")
    if not math.isfinite(args.kick_hysteresis) or not 0 <= args.kick_hysteresis <= 2:
        parser.error("kick hysteresis must be finite and 0..2")
    if args.setup is not None and (not math.isfinite(args.setup) or not .05 <= args.setup <= 5):
        parser.error("setup must be finite and 0.05..5 (slow opponent uses 1.8 times this)")
    if args.observation_noise is not None and (
            not math.isfinite(args.observation_noise) or not 0 <= args.observation_noise <= 1):
        parser.error("observation noise must be finite and 0..1 metres")
    started = time.monotonic()
    binary_hash = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    source = args.source_root or Path(__file__).resolve().parents[1] / "src"
    source_files = ("match_policy.hpp", "strategy_logic.hpp", "tactical_sim.hpp",
                    "world_model.hpp", "navigation.hpp", "field_geometry.hpp")
    if not all((source / name).is_file() for name in source_files):
        parser.error("source root must contain the complete shared policy/model headers")
    source_snapshot = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                       for name in source_files}
    results = []
    measured = (sandbox_parameters(json.loads(args.calibration.read_text()))
                if args.calibration else {})
    own_speed = measured.get("speed", .4)
    own_setup = args.setup if args.setup is not None else measured.get("setup", 1.0)
    profiles = {"slow": (own_speed * .625, own_setup * 1.8),
                "equal": (own_speed, own_setup), "fast": (own_speed * 1.5, own_setup * .6)}
    splits = (("development", args.development_seed), ("holdout", args.holdout_seed))
    for split, base_seed in splits:
        if args.split != "both" and args.split != split:
            continue
        for mode in ("oracle", "realistic"):
            for opponent in args.opponents:
                for scene in args.scenarios:
                    for profile, (speed, setup) in profiles.items():
                        for seed in range(base_seed, base_seed + args.seeds):
                            pair = []
                            for color in ("red", "blue"):
                                command = [str(args.binary), "--duration", str(args.duration),
                                           "--seed", str(seed), "--mode", mode,
                                           "--opponent", opponent,
                                           "--scenario", scene, "--cupcup-color", color,
                                           "--opponent-speed", str(speed),
                                           "--opponent-setup", str(setup),
                                           "--kick-hysteresis", str(args.kick_hysteresis),
                                           "--defense-mode", args.defense_mode,
                                           "--vision-model", args.vision_model]
                                if args.frames == "final":
                                    command.extend(["--frames", "final"])
                                for name, value in measured.items():
                                    if name == "setup":
                                        continue
                                    command.extend(["--" + name, str(value)])
                                command.extend(["--setup", str(own_setup)])
                                if args.observation_noise is not None:
                                    command.extend(["--noise", str(args.observation_noise)])
                                if measured:
                                    command.extend(["--opponent-turn", str(measured["turn"])])
                                match = json.loads(subprocess.check_output(command, text=True))
                                final = match["frames"][-1]
                                other = "blue" if color == "red" else "red"
                                row = {
                                    "split": split, "mode": mode, "opponent": opponent,
                                    "scenario": scene, "profile": profile,
                                    "seed": seed, "color": color,
                                    "robot_contact_revision": match.get(
                                        "robot_contact_revision", 1),
                                    "ball_contact_revision": match.get("ball_contact_revision", 1),
                                    "observation_execution_revision": match.get(
                                        "observation_execution_revision", 1),
                                    "horizontal_vision_revision": match.get(
                                        "horizontal_vision_revision", 1),
                                    "goals_for": final[color + "_score"],
                                    "goals_against": final[other + "_score"],
                                    "kicks": match["metrics"][color + "_kicks"],
                                    "penalties": match["metrics"][color + "_penalties"],
                                    "collisions": match["metrics"]["collision_steps"],
                                    "restarts": match["metrics"]["restarts"],
                                    "play_seconds": match["metrics"]["play_seconds"],
                                    "reproduce": command,
                                }
                                for metric in SUM_METRICS + MAX_METRICS:
                                    row[metric] = match["metrics"][color + "_" + metric]
                                row["stationary_ball_s"] = match["metrics"]["stationary_ball_s"]
                                pair.append(row)
                                results.append(row)
                            # These are rotated copies, not independent match samples.
                            if any(not math.isclose(pair[0][key], pair[1][key], abs_tol=1e-6)
                                   for key in (
                                    "goals_for", "goals_against", "kicks",
                                    "penalties", "collisions", "mean_ball_attack_m",
                                    "opponent_half_s", "own_box_s", "robot_blocked_s")):
                                raise RuntimeError(f"color symmetry failed: {pair}")
    groups = {}
    for row in results:
        key = "/".join(row[name] for name in ("split", "mode", "opponent", "scenario", "profile"))
        group = groups.setdefault(key, {
            "matches": 0, "wins": 0, "draws": 0, "losses": 0,
            "goals_for": 0, "goals_against": 0, "kicks": 0,
            "penalties": 0, "collisions": 0, "restarts": 0,
            "play_seconds": 0, "stationary_ball_s": 0,
            **{name: 0 for name in SUM_METRICS + MAX_METRICS}})
        group["matches"] += 1
        outcome = "wins" if row["goals_for"] > row["goals_against"] else (
            "losses" if row["goals_for"] < row["goals_against"] else "draws")
        group[outcome] += 1
        for metric in ("goals_for", "goals_against", "kicks",
                       "penalties", "collisions", "restarts", "play_seconds",
                       "stationary_ball_s") + SUM_METRICS:
            group[metric] += row[metric]
        for metric in MAX_METRICS:
            group[metric] = max(group[metric], row[metric])
    for group in groups.values():
        group["mean_ball_attack_m"] /= group["matches"]
    if hashlib.sha256(args.binary.read_bytes()).hexdigest() != binary_hash:
        raise RuntimeError("sandbox executable changed during evaluation; mixed-version evidence")
    report = {
        "model": match["model"], "calibrated": False,
        "abstract_opponent_revision": match["abstract_opponent_revision"],
        "matches": len(results), "elapsed_wall_seconds": time.monotonic() - started,
        "nominal_measured_parameters": measured,
        "own_setup_s": own_setup,
        "observation_sigma_m": match["parameters"]["observation_sigma_m"],
        "observation_sigma_source": "explicit-provisional-override"
        if args.observation_noise is not None else "unmeasured-model-default",
        "setup_source": "explicit-provisional-override" if args.setup is not None else (
            "measurement-report" if "setup" in measured else "unmeasured-model-default"),
        "opponent_setup_ratios": {"slow": 1.8, "equal": 1.0, "fast": .6},
        "calibration_source": str(args.calibration) if args.calibration else None,
        "calibration_sha256": hashlib.sha256(args.calibration.read_bytes()).hexdigest()
        if args.calibration else None,
        "duration_model_seconds": args.duration,
        "requested_split": args.split,
        "seed_ranges": dict(splits),
        "kick_direction_hysteresis": args.kick_hysteresis,
        "defense_mode": args.defense_mode,
        "binary_sha256": binary_hash,
        "source_sha256": source_snapshot,
        "source_root": str(source.resolve()),
        "source_hash_scope": "selected source snapshot, NOT proof of compiled executable sources",
        "source_changed_during_run": any(
            hashlib.sha256((source / name).read_bytes()).hexdigest() != digest
            for name, digest in source_snapshot.items()),
        "caveat": ("Color pairs are correlated rotated copies; "
                   "model parameters remain provisional. "
                   "A holdout seed label does not certify it was never inspected. "
                   "Do not interpret this report as real competition strength."),
        "robot_contact_revisions": sorted({row["robot_contact_revision"] for row in results}),
        "ball_contact_revisions": sorted({row["ball_contact_revision"] for row in results}),
        "observation_execution_revisions": sorted({
            row["observation_execution_revision"] for row in results}),
        "horizontal_vision_revisions": sorted({
            row["horizontal_vision_revision"] for row in results}),
        "groups": groups, "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"matches": len(results), "elapsed_seconds": report["elapsed_wall_seconds"],
                      "goals_for": sum(row["goals_for"] for row in results),
                      "goals_against": sum(row["goals_against"] for row in results),
                      "penalties": sum(row["penalties"] for row in results),
                      "collisions": sum(row["collisions"] for row in results),
                      "report": str(args.output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
