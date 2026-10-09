#!/usr/bin/env python3
"""
Run a repeatable Webots 2v2 regression match and summarize its logs.

This is deliberately a test harness, not part of the submitted runtime.  It
starts the existing field/supervisor stack, waits for a real field heartbeat,
then starts the headless referee and both team strategies.  Each process gets
its own log so a short match and a formal-length match can be compared without
relying on terminal scrollback.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import io
import json
import math
import os
import re
import signal
import shlex
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
INSTALL = Path(os.environ.get("CUPCUP_INSTALL", str(Path.home() / ".cache/cupcup/install")))
LOG_ROOT = Path(os.environ.get(
    "CUPCUP_MATCH_LOG", str(Path.home() / ".local/state/cupcup/matches")))


def seed_provenance(seed: int | None, official: bool) -> dict:
    """Released supervisor seeds from time(NULL), not CUPCUP_SIM_SEED."""
    return {"requested": seed,
            "localization_seed_applied": seed is not None and not official,
            "localization_seed_source": "time(NULL)" if official else "CUPCUP_SIM_SEED-or-time",
            "physics_seed_controlled": False}


def binary_matches(path: Path, expected: str) -> bool:
    """Reject build/launch races; a source snapshot cannot identify the binary."""
    return path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == expected


def command_prefix() -> str:
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {INSTALL / 'setup.bash'} && "
    )


def start_process(command: str, log_path: Path, env: dict[str, str],
                  workdir: Path = ROOT) -> subprocess.Popen:
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=workdir,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    # Keep the file alive through the child process, and let the descriptor be
    # closed by the parent only after the child has inherited it.
    log.close()
    return process


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def find_motion_pid(robot: str, process_group: int) -> int | None:
    """Resolve only this match's exact robot motion process for outage injection."""
    expected = str(INSTALL / "motion/lib/motion/motion").encode()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
            pid = int(entry.name)
            if (len(args) >= 2 and args[0] == expected and args[1] == robot.encode()
                    and os.getpgid(pid) == process_group):
                return pid
        except (OSError, ProcessLookupError):
            continue
    return None


def wait_for_field(log_path: Path, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # The supervisor uses the default ROS reliability profile, while the
        # CLI echo command can negotiate an incompatible QoS on this setup.
        # A short hz probe observes the same real publisher and is allowed to
        # time out after it has already printed a rate.
        probe = subprocess.run(
            ["bash", "-lc", command_prefix() + "timeout 4 ros2 topic hz /sensor/field"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        if "average rate" in (probe.stdout + probe.stderr):
            return True
        time.sleep(1.0)
    return False


def summarize(log_dir: Path, elapsed: float, color: str, opponent: str,
              seed: int | None) -> None:
    files = sorted(log_dir.glob("*.log"))
    joined = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in files)
    events = re.findall(r"field event=(\d+).*?score=(\d+):(\d+)", joined)
    goals = [event for event in events if event[0] == "4"]

    def read_log(name: str) -> str:
        path = log_dir / name
        return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""

    referee_log = read_log("referee.log")
    cupcup_log = read_log("cupcup.log")
    opponent_log = read_log("opponent.log")
    # CLEAR writes both a generic KICK line and an explanatory defender line.
    # Count the state-entry record once, not both text mentions.
    kicks = len(re.findall(r"\b(?:red|blue)_[12] kick foot=", cupcup_log))
    clears = len(re.findall(r"defender clear", cupcup_log))
    falls = len(re.findall(r"fall=[1-9]", cupcup_log))
    opponent_falls = len(re.findall(r"fall=[1-9]", opponent_log))
    waits = len(re.findall(r"waiting:", cupcup_log))
    tactical = re.findall(r"tactical=(HOLD|CHASE|SUPPORT|DEFEND|CLEAR)", cupcup_log)
    states = re.findall(
        r"\b(SEARCH|APPROACH|ORBIT|ALIGN|SETTLE|KICK|VERIFY|RECOVER) tactical=",
        cupcup_log,
    )
    scores = re.findall(r"\bscore=(\d+):(\d+)", joined)
    print(f"color={color} opponent={opponent} seed={seed if seed is not None else 'random'}")
    manifest_path = log_dir / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        # Old manifests did not record this distinction. Unknown is not true.
        print("localization_seed_applied=" + str(
            manifest.get("seed_provenance", {}).get("localization_seed_applied", "unknown")))
    print(f"match_seconds={elapsed:.1f}")
    print("duration_clock=wall_seconds")
    result_path = log_dir / "released_referee.json"
    if result_path.is_file():
        result = json.loads(result_path.read_text(encoding="utf-8"))
        print(f"referee_play_seconds={result['referee_play_seconds']}")
        print(f"referee_end_observed={result['end_observed']}")
        print(f"formal_900_completed={result['formal_900_completed']}")
    trace_path = log_dir / "world_trace.csv"
    if trace_path.exists():
        first_time = last_time = None
        with trace_path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                try:
                    stamp = float(row["time"])
                except (KeyError, ValueError):
                    continue
                if not math.isfinite(stamp):
                    continue
                if first_time is None:
                    first_time = stamp
                last_time = stamp
        if first_time is not None:
            print(f"simulation_seconds={last_time - first_time:.2f}")
    print(f"field_events={len(events)}")
    print(f"goal_events={len(goals)}")
    player_out_teams = re.findall(r"player_out team=(red|blue)", referee_log)
    opponent_color = "blue" if color == "red" else "red"
    print(f"cupcup_player_out_penalties={player_out_teams.count(color)}")
    print(f"opponent_player_out_penalties={player_out_teams.count(opponent_color)}")
    if scores:
        print(f"last_score={scores[-1][0]}:{scores[-1][1]}")
    print(f"kick_transitions={kicks}")
    print(f"defender_clear_actions={clears}")
    tactical_counts = ",".join(
        f"{name}:{tactical.count(name)}"
        for name in ("CHASE", "SUPPORT", "DEFEND", "CLEAR", "HOLD")
    )
    print(f"tactical_samples={tactical_counts}")
    state_counts = ",".join(
        f"{name}:{states.count(name)}"
        for name in ("SEARCH", "APPROACH", "ORBIT", "ALIGN", "SETTLE", "KICK", "VERIFY", "RECOVER")
    )
    print(f"strategy_state_samples={state_counts}")
    print(f"health_wait_logs={waits}")
    print(f"cupcup_fall_logs={falls}")
    print(f"opponent_fall_logs={opponent_falls}")
    if "Traceback" in opponent_log or "ERROR" in opponent_log:
        print("opponent_log_errors=present")
    print(f"logs={log_dir}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", choices=("red", "blue"), default="red")
    parser.add_argument("--duration", type=float, default=180.0,
                        help="wall-clock seconds, NOT simulated match seconds")
    parser.add_argument("--opponent",
                        choices=("unirobot", "cupcup", "rush", "wall", "keeper", "probe"),
                        default="unirobot")
    parser.add_argument(
        "--seed", type=int,
        help="fix experimental localization noise; released supervisor ignores this seed",
    )
    parser.add_argument(
        "--trace", action="store_true",
        help="record simulator truth beside the robot's existing estimates for offline evaluation",
    )
    parser.add_argument(
        "--calibration", action="store_true",
        help="hold robots in READY and sweep ball range/head pose for vision calibration",
    )
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--official-launch-compat", action="store_true",
                        help="test released sources using the matching local Webots library")
    parser.add_argument("--trace-judge",
                        help="instrumented released supervisor; requires official --trace")
    parser.add_argument("--released-referee",
                        help="path to test operator linked against untouched released CtrlWindow")
    parser.add_argument("--referee-play-seconds", type=int, default=900,
                        help="original referee PLAY seconds; --duration is the wall watchdog")
    parser.add_argument("--images", action="store_true",
                        help="save own team's raw camera images at 2 Hz for failure inspection")
    parser.add_argument("--observe", action="store_true",
                        help="record the live belief map and intents without simulator truth")
    parser.add_argument("--no-ball-pattern-filter", action="store_true",
                        help="disable platform-specific ball appearance check for A/B testing")
    parser.add_argument("--kick-settle-frames", type=int, default=16,
                        help="distinct stable frames before kick; 3 reproduces old preparation")
    parser.add_argument("--ball-verifier-model", type=Path,
                        help="optional external patch model for own team only; not bundled")
    parser.add_argument("--motion-outage", type=float, default=0.0,
                        help="pause own forward's motion service after 10 s, then resume")
    args = parser.parse_args()
    if args.ball_verifier_model:
        args.ball_verifier_model = args.ball_verifier_model.resolve()
        if not args.ball_verifier_model.is_file():
            parser.error("--ball-verifier-model must name an existing model file")
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and positive (wall seconds)")
    if args.seed is not None and not 0 <= args.seed <= 4294967295:
        parser.error("--seed must be an unsigned 32-bit integer")
    if not 3 <= args.kick_settle_frames <= 20:
        parser.error("--kick-settle-frames must be 3..20")
    if args.released_referee:
        if not args.official_launch_compat or args.calibration:
            parser.error("--released-referee requires --official-launch-compat and no calibration")
        if not 1 <= args.referee_play_seconds <= 900 or args.duration <= args.referee_play_seconds:
            parser.error("referee PLAY seconds must be 1..900 and less than "
                         "wall watchdog duration")
        binary = Path(args.released_referee).resolve()
        if not binary.is_file() or not os.access(binary, os.X_OK):
            parser.error("released referee executable is missing or not executable")
        args.released_referee = str(binary)
    if not 0.0 <= args.motion_outage <= 10.0:
        parser.error("--motion-outage must be between 0 and 10 seconds")
    if args.motion_outage and (args.calibration or args.duration < 20.0):
        parser.error("--motion-outage needs a normal match of at least 20 seconds")
    if args.calibration and args.opponent != "unirobot":
        parser.error("--calibration requires --opponent unirobot")
    if args.calibration and args.official_launch_compat:
        parser.error("calibration requires the experimental supervisor, not released sources")
    if args.calibration:
        args.trace = True
    if args.images and not args.official_launch_compat:
        args.trace = True
    if args.official_launch_compat and args.trace and not args.trace_judge:
        parser.error("official --trace requires --trace-judge; "
                     "original supervisor has no trace writer")
    if args.trace_judge:
        if not args.official_launch_compat or not args.trace:
            parser.error("--trace-judge requires --official-launch-compat --trace")
        binary = Path(args.trace_judge).resolve()
        if not binary.is_file() or not os.access(binary, os.X_OK):
            parser.error("trace judge executable is missing or not executable")
        args.trace_judge = str(binary)

    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_dir = LOG_ROOT / f"{timestamp}-{args.color}-vs-{args.opponent}"
    log_dir.mkdir(parents=True, exist_ok=True)
    binaries = ("cupcup/lib/cupcup/cupcup", "controller/lib/controller/controller",
                "controller/lib/controller/supervisor", "motion/lib/motion/motion")
    # Hash the installed executables, not just the dirty source tree: a source
    # edit made during a match must not be mistaken for the version it ran.
    manifest = {
        "arguments": {key: str(value) if isinstance(value, Path) else value
                      for key, value in vars(args).items()}, "install": str(INSTALL),
        "started_local": timestamp, "duration_clock": "wall_seconds",
        "seed_provenance": seed_provenance(args.seed, args.official_launch_compat),
        "dds_environment": {name: os.environ.get(name) for name in (
            "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
            "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_BUILTIN_TRANSPORTS",
            "RMW_FASTRTPS_PUBLICATION_MODE", "RMW_FASTRTPS_USE_QOS_FROM_XML")},
        "binary_sha256": {
            path: hashlib.sha256((INSTALL / path).read_bytes()).hexdigest()
            for path in binaries if (INSTALL / path).is_file()
        },
        "configuration_sha256": {
            path: hashlib.sha256((INSTALL / path).read_bytes()).hexdigest()
            for path in ("cupcup/share/cupcup/config/strategy.yaml",
                         "cupcup/share/cupcup/launch/player_launch.py")
            if (INSTALL / path).is_file()
        },
    }
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    if args.ball_verifier_model:
        manifest["ball_verifier_model_sha256"] = hashlib.sha256(
            args.ball_verifier_model.read_bytes()).hexdigest()
    if profile and Path(profile).is_file():
        manifest["dds_profile_sha256"] = hashlib.sha256(Path(profile).read_bytes()).hexdigest()
    if args.released_referee:
        manifest["referee_mode"] = "released-CtrlWindow-automated-operator"
        manifest["released_referee_binary_sha256"] = hashlib.sha256(
            Path(args.released_referee).read_bytes()).hexdigest()
    if args.trace_judge:
        manifest["trace_mode"] = "released-supervisor-read-only-step-wrapper"
        manifest["trace_judge_binary_sha256"] = hashlib.sha256(
            Path(args.trace_judge).read_bytes()).hexdigest()
    (log_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    environment = os.environ.copy()
    for name in ("CUPCUP_SIM_SEED", "CUPCUP_TRACE_PATH", "CUPCUP_PERCEPTION_CALIBRATION"):
        environment.pop(name, None)
    environment["CUPCUP_MOCK_COLOR"] = args.color
    environment["CUPCUP_MOCK_OPPONENT"] = (
        "cupcup_b" if args.opponent == "cupcup" else args.opponent
    )
    if args.seed is not None:
        environment["CUPCUP_SIM_SEED"] = str(args.seed)
    trace_path = log_dir / "world_trace.csv"
    if args.trace:
        environment["CUPCUP_TRACE_PATH"] = str(trace_path)
    # Never let inherited test injection change a nominal original run.
    environment.pop("CUPCUP_TRACE_JUDGE", None)
    if args.trace_judge:
        environment["CUPCUP_TRACE_JUDGE"] = args.trace_judge
    if args.calibration:
        environment["CUPCUP_PERCEPTION_CALIBRATION"] = "1"
    processes: list[subprocess.Popen] = []
    paused_motion: int | None = None
    resume_at = 0.0
    outage_injected = False
    started = time.monotonic()
    analysis_failed = False
    try:
        start = start_process(
            command_prefix() + ("ros2 launch " + shlex.quote(str(
                ROOT / "src/cupcup/tests/official_start_launch.py"))
                if args.official_launch_compat else "ros2 launch start start_launch.py"),
            log_dir / "start.log",
            environment,
        )
        processes.append(start)
        print(f"waiting_for_field logs={log_dir}", flush=True)
        if not wait_for_field(log_dir / "start.log", args.startup_timeout):
            print("ERROR: /sensor/field did not become ready", file=sys.stderr)
            return 2
        if args.trace and (not trace_path.is_file() or trace_path.stat().st_size == 0):
            print("ERROR: requested trace writer produced no evidence", file=sys.stderr)
            return 7
        if args.trace_judge and not binary_matches(
                Path(args.trace_judge), manifest["trace_judge_binary_sha256"]):
            print("ERROR: trace executable changed during startup", file=sys.stderr)
            return 7

        referee_script = ROOT / "src/cupcup/tests/mock_gamectrl.py"
        referee_command = command_prefix() + f"python3 {referee_script}"
        if args.released_referee:
            referee_command = command_prefix() + shlex.join([
                args.released_referee, args.color, environment["CUPCUP_MOCK_OPPONENT"],
                str(args.referee_play_seconds), str(log_dir / "released_referee.json")])
            environment["QT_QPA_PLATFORM"] = "offscreen"
        referee = start_process(
            referee_command,
            log_dir / "referee.log",
            environment,
            log_dir if args.released_referee else ROOT,
        )
        processes.append(referee)
        if args.observe:
            observer = start_process(
                command_prefix() + "ros2 run cupcup cupcup_visualizer --port 0 --record "
                + shlex.quote(str(log_dir / "belief.jsonl")),
                log_dir / "observer.log", environment)
            processes.append(observer)
        if args.images:
            recorder = ROOT / "src/cupcup/tests/record_images.py"
            recording = start_process(
                command_prefix() + f"python3 {recorder} {args.color} {log_dir / 'images'}",
                log_dir / "images.log", environment)
            processes.append(recording)
        cupcup = start_process(
            command_prefix() + "ros2 launch cupcup player_launch.py ball_pattern_filter:="
            + ("false" if args.no_ball_pattern_filter else "true")
            + " kick_settle_frames:=" + str(args.kick_settle_frames)
            + (" ball_verifier_model:=" + shlex.quote(str(args.ball_verifier_model))
               if args.ball_verifier_model else ""),
            log_dir / "cupcup.log",
            environment,
        )
        processes.append(cupcup)
        if args.calibration:
            opponent_command = None
        elif args.opponent == "unirobot":
            opponent_command = command_prefix() + "ros2 launch unirobot player_launch.py"
        elif args.opponent == "cupcup":
            opponent_command = command_prefix() + (
                "ros2 launch cupcup player_launch.py team_name:=cupcup_b"
            )
        else:
            opponent_script = ROOT / "src/cupcup/tests/scripted_opponent.py"
            opponent_color = "blue" if args.color == "red" else "red"
            opponent_command = command_prefix() + (
                f"(python3 {opponent_script} {opponent_color}_1 {args.opponent} & "
                f"python3 {opponent_script} {opponent_color}_2 {args.opponent})"
            )
        if opponent_command is not None:
            opponent = start_process(opponent_command, log_dir / "opponent.log", environment)
            processes.append(opponent)

        print(f"running_match seconds={args.duration:.1f}", flush=True)
        monitored = {"cupcup": cupcup, "referee": referee}
        if opponent_command is not None:
            monitored["opponent"] = opponent
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            if args.released_referee and (log_dir / "released_referee.json").is_file():
                # File is written at actual END. Wait for the process to close
                # it before reading, and require a successful requested clock.
                if referee.poll() is None:
                    time.sleep(0.1)
                    continue
                result = json.loads((log_dir / "released_referee.json").read_text())
                if (referee.returncode != 0 or not result.get("end_observed") or
                        not result.get("target_completed") or
                        result.get("requested_play_seconds") != args.referee_play_seconds):
                    print("ERROR: released referee did not complete target", file=sys.stderr)
                    return 6
                print(f"released_referee_completed play_seconds={result['referee_play_seconds']}",
                      flush=True)
                break
            if start.poll() is not None:
                print("ERROR: Webots launch exited during match", file=sys.stderr)
                return 3
            for name, process in monitored.items():
                if process.poll() is not None:
                    print(f"ERROR: {name} exited during match", file=sys.stderr)
                    return 5
                text = (log_dir / f"{name}.log").read_text(errors="replace")
                if "process has died" in text or "Traceback" in text:
                    print(f"ERROR: {name} child failed during match", file=sys.stderr)
                    return 5
            now = time.monotonic()
            if args.motion_outage and not outage_injected and now >= deadline - args.duration + 10:
                paused_motion = find_motion_pid(f"{args.color}_1", start.pid)
                if paused_motion is None:
                    print("ERROR: could not resolve motion process for outage", file=sys.stderr)
                    return 4
                os.kill(paused_motion, signal.SIGSTOP)
                resume_at = now + args.motion_outage
                outage_injected = True
                print(f"motion_outage_start robot={args.color}_1 pid={paused_motion}", flush=True)
            if paused_motion is not None and now >= resume_at:
                os.kill(paused_motion, signal.SIGCONT)
                print(f"motion_outage_end pid={paused_motion}", flush=True)
                paused_motion = None
            time.sleep(1.0)
        else:
            if args.released_referee:
                print("ERROR: wall watchdog expired before released referee target",
                      file=sys.stderr)
                return 6
    finally:
        if paused_motion is not None:
            try:
                os.kill(paused_motion, signal.SIGCONT)
            except ProcessLookupError:
                pass
        for process in reversed(processes):
            stop_process(process)
        if args.trace_judge and not binary_matches(
                Path(args.trace_judge), manifest["trace_judge_binary_sha256"]):
            analysis_failed = True
            print("ERROR: trace executable changed during run; reject this evidence",
                  file=sys.stderr)
        if args.ball_verifier_model and not binary_matches(
                args.ball_verifier_model, manifest["ball_verifier_model_sha256"]):
            analysis_failed = True
            print("ERROR: ball verifier model changed during run; reject this evidence",
                  file=sys.stderr)
        if args.trace and trace_path.exists():
            analyzer = ROOT / "src/cupcup/tests/analyze_world_trace.py"
            result = subprocess.run([sys.executable, str(analyzer), str(trace_path)],
                                    check=False, capture_output=True, text=True)
            (log_dir / "world_analysis.txt").write_text(
                result.stdout + result.stderr, encoding="utf-8")
            print(result.stdout, end="")
            if result.returncode:
                print(result.stderr, file=sys.stderr)
            if args.calibration:
                calibration_analyzer = (
                    ROOT / "src/cupcup/tests/analyze_perception_calibration.py"
                )
                result = subprocess.run(
                    [sys.executable, str(calibration_analyzer), str(trace_path),
                     "--color", args.color],
                    check=False, capture_output=True, text=True,
                )
                (log_dir / "calibration_analysis.txt").write_text(
                    result.stdout + result.stderr, encoding="utf-8")
                print(result.stdout, end="")
            if args.trace_judge:
                contact_analyzer = ROOT / "src/cupcup/tests/analyze_kick_contacts.py"
                result = subprocess.run(
                    [sys.executable, str(contact_analyzer), str(trace_path),
                     "--output", str(log_dir / "kick_contacts.json")],
                    check=False, capture_output=True, text=True)
                (log_dir / "kick_contacts.txt").write_text(
                    result.stdout + result.stderr, encoding="utf-8")
                print(result.stdout, end="")
                if result.returncode:
                    analysis_failed = True
                    print(result.stderr, file=sys.stderr)
            else:
                geometry_analyzer = ROOT / "src/cupcup/tests/compare_ball_geometry.py"
                result = subprocess.run(
                    [sys.executable, str(geometry_analyzer), str(trace_path),
                     "--color", args.color,
                     "--plane-height", "0.05" if args.calibration else "0.07",
                     "--output", str(log_dir / "geometry_analysis.json")],
                    check=False, capture_output=True, text=True)
                (log_dir / "geometry_analysis.txt").write_text(
                    result.stdout + result.stderr, encoding="utf-8")
        summary = io.StringIO()
        with contextlib.redirect_stdout(summary):
            summarize(log_dir, time.monotonic() - started, args.color, args.opponent, args.seed)
            print(f"motion_outage_seconds={args.motion_outage}")
        (log_dir / "summary.txt").write_text(summary.getvalue(), encoding="utf-8")
        print(summary.getvalue(), end="")
    return 8 if analysis_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
