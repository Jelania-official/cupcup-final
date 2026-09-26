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
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
INSTALL = Path(os.environ.get("CUPCUP_INSTALL", "/tmp/cupcup-full-install-v2"))
LOG_ROOT = Path(os.environ.get("CUPCUP_MATCH_LOG", "/tmp/cupcup-match-logs"))


def command_prefix() -> str:
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {INSTALL / 'setup.bash'} && "
    )


def start_process(command: str, log_path: Path, env: dict[str, str]) -> subprocess.Popen:
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=ROOT,
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


def summarize(log_dir: Path, elapsed: float) -> None:
    files = sorted(log_dir.glob("*.log"))
    joined = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in files)
    events = re.findall(r"field event=(\d+).*?score=(\d+):(\d+)", joined)
    goals = [event for event in events if event[0] == "4"]
    kicks = len(re.findall(r"kick foot=", joined))
    clears = len(re.findall(r"defender clear", joined))
    falls = len(re.findall(r"fall=[1-9]", joined))
    waits = len(re.findall(r"waiting:", joined))
    print(f"match_seconds={elapsed:.1f}")
    print(f"field_events={len(events)}")
    print(f"goal_events={len(goals)}")
    if events:
        print(f"last_score={events[-1][1]}:{events[-1][2]}")
    print(f"kick_transitions={kicks}")
    print(f"defender_clear_actions={clears}")
    print(f"health_wait_logs={waits}")
    print(f"fall_logs={falls}")
    print(f"logs={log_dir}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", choices=("red", "blue"), default="red")
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--opponent", default="unirobot")
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    args = parser.parse_args()

    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_dir = LOG_ROOT / f"{timestamp}-{args.color}-vs-{args.opponent}"
    log_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["CUPCUP_MOCK_COLOR"] = args.color
    environment["CUPCUP_MOCK_OPPONENT"] = (
        "cupcup_b" if args.opponent == "cupcup" else args.opponent
    )
    processes: list[subprocess.Popen] = []
    started = time.monotonic()
    try:
        start = start_process(
            command_prefix() + "ros2 launch start start_launch.py",
            log_dir / "start.log",
            environment,
        )
        processes.append(start)
        print(f"waiting_for_field logs={log_dir}", flush=True)
        if not wait_for_field(log_dir / "start.log", args.startup_timeout):
            print("ERROR: /sensor/field did not become ready", file=sys.stderr)
            return 2

        referee = start_process(
            command_prefix() + f"python3 {ROOT / 'src/cupcup/tests/mock_gamectrl.py'}",
            log_dir / "referee.log",
            environment,
        )
        processes.append(referee)
        cupcup = start_process(
            command_prefix() + "ros2 launch cupcup player_launch.py",
            log_dir / "cupcup.log",
            environment,
        )
        processes.append(cupcup)
        if args.opponent == "unirobot":
            opponent_command = command_prefix() + "ros2 launch unirobot player_launch.py"
        elif args.opponent == "cupcup":
            opponent_command = command_prefix() + (
                "ros2 launch cupcup player_launch.py team_name:=cupcup_b"
            )
        else:
            opponent_script = ROOT / "src/cupcup/tests/scripted_opponent.py"
            opponent_command = command_prefix() + (
                f"(python3 {opponent_script} blue_1 {args.opponent} & "
                f"python3 {opponent_script} blue_2 {args.opponent})"
            )
        opponent = start_process(opponent_command, log_dir / "opponent.log", environment)
        processes.append(opponent)

        print(f"running_match seconds={args.duration:.1f}", flush=True)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            if start.poll() is not None:
                print("ERROR: Webots launch exited during match", file=sys.stderr)
                return 3
            time.sleep(1.0)
    finally:
        for process in reversed(processes):
            stop_process(process)
        summarize(log_dir, time.monotonic() - started)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
