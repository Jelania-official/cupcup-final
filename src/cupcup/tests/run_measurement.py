#!/usr/bin/env python3
"""Run controlled motion/kick/rolling trials against an isolated installed platform."""

import argparse
import csv
import hashlib
import json
import os
import shlex
import subprocess
import time
from pathlib import Path

from run_match import ROOT, INSTALL, binary_matches, command_prefix, start_process, stop_process


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--color", choices=("red", "blue"), default="red")
    parser.add_argument("--profile", choices=(
        "calibration", "kick-timing", "kick-blocking", "robot-views", "robot-dynamic",
        "robot-ball-views",
        "ball-views", "ball-dynamic"),
                        default="calibration")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--stall-timeout", type=float, default=90.0,
                        help="abort after this many wall seconds without CSV progress")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--controller-adapter", action="store_true",
                        help="use bounded service waits with otherwise released SimRobot")
    parser.add_argument("--released-source", type=Path,
                        default=Path.home() / ".cache/cupcup/upstream-validation"
                        / "seurobocup-kidsize2026Fin")
    args = parser.parse_args()
    if (not 1 <= args.repeats <= 10 or not 30 <= args.timeout <= 3600
            or not 30 <= args.stall_timeout <= 300):
        parser.error("repeats 1..10, timeout 30..3600 and stall-timeout 30..300 wall seconds")
    output = args.output or Path.home() / ".local/state/cupcup/measurements" / (
        time.strftime("%Y%m%d-%H%M%S") + "-" + args.color)
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").mkdir()
    build = Path.home() / ".cache/cupcup/measurement-build"
    source = ROOT / "src/cupcup/tests/measurement"
    released = args.released_source / "src/simulation/controller/src"
    if not (released / "SimRobot.cpp").is_file():
        parser.error("released controller source not found")
    binary = build / "measurement_supervisor"
    command = (command_prefix() + "cmake -S " + shlex.quote(str(source)) + " -B "
               + shlex.quote(str(build)) + " -DRELEASED_CONTROLLER_SOURCE="
               + shlex.quote(str(released)) + " && cmake --build "
               + shlex.quote(str(build)) + " -j2")
    result = subprocess.run(["bash", "-lc", command], capture_output=True, text=True)
    (output / "build.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        print(f"build failed; inspect {output}")
        return 2
    environment = os.environ.copy()
    environment.pop("CUPCUP_MEASUREMENT_CONTROLLER", None)
    environment.update({"CUPCUP_MEASUREMENT_JUDGE": str(binary),
                        "CUPCUP_MEASURE_OUTPUT": str(output),
                        "CUPCUP_MEASURE_COLOR": args.color,
                        "CUPCUP_MEASURE_PROFILE": args.profile,
                        "CUPCUP_MEASURE_REPEATS": str(args.repeats)})
    if args.controller_adapter:
        environment["CUPCUP_MEASUREMENT_CONTROLLER"] = str(build / "test_controller_adapter")
    artifacts = {
        "measurement": binary,
        "measurement_source": source / "supervisor.cpp",
        "measurement_launch": ROOT / "src/cupcup/tests/measurement_launch.py",
        "measurement_runner": Path(__file__),
        "controller": INSTALL / "controller/lib/controller/controller",
        "motion": INSTALL / "motion/lib/motion/motion",
        "robot_model": INSTALL / "webots/share/webots/models/protos/SEURobot.proto",
        "ball_model": INSTALL / "webots/share/webots/models/worlds/sim-robot.wbt",
        "walk": INSTALL / "params/share/params/conf/action/walk.conf",
        "actions": INSTALL / "params/share/params/conf/action/action.conf",
    }
    profile_file = environment.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    if profile_file:
        artifacts["dds_profile"] = Path(profile_file).resolve()
    if args.controller_adapter:
        artifacts["controller_adapter"] = build / "test_controller_adapter"
        artifacts["adapter_source"] = source / "controller_adapter.cpp"
        artifacts["released_sensor_source"] = released / "SimRobot.cpp"
        artifacts["target_logging_source"] = source / "target_trace.hpp"
    if args.profile == "kick-timing":
        config = INSTALL / "params/share/params/conf"
        templates = output / "action_templates.csv"
        template_command = command_prefix() + " ".join(shlex.quote(str(path)) for path in (
            build / "action_templates", config / "model/robot.conf",
            config / "action/offset.conf", config / "action/action.conf", templates))
        reference = subprocess.run(["bash", "-lc", template_command], capture_output=True,
                                   text=True)
        (output / "templates.log").write_text(reference.stdout + reference.stderr)
        if reference.returncode:
            print(f"action templates failed; inspect {output}")
            return 2
        artifacts["action_templates"] = templates
        artifacts["template_generator"] = build / "action_templates"
        artifacts["template_source"] = source / "action_templates.cpp"
        artifacts["robot_configuration"] = config / "model/robot.conf"
        artifacts["offset_configuration"] = config / "action/offset.conf"
    arguments = {key: str(value) if isinstance(value, Path) else value
                 for key, value in vars(args).items()}
    manifest = {"arguments": arguments | {"output": str(output)}, "install": str(INSTALL),
                "sha256": {key: hashlib.sha256(path.read_bytes()).hexdigest()
                           for key, path in artifacts.items() if path.is_file()},
                "clock": "Webots simulation seconds", "competition_strategy_running": False}
    manifest["dds_environment"] = {key: environment.get(key) for key in (
        "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
        "RMW_FASTRTPS_USE_QOS_FROM_XML", "RMW_FASTRTPS_PUBLICATION_MODE")}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    launch = ROOT / "src/cupcup/tests/measurement_launch.py"
    process = start_process(command_prefix() + "ros2 launch " + shlex.quote(str(launch)),
                            output / "simulation.log", environment)
    print(f"measurement running output={output}", flush=True)
    started = time.monotonic()
    previous = 0
    last_progress, last_size, stop_reason = started, -1, "process_exit"
    try:
        while process.poll() is None and time.monotonic() - started < args.timeout:
            trace = output / "motion_trace.csv"
            size = trace.stat().st_size if trace.is_file() else 0
            if size > last_size:
                last_progress, last_size = time.monotonic(), size
            path = output / "trials.csv"
            if path.is_file():
                with path.open() as stream:
                    complete = sum(1 for _ in csv.DictReader(stream))
                if complete != previous:
                    print(f"completed {complete}/{args.repeats * 8} trials", flush=True)
                    previous = complete
            if time.monotonic() - last_progress > args.stall_timeout:
                stop_reason = "no_csv_progress"
                break
            time.sleep(1.0)
        if process.poll() is None and stop_reason == "process_exit":
            stop_reason = "wall_timeout"
    finally:
        stop_process(process)
    trials = []
    if (output / "trials.csv").is_file():
        with (output / "trials.csv").open() as stream:
            trials = list(csv.DictReader(stream))
    changed = [name for name, path in artifacts.items() if name in manifest["sha256"]
               and not binary_matches(path, manifest["sha256"][name])]
    complete = len(trials) == args.repeats * 8 and not changed
    if args.profile in ("robot-views", "robot-dynamic", "robot-ball-views",
                        "ball-views", "ball-dynamic"):
        samples = []
        view_file = output / (
            "robot_views.csv" if args.profile.startswith("robot-") else "ball_views.csv")
        if view_file.is_file():
            with view_file.open() as stream:
                samples = list(csv.DictReader(stream))
        complete = complete and len(samples) == len(trials) and all(
            row["image_saved"] == "1" and (output / "images" / row["image"]).is_file()
            and all(0 <= float(row[key]) <= .25 for key in ("image_age", "imu_age", "head_age"))
            for row in samples)
    result = {"complete": complete, "trials": len(trials), "stop_reason": stop_reason,
              "changed_artifacts": changed,
              "wall_seconds": time.monotonic() - started, "returncode": process.poll()}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"complete={complete} wall_seconds={result['wall_seconds']:.1f} logs={output}")
    return 0 if complete else 3


if __name__ == "__main__":
    raise SystemExit(main())
