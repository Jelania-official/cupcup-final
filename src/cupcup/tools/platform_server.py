#!/usr/bin/env python3
"""Serve the cupcup live match monitor and tactical sandbox UI."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


class LiveState:
    def __init__(self, record_path: Path | None = None) -> None:
        self.lock = threading.Lock()
        self.robots: dict[str, dict[str, object]] = {}
        self.received: dict[str, float] = {}
        self.score = {"red": 0, "blue": 0}
        self.game_state: int | None = None
        self.ros_available = False
        self.started_at = time.monotonic()
        self.history: deque[dict] = deque(maxlen=4500)
        self.record_path = record_path
        self.last_recorded = -1.0
        if record_path is not None:
            record_path.parent.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def parse_talk(wire: str) -> dict[str, str]:
        if not wire.startswith("cupcup|"):
            return {}
        fields = {}
        for field in wire.split("|")[1:]:
            key, separator, value = field.partition("=")
            if separator:
                fields[key] = value
        return fields

    @staticmethod
    def safe_number(fields: dict[str, str], key: str) -> float | None:
        try:
            value = float(fields[key])
        except (KeyError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def receive_talk(self, robot: str, wire: str) -> None:
        fields = self.parse_talk(wire)
        if not fields:
            return
        names = {
            "x": "px", "z": "pz", "yaw": "pyaw",
            "ball_x": "bx", "ball_z": "bz", "ball_age": "bmap_age",
            "confidence": "conf", "target_x": "tx", "target_z": "tz",
            "candidate_x": "rx", "candidate_z": "rz",
            "candidate_confidence": "rc", "candidate_age": "ra",
            "pose_age": "pose_age",
            "kick_x": "kx", "kick_z": "kz",
            "ball_distance": "bdist",
            "aim_yaw": "aim_yaw", "aim_offset": "aim_offset",
        }
        data: dict[str, object] = {
            "name": robot,
            "role": fields.get("role", "unknown"),
            "state": fields.get("state", "unknown"),
            "action": fields.get("tac", fields.get("state", "unknown")),
            "healthy": fields.get("healthy") == "1",
            "sees_ball": fields.get("ball") == "1",
            "reason": fields.get("why", "unknown"),
        }
        for output, source in names.items():
            data[output] = self.safe_number(fields, source)
        count = self.safe_number(fields, "peer_obstacles")
        data["peer_obstacles"] = (int(count) if count is not None and
                                  0 <= count <= 4 and count == int(count) else None)
        source = fields.get("ball_range_source", "unknown")
        data["ball_range_source"] = source if source in {
            "ground_ray", "radius"} else "unknown"
        mx, mz, age = (self.safe_number(fields, key) for key in ("mbx", "mbz", "mbage"))
        data["mapped_ball"] = ([mx, mz] if None not in (mx, mz, age) and
                               abs(mx) <= 6 and abs(mz) <= 4 and 0 <= age <= .65 else None)
        data["mapped_ball_age"] = age if data["mapped_ball"] is not None else None
        source = fields.get("mbsource", "unknown")
        data["mapped_ball_source"] = source if source in {
            "local", "ground_ray", "radius", "teammate"} else "unknown"
        if data["target_x"] is not None and data["target_z"] is not None:
            data["target"] = [data["target_x"], data["target_z"]]
        else:
            data["target"] = None
        data["kick_target"] = ([data["kick_x"], data["kick_z"]]
                               if None not in (data["kick_x"], data["kick_z"]) else None)
        if data["candidate_x"] is not None and data["candidate_z"] is not None:
            data["robot_candidate"] = {
                "x": data["candidate_x"], "z": data["candidate_z"],
                "confidence": data["candidate_confidence"],
                "age_s": data["candidate_age"],
            }
        else:
            data["robot_candidate"] = None
        data["robot_tracks"] = []
        for index in range(4):
            prefix = f"r{index}"
            x = self.safe_number(fields, prefix + "x")
            z = self.safe_number(fields, prefix + "z")
            if x is None or z is None:
                continue
            team = fields.get(prefix + "team", "unknown")
            source = fields.get(prefix + "source", "unknown")
            data["robot_tracks"].append({
                "track_id": self.safe_number(fields, prefix + "id"),
                "team": team if team in {"red", "blue"} else "unknown",
                "latest_source": source if source in {
                    "box_height", "number_square"} else "unknown",
                "x": x, "z": z,
                "confidence": self.safe_number(fields, prefix + "conf"),
                "uncertainty": self.safe_number(fields, prefix + "unc"),
                "age_s": self.safe_number(fields, prefix + "age"),
            })
        # Effective one-hop policy inputs are distinct from local visual tracks.
        data["peer_obstacle_tracks"] = []
        for index in range(data["peer_obstacles"] or 0):
            prefix = f"pobs{index}"
            values = {key: self.safe_number(fields, prefix + key)
                      for key in ("x", "z", "conf", "unc", "age")}
            team = fields.get(prefix + "team")
            if (None in values.values() or abs(values["x"]) > 6 or abs(values["z"]) > 4 or
                    not 0 < values["conf"] <= .25 or values["unc"] < 0 or
                    not 0 <= values["age"] <= .65 or team not in {"red", "blue"}):
                continue
            data["peer_obstacle_tracks"].append({
                "x": values["x"], "z": values["z"], "team": team,
                "confidence": values["conf"], "uncertainty": values["unc"],
                "age_s": values["age"], "source": "teammate_direct_number_square"})
        with self.lock:
            self.robots[robot] = data
            self.received[robot] = time.monotonic()
            self._record_locked()

    def _snapshot_locked(self) -> dict[str, object]:
        now = time.monotonic()
        robots = []
        for robot, data in sorted(self.robots.items()):
            item = dict(data)
            item["message_age_s"] = max(0.0, now - self.received.get(robot, now))
            robots.append(item)
        return {
            "mode": "live", "ros_available": self.ros_available,
            "score": dict(self.score), "game_state": self.game_state,
            "robots": robots, "elapsed_s": now - self.started_at,
            "truth_available": False,
        }

    def _record_locked(self) -> None:
        now = time.monotonic()
        if now - self.last_recorded < 0.2:
            return
        frame = self._snapshot_locked()
        self.history.append(frame)
        if self.record_path is not None:
            with self.record_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(frame, ensure_ascii=False, allow_nan=False) + "\n")
        self.last_recorded = now

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return self._snapshot_locked()

    def replay(self) -> dict:
        with self.lock:
            return {"model": "live-belief-v1", "truth_available": False,
                    "frames": list(self.history)}


def start_ros_bridge(
    state: LiveState,
) -> tuple[object | None, object | None, threading.Thread | None]:
    try:
        import rclpy
        from common.msg import GameData, Talk
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
    except ImportError as error:
        print(
            "ROS 2 Python interface unavailable; live mode disabled: "
            f"{error}", flush=True
        )
        return None, None, None

    # Let the server's main thread catch Ctrl-C and shut the executor down
    # before tearing down the shared rclpy context.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("cupcup_visualizer")
    for team in ("red", "blue"):
        for player_id in (1, 2):
            robot = f"{team}_{player_id}"
            node.create_subscription(
                Talk,
                f"/{robot}/talk/talk_str",
                lambda msg, name=robot: _receive_with_time(
                    state, name, msg.talk_str
                ),
                10,
            )

    def game_callback(message: object) -> None:
        with state.lock:
            state.score = {
                "red": int(message.red_score),
                "blue": int(message.blue_score),
            }
            state.game_state = int(message.state)
            state._record_locked()

    node.create_subscription(GameData, "/sensor/game", game_callback, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    state.ros_available = True
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, node, thread


def _receive_with_time(state: LiveState, robot: str, wire: str) -> None:
    state.receive_talk(robot, wire)


def package_paths() -> tuple[Path, Path]:
    here = Path(__file__).resolve().parent
    web = here / "web"
    sandbox = Path(os.environ.get("CUPCUP_SANDBOX_BINARY", ""))
    try:
        from ament_index_python.packages import (
            get_package_prefix,
            get_package_share_directory,
        )
        prefix = Path(get_package_prefix("cupcup"))
        if not sandbox.is_file():
            sandbox = prefix / "lib" / "cupcup" / "cupcup_sandbox"
        installed_web = Path(get_package_share_directory("cupcup")) / "web"
        if installed_web.is_dir():
            web = installed_web
    except ImportError:
        pass
    return web, sandbox


def experiment_options(form: dict[str, list[str]]) -> list[str]:
    """One validated parameter mapping for both replay and paired batch tests."""
    options = []
    fields = {
        "speed": (0.4, 0.0, 1.5), "turn": (150.0, 1.0, 360.0),
        "setup": (1.0, 0.05, 10.0),
        "opponent_speed": (0.4, 0.0, 1.5), "opponent_turn": (150.0, 1.0, 360.0),
        "opponent_setup": (1.0, 0.05, 10.0),
        "opponent_kick_speed": (float(form.get("kick_speed", ["0.9"])[0]), 0.0, 8.0),
        "delay": (0.1, 0.0, 2.0), "dropout": (0.0, 0.0, 1.0),
        "self_noise": (0.0, 0.0, 1.0), "ball_decay": (0.9, 0.05, 5.0),
        "kick_hysteresis": (0.25, 0.0, 2.0),
    }
    for key, (default, low, high) in fields.items():
        value = float(form.get(key, [str(default)])[0])
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} out of range")
        options.extend(["--" + key.replace("_", "-"), str(value)])
    for key, default, allowed in (
        ("mode", "realistic", {"oracle", "realistic"}),
        ("scenario", "kickoff", {"kickoff", "own-half", "incoming", "sideline", "shot"}),
        ("defense-mode", "line", {"line", "fixed"}),
        ("vision-model", "pan", {"pan", "legacy"}),
    ):
        value = form.get(key.replace("-", "_"), [default])[0]
        if value not in allowed:
            raise ValueError(f"unknown {key}")
        options.extend(["--" + key, value])
    initial_mode = form.get("initial_state_mode", ["preset"])[0]
    if initial_mode not in {"preset", "custom"}:
        raise ValueError("unknown initial state mode")
    if initial_mode == "custom":
        values = []
        for entity in ("red1", "red2", "blue1", "blue2", "ball"):
            axes = ("x", "z", "vx", "vz") if entity == "ball" else ("x", "z", "yaw")
            for axis in axes:
                key = f"initial_{entity}_{axis}"
                if key not in form:
                    raise ValueError(f"missing {key}")
                value = float(form[key][0])
                bound = {"x": 4.5 if entity == "ball" else 4.3,
                         "z": 3.0 if entity == "ball" else 2.8,
                         "yaw": 180, "vx": 8, "vz": 8}[axis]
                if not math.isfinite(value) or abs(value) > bound:
                    raise ValueError(f"initial {entity} {axis} out of range")
                values.append(str(value))
        options.extend(["--initial-state", " ".join(values)])
    return options


def make_handler(state: LiveState, web_root: Path, sandbox_binary: Path):
    class Handler(BaseHTTPRequestHandler):
        server_version = "cupcup-platform/0.1"

        def _send(
            self, payload: bytes, content_type: str, status: int = 200
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/api/live/history":
                self._send(json.dumps(state.replay(), allow_nan=False).encode(),
                           "application/json; charset=utf-8")
                return
            if path == "/api/live":
                payload = json.dumps(
                    state.snapshot(), allow_nan=False
                ).encode()
                self._send(payload, "application/json; charset=utf-8")
                return
            if path == "/api/health":
                payload = json.dumps({
                    "ok": True,
                    "sandbox_available": sandbox_binary.is_file(),
                    "ros_available": state.ros_available,
                }).encode()
                self._send(payload, "application/json; charset=utf-8")
                return
            assets = {
                "/": "index.html", "/index.html": "index.html",
                "/app.js": "app.js", "/style.css": "style.css",
            }
            # Only serve known assets; ament symlink-install links these files
            # outside the install directory. Never accept arbitrary paths.
            if path not in assets:
                self._send(b"forbidden", "text/plain", 403)
                return
            target = web_root / assets[path]
            if not target.is_file():
                self._send(b"not found", "text/plain", 404)
                return
            content_type = {
                ".html": "text/html; charset=utf-8",
                ".css": "text/css; charset=utf-8",
                ".js": "text/javascript; charset=utf-8",
            }.get(target.suffix, "application/octet-stream")
            self._send(target.read_bytes(), content_type)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path not in {"/api/sandbox", "/api/evaluate"}:
                self._send(b"not found", "text/plain", 404)
                return
            if not sandbox_binary.is_file():
                self._send(
                    b'{"error":"cupcup_sandbox binary is not built"}',
                    "application/json; charset=utf-8",
                    503,
                )
                return
            length = min(int(self.headers.get("Content-Length", "0")), 4096)
            form = parse_qs(self.rfile.read(length).decode("utf-8", "replace"))
            try:
                duration = min(
                    900.0, max(5.0, float(form.get("duration", ["45"])[0]))
                )
                seed = int(form.get("seed", ["7"])[0]) % (2**32)
                noise = min(
                    0.8, max(0.0, float(form.get("noise", ["0.20"])[0]))
                )
                kick_speed = min(
                    6.0, max(0.0, float(form.get("kick_speed", ["0.9"])[0]))
                )
                if path == "/api/evaluate":
                    trials = int(form.get("trials", ["8"])[0])
                    if not 2 <= trials <= 32:
                        raise ValueError("trials must be between 2 and 32")
                extra_options = experiment_options(form)
                opponent = form.get("opponent", ["block"])[0]
                cupcup_color = form.get("cupcup_color", ["red"])[0]
                if cupcup_color not in {"red", "blue"}:
                    raise ValueError("unknown cupcup color")
                if opponent not in {"press", "block", "keeper", "shared"}:
                    raise ValueError("unknown opponent style")
            except (ValueError, OverflowError):
                self._send(
                    b'{"error":"invalid sandbox parameters"}',
                    "application/json; charset=utf-8",
                    400,
                )
                return
            if path == "/api/evaluate":
                return self._evaluate(
                    seed, trials, duration, noise, kick_speed, opponent, extra_options
                )
            command = [
                str(sandbox_binary), "--duration", str(duration),
                "--seed", str(seed),
                "--noise", str(noise), "--kick-speed", str(kick_speed),
                "--opponent", opponent, "--cupcup-color", cupcup_color,
            ] + extra_options
            try:
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=8.0,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                self._send(
                    b'{"error":"sandbox timed out"}',
                    "application/json; charset=utf-8",
                    504,
                )
                return
            if result.returncode != 0:
                self._send(
                    json.dumps({"error": result.stderr.strip()}).encode(),
                    "application/json; charset=utf-8",
                    400,
                )
                return
            try:
                json.loads(result.stdout)
            except json.JSONDecodeError:
                self._send(
                    b'{"error":"sandbox produced invalid output"}',
                    "application/json; charset=utf-8",
                    500,
                )
                return
            self._send(
                result.stdout.encode(), "application/json; charset=utf-8"
            )

        def _evaluate(
            self, seed: int, trials: int, duration: float, noise: float,
            kick_speed: float, opponent: str, extra_options: list[str],
        ) -> None:
            results = []
            for index in range(trials):
                trial_seed = (seed + index) % (2**32)
                for color in ("red", "blue"):
                    command = [
                        str(sandbox_binary), "--duration", str(duration),
                        "--seed", str(trial_seed), "--noise", str(noise),
                        "--kick-speed", str(kick_speed),
                        "--opponent", opponent,
                        "--cupcup-color", color,
                        "--frames", "final",
                    ] + extra_options
                    try:
                        result = subprocess.run(
                            command, capture_output=True, text=True,
                            timeout=8.0, check=False,
                        )
                    except subprocess.TimeoutExpired:
                        self._send(
                            b'{"error":"evaluation timed out"}',
                            "application/json; charset=utf-8", 504,
                        )
                        return
                    if result.returncode != 0:
                        self._send(
                            json.dumps({
                                "error": result.stderr.strip()
                            }).encode(),
                            "application/json; charset=utf-8", 400,
                        )
                        return
                    try:
                        experiment = json.loads(result.stdout)
                        final = experiment["frames"][-1]
                        cupcup_goals = final[f"{color}_score"]
                        other = "blue" if color == "red" else "red"
                        results.append({
                            "seed": trial_seed, "color": color,
                            "cupcup_goals": cupcup_goals,
                            "opponent_goals": final[f"{other}_score"],
                            "cupcup_kicks": experiment["metrics"][f"{color}_kicks"],
                            "cupcup_penalties": experiment["metrics"][f"{color}_penalties"],
                            "restarts": experiment["metrics"]["restarts"],
                            "collision_steps": experiment["metrics"]["collision_steps"],
                        })
                    except (KeyError, IndexError, json.JSONDecodeError):
                        self._send(
                            b'{"error":"sandbox produced invalid evaluation"}',
                            "application/json; charset=utf-8", 500,
                        )
                        return
            wins = sum(
                item["cupcup_goals"] > item["opponent_goals"]
                for item in results
            )
            draws = sum(
                item["cupcup_goals"] == item["opponent_goals"]
                for item in results
            )
            goals = sum(
                item["cupcup_goals"] + item["opponent_goals"]
                for item in results
            )
            by_color = {}
            for color in ("red", "blue"):
                subset = [item for item in results if item["color"] == color]
                by_color[color] = {
                    "wins": sum(
                        item["cupcup_goals"] > item["opponent_goals"]
                        for item in subset
                    ),
                    "draws": sum(
                        item["cupcup_goals"] == item["opponent_goals"]
                        for item in subset
                    ),
                    "losses": sum(
                        item["cupcup_goals"] < item["opponent_goals"]
                        for item in subset
                    ),
                }
            payload = {
                "model": experiment["model"],
                "calibrated": experiment["calibrated"],
                "parameters": experiment["parameters"],
                "mode": experiment["mode"], "scenario": experiment["scenario"],
                "opponent": opponent,
                "duration_s": duration,
                "trials": trials,
                "matches": len(results),
                "cupcup_wins": wins,
                "draws": draws,
                "cupcup_losses": len(results) - wins - draws,
                "mean_goals_per_match": goals / len(results),
                "by_color": by_color,
                "results": results,
            }
            self._send(
                json.dumps(payload, allow_nan=False).encode(),
                "application/json; charset=utf-8",
            )

        def log_message(self, format_string: str, *args: object) -> None:
            print(f"visualizer: {format_string % args}", flush=True)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--record", type=Path,
                        help="live estimate JSONL path (default: persistent per-session log)")
    args = parser.parse_args()
    web_root, sandbox_binary = package_paths()
    if not web_root.is_dir():
        parser.error(f"web assets not found: {web_root}")
    record_path = args.record or Path.home() / ".local/state/cupcup/platform" / (
        time.strftime("%Y%m%d-%H%M%S") + ".jsonl")
    state = LiveState(record_path if not args.no_ros else None)
    executor = node = ros_thread = None
    if not args.no_ros:
        executor, node, ros_thread = start_ros_bridge(state)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(
        state, web_root, sandbox_binary
    ))
    status = (
        f"cupcup platform at http://{args.host}:{server.server_port} "
        f"(live_ros={state.ros_available}, "
        f"sandbox={sandbox_binary.is_file()})"
    )
    print(status, flush=True)
    if not args.no_ros:
        print(f"live estimates recorded to {record_path}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if ros_thread is not None:
            ros_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except ImportError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
