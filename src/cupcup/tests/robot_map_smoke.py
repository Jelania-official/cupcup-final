#!/usr/bin/env python3
"""Verify production RGB-to-map-to-Talk with a recorded static view, without Webots."""

import argparse
import csv
import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
from common.msg import GameData, HeadAngles, ImuData, Location, Talk
from common.srv import GetColor
from rclpy.node import Node
from sensor_msgs.msg import Image

from analyze_world_trace import parse_talk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurement", type=Path)
    parser.add_argument("--trial", type=int, default=1,
                        help="recorded robot view trial (default: 1)")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ball-model", type=Path)
    parser.add_argument("--verifier-model", type=Path)
    parser.add_argument("--playing", action="store_true",
                        help="synthetic PLAY in isolated domain; no motion services are started")
    parser.add_argument("--trace", action="store_true",
                        help="record existing production sensor-age diagnostics")
    parser.add_argument("--peer-obstacles", action="store_true",
                        help="also inject one-hop opponent Talk; requires --playing")
    args = parser.parse_args()
    if args.peer_obstacles and not args.playing:
        parser.error("--peer-obstacles requires --playing")
    models = {name: path.resolve() for name, path in (
        ("ball_model", args.ball_model), ("ball_verifier_model", args.verifier_model)) if path}
    if any(not path.is_file() for path in models.values()):
        parser.error("model arguments must name existing files")
    model_hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest()
                    for name, path in models.items()}
    # Never publish synthetic sensor inputs into a live match's DDS domain.
    if os.environ.get("ROS_DOMAIN_ID") != "142":
        parser.error("run only in the dedicated ROS_DOMAIN_ID=142 fixture domain")
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.measurement / "robot_views.csv").open() as stream:
        sample = next((row for row in csv.DictReader(stream)
                       if row["trial"] == str(args.trial)), None)
    if sample is None:
        parser.error("selected trial is absent from robot_views.csv")
    image_path = args.measurement / "images" / sample["image"]
    pixels = cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)
    observer, color = sample["observer"], sample["observer"].split("_")[0]
    binary_hash = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    rclpy.init()
    node = Node("cupcup_recorded_map_fixture")

    def get_color(request, response):
        response.color = color if request.team == "cupcup" else "invalid"
        return response

    service = node.create_service(GetColor, "gamectrl/get_color", get_color)
    publishers = {
        "game": node.create_publisher(GameData, "/sensor/game", 2),
        "imu": node.create_publisher(ImuData, f"/{observer}/sensor/imu", 2),
        "head": node.create_publisher(HeadAngles, f"/{observer}/sensor/joint/head", 2),
        "location": node.create_publisher(Location, f"/sensor/{observer}_location", 2),
        "image": node.create_publisher(Image, f"/{observer}/sensor/image", 2),
    }
    records = []
    peer_publisher = node.create_publisher(Talk, f"/{color}_2/talk/talk_str", 2)
    started = time.monotonic()

    def receive(message):
        records.append({"elapsed_s": time.monotonic() - started,
                        "fields": parse_talk(message.talk_str)})

    subscription = node.create_subscription(Talk, f"/{observer}/talk/talk_str", receive, 5)
    game = GameData()
    game.state, game.mode = GameData.STATE_PAUSE, GameData.MODE_NORM
    if args.playing:
        game.state = GameData.STATE_PLAY
        for player in game.red_players + game.blue_players:
            player.state = player.PLAYER_NORMAL
    imu = ImuData()
    imu.yaw, imu.pitch, imu.roll = (float(sample[k]) for k in (
        "imu_yaw", "imu_pitch", "imu_roll"))
    head = HeadAngles()
    head.yaw, head.pitch = float(sample["head_yaw"]), float(sample["head_pitch"])
    location = Location()  # Fixed zero fixture origin, NOT measured global localization.
    image = Image()
    image.height, image.width, image.encoding = pixels.shape[0], pixels.shape[1], "rgb8"
    image.step, image.data = image.width * 3, pixels.tobytes()
    # No header/sensor stamps, matching the original publisher contract.
    log = (args.output / "player.log").open("w")
    command = [str(args.binary), "cupcup_1"]
    if models:
        command += ["--ros-args"]
        for name, path in models.items():
            command += ["-p", f"{name}:={path}"]
    fixture_env = os.environ.copy()
    if args.trace:
        fixture_env["CUPCUP_TRACE_PATH"] = str(args.output.resolve() / "diagnostic-trace")
    process = subprocess.Popen(command, stdout=log, env=fixture_env,
                               stderr=subprocess.STDOUT, start_new_session=True)
    try:
        last_publish = -1.0
        while time.monotonic() - started < 12 and process.poll() is None:
            elapsed = time.monotonic() - started
            if elapsed - last_publish >= .1:
                publishers["game"].publish(game)
                publishers["imu"].publish(imu)
                publishers["location"].publish(location)
                if not 4 <= elapsed < 6:
                    publishers["head"].publish(head)
                image.data = pixels.tobytes() if elapsed < 10 else bytes(image.height * image.step)
                publishers["image"].publish(image)
                if args.peer_obstacles and not 4 <= elapsed < 6:
                    peer = Talk()
                    enemy = "blue" if color == "red" else "red"
                    team = color if 8 <= elapsed < 10 else enemy
                    source = "box_height" if elapsed >= 10 else "number_square"
                    peer.talk_str = ("cupcup|id=2|role=defender|healthy=1|robots=1"
                                     "|r0x=-3|r0z=-2|r0conf=0.25|r0unc=1|r0age=0.1"
                                     f"|r0team={team}|r0source={source}")
                    peer_publisher.publish(peer)
                last_publish = elapsed
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        node.destroy_subscription(subscription)
        node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()

    def contains_marker(record):
        fields = record["fields"]
        return any(fields.get(f"r{i}source") == "number_square"
                   and fields.get(f"r{i}team") == sample["target"].split("_")[0]
                   and 0 < float(fields[f"r{i}conf"]) <= .25 for i in range(4))

    phases = {name: [r for r in records if low <= r["elapsed_s"] < high]
              for name, low, high in (("valid", 2, 4), ("head_stale", 5, 6),
                                      ("recovered", 8, 10), ("blank", 11, 12))}
    unchanged = binary_hash == hashlib.sha256(args.binary.read_bytes()).hexdigest()
    checks = {"recorded_rgb_marker_reaches_map_and_talk": any(
                  map(contains_marker, phases["valid"])),
              "stale_head_does_not_refresh_position": bool(phases["head_stale"]) and all(
                  r["fields"].get("robots") == "0" for r in phases["head_stale"]),
              "restored_head_recovers_marker": any(map(contains_marker, phases["recovered"])),
              "blank_image_expires_marker": bool(phases["blank"]) and all(
                  r["fields"].get("robots") == "0" for r in phases["blank"]),
              "binary_unchanged": unchanged}
    text = (args.output / "player.log").read_text()
    if args.peer_obstacles:
        def shared_count(record):
            return int(record["fields"].get("peer_obstacles", "-1"))
        checks["peer_opponent_reaches_policy_input"] = any(
            shared_count(r) == 1 for r in phases["valid"])
        checks["effective_peer_geometry_is_published_separately"] = any(
            shared_count(r) == 1 and float(r["fields"].get("pobs0x", "nan")) == -3.0
            and float(r["fields"].get("pobs0z", "nan")) == -2.0
            and 0 <= float(r["fields"].get("pobs0age", "nan")) <= .65
            and 0 < float(r["fields"].get("pobs0conf", "nan")) <= .25
            for r in phases["valid"])
        checks["silent_peer_expires"] = bool(phases["head_stale"]) and all(
            shared_count(r) == 0 for r in phases["head_stale"])
        checks["peer_own_team_rejected"] = bool(phases["recovered"]) and all(
            shared_count(r) == 0 for r in phases["recovered"] if r["elapsed_s"] >= 8.8)
        checks["peer_box_height_rejected"] = bool(phases["blank"]) and all(
            shared_count(r) == 0 for r in phases["blank"])
        checks["peer_track_not_relayed"] = all(
            not (float(r["fields"].get(f"r{i}x", "nan")) == -3.0 and
                 float(r["fields"].get(f"r{i}z", "nan")) == -2.0)
            for r in records for i in range(4))
    if args.trace:
        # Tracks may persist for their TTL, but may not receive a timestamp
        # newer than the last image with a usable receipt pose. This checks
        # refreshes separately from the existing all-expired wall-clock gate.
        stale_pose = [r for r in records if 4 <= r["elapsed_s"] < 6
                      and max(float(r["fields"].get("iha", "nan")),
                              float(r["fields"].get("iia", "nan"))) > .25]
        checks["stale_receipt_pose_does_not_refresh_tracks"] = bool(stale_pose) and all(
            float(r["fields"].get(f"r{i}age", "nan")) + .251 >= max(
                float(r["fields"]["iha"]), float(r["fields"]["iia"]))
            for r in stale_pose for i in range(4)
            if f"r{i}source" in r["fields"])
    if models:
        checks["models_unchanged"] = all(
            hashlib.sha256(path.read_bytes()).hexdigest() == model_hashes[name]
            for name, path in models.items())
    if args.verifier_model:
        checks["verifier_loaded_without_failure"] = (
            "loaded optional ball verifier" in text
            and "verifier failed" not in text and "verifier disabled" not in text)
    report = {"checks": checks, "passed": all(checks.values()), "records": records,
              "binary_sha256": binary_hash,
              "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
              "domain": 142, "network_model_loaded": "loaded ball model" in text,
              "model_sha256": model_hashes,
              "synthetic_game_state": game.state,
              "sensor_age_diagnostics": args.trace,
              "recorded_trial": args.trial,
              "limits": "Settled RGB; synthetic self origin; no motion/extrinsic accuracy claim"}
    (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
