#!/usr/bin/env python3
"""
Isolated nonowner-search contract, or archived arrival-only experiment.

Default search tests current behavior. Arrival requires the rejected candidate
binary and --contract arrival. Neither mode proves motor motion or vision accuracy.
"""

import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from common.msg import BodyTask, GameData, HeadAngles, ImuData, Location, Talk
from common.srv import GetColor
from rclpy.node import Node
from sensor_msgs.msg import Image
from analyze_world_trace import parse_talk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--color", choices=("red", "blue"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", choices=("search", "arrival"), default="search")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "142":
        parser.error("only run in dedicated fixture domain 142")
    args.output.mkdir(parents=True, exist_ok=False)
    digest = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    robot = f"{args.color}_2"
    rclpy.init()
    node = Node("cupcup_observer_heading_fixture")

    def color_service(request, response):
        response.color = args.color if request.team == "cupcup" else "invalid"
        return response

    service = node.create_service(GetColor, "gamectrl/get_color", color_service)
    pubs = {"game": node.create_publisher(GameData, "/sensor/game", 2),
            "imu": node.create_publisher(ImuData, f"/{robot}/sensor/imu", 2),
            "head": node.create_publisher(HeadAngles, f"/{robot}/sensor/joint/head", 2),
            "location": node.create_publisher(Location, f"/sensor/{robot}_location", 2),
            "image": node.create_publisher(Image, f"/{robot}/sensor/image", 2),
            "talk": node.create_publisher(Talk, f"/{args.color}_1/talk/talk_str", 2)}
    records = []
    peer_publish_elapsed = []
    latest_talk = {}
    started = time.monotonic()

    def receive(body):
        records.append({"elapsed_s": time.monotonic() - started, "type": body.type,
                        "step": body.step, "lateral": body.lateral, "turn": body.turn,
                        "talk": dict(latest_talk)})

    def receive_talk(message):
        latest_talk.clear()
        latest_talk.update(parse_talk(message.talk_str))

    sub = node.create_subscription(BodyTask, f"/{robot}/task/body", receive, 5)
    talk_sub = node.create_subscription(Talk, f"/{robot}/talk/talk_str", receive_talk, 5)
    game = GameData()
    game.mode, game.state = GameData.MODE_NORM, GameData.STATE_PLAY
    for player in game.red_players + game.blue_players:
        player.state = player.PLAYER_NORMAL
    imu, head, location = ImuData(), HeadAngles(), Location()
    imu.yaw = 0.0 if args.color == "red" else 180.0
    location.x = 1.55 if args.color == "red" else -1.55
    image = Image()
    image.width, image.height, image.encoding = 320, 240, "rgb8"
    image.step, image.data = 960, bytes(320 * 240 * 3)
    peer = Talk()
    peer.talk_str = ("cupcup|id=1|role=forward|healthy=1|ball=1|active=1|claim=1|"
                     "bx=0|bz=0|bmap_age=0.1|age=0.1|conf=0.9|bdist=0.2")
    log = (args.output / "player.log").open("w")
    child = subprocess.Popen([str(args.binary), "cupcup_2"], stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    try:
        previous = -1.0
        while time.monotonic() - started < 7 and child.poll() is None:
            elapsed = time.monotonic() - started
            if elapsed - previous >= .1:
                for key, value in (("game", game), ("imu", imu), ("head", head),
                                   ("location", location), ("image", image)):
                    pubs[key].publish(value)
                # Other DDS writes may block: do not use a timestamp sampled
                # before those writes to decide whether the peer is still live.
                peer_elapsed = time.monotonic() - started
                if peer_elapsed < 4:
                    pubs["talk"].publish(peer)
                    peer_publish_elapsed.append(peer_elapsed)
                previous = elapsed
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        log.close()
        node.destroy_subscription(sub)
        node.destroy_subscription(talk_sub)
        node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
    fresh = [r for r in records if 2.5 <= r["elapsed_s"] < 4]
    stale = [r for r in records if 5.5 <= r["elapsed_s"] < 7]
    checks = {"no_translation_or_kick": bool(records) and all(
                  r["type"] == BodyTask.TASK_WALK and r["step"] == r["lateral"] == 0
                  for r in records),
              "binary_unchanged": digest == hashlib.sha256(args.binary.read_bytes()).hexdigest()}
    if args.contract == "search":
        checks["shared_ball_map_visible_without_local_detection"] = any(
            r["talk"].get("mbsource") == "teammate" and
            "mbx" in r["talk"] and "mbz" in r["talk"] and
            r["talk"].get("ball") == "0" and "bx" not in r["talk"] for r in fresh)
        checks["expired_diagnostic_map_is_absent"] = bool(stale) and all(
            "mbx" not in r["talk"] and "mbz" not in r["talk"] for r in stale)
        checks["fresh_ball_uses_navigation_not_search"] = bool(fresh) and all(
            abs(r["turn"]) <= 7.01 for r in fresh)
        checks["expired_ball_stops_translation_and_searches"] = bool(stale) and all(
            abs(r["turn"]) >= 7.9 for r in stale)
    else:
        checks["fresh_ball_turns_at_anchor"] = bool(fresh) and all(
            abs(r["turn"]) >= 5 for r in fresh)
        checks["expired_ball_holds_heading"] = bool(stale) and all(
            abs(r["turn"]) < .01 for r in stale)
    report = {"checks": checks, "passed": all(checks.values()), "records": records,
              "peer_publish_elapsed_s": peer_publish_elapsed,
              "color": args.color, "contract": args.contract, "domain": 142,
              "binary_sha256": digest,
              "limits": "Fixed synthetic sensors; command contract, not motion/vision accuracy"}
    (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
