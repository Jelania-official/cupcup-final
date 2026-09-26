#!/usr/bin/env python3
"""
Small deterministic opponent styles for strategy regression.

This node is intentionally test-only.  It exercises the same BodyTask and
HeadTask interfaces as a team package, while keeping motion conservative so a
failure in cupcup is not hidden by an unrelated opponent crash.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from common.msg import BodyTask, GameData, HeadTask, ImuData, Location
from rclpy.node import Node


class ScriptedOpponent(Node):
    def __init__(self, robot: str, style: str):
        super().__init__(f"scripted_{robot}_{style}")
        self.robot = robot
        self.style = style
        self.color = "blue" if "blue" in robot else "red"
        self.attack_sign = 1.0 if self.color == "blue" else -1.0
        self.game_state = GameData.STATE_INIT
        self.location = Location()
        self.imu = ImuData()
        self.started = time.monotonic()
        self.last_log = 0.0
        self.body_publisher = self.create_publisher(BodyTask, f"/{robot}/task/body", 5)
        self.head_publisher = self.create_publisher(HeadTask, f"/{robot}/task/head", 5)
        self.create_subscription(GameData, "/sensor/game", self.game_update, 5)
        self.create_subscription(Location, f"/sensor/{robot}_location", self.location_update, 5)
        self.create_subscription(ImuData, f"/{robot}/sensor/imu", self.imu_update, 5)
        self.create_timer(0.10, self.tick)

    def game_update(self, message: GameData) -> None:
        self.game_state = message.state

    def location_update(self, message: Location) -> None:
        self.location = message

    def imu_update(self, message: ImuData) -> None:
        self.imu = message

    def tick(self) -> None:
        body = BodyTask()
        body.type = BodyTask.TASK_WALK
        body.count = 0
        head = HeadTask()
        elapsed = time.monotonic() - self.started
        head.yaw = 45.0 * math.sin(elapsed / 2.5)
        head.pitch = 28.0 if math.sin(elapsed / 5.0) < 0 else 45.0

        safe_to_move = (
            self.game_state == GameData.STATE_PLAY
            and self.imu.fall == ImuData.FALL_NONE
        )
        if safe_to_move:
            if self.style == "rush" and self.robot.endswith("_1"):
                # Press to a conservative point just inside the opponent half,
                # then stop; this tests obstruction/role arbitration without
                # turning the fixture into an unrealistic full-speed bot.
                attack_x = self.attack_sign * self.location.x
                if attack_x < 0.8:
                    body.count = 1
                    body.step = 0.018
                else:
                    body.count = 0
            elif self.style == "wall":
                # Hold a shallow own-half line.  A small lateral oscillation
                # tests that cupcup handles a moving blocker without chasing
                # it out of bounds.
                body.count = 1
                body.lateral = 0.012 * math.sin(elapsed / 4.0)
            elif self.style == "keeper":
                body.count = 0

        self.body_publisher.publish(body)
        self.head_publisher.publish(head)
        if elapsed - self.last_log > 5.0:
            self.get_logger().info(
                f"style={self.style} state={self.game_state} "
                f"loc=({self.location.x:.2f},{self.location.z:.2f}) "
                f"fall={self.imu.fall} step={body.step:.3f} lateral={body.lateral:.3f}"
            )
            self.last_log = elapsed


def main() -> int:
    if len(sys.argv) != 3 or sys.argv[2] not in {"rush", "wall", "keeper"}:
        print("usage: scripted_opponent.py <robot_name> <rush|wall|keeper>", file=sys.stderr)
        return 2
    rclpy.init()
    node = ScriptedOpponent(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
