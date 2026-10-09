#!/usr/bin/env python3
"""Headless smoke-test referee for the normal 2v2 Webots chain."""

import time
import os

import rclpy
from common.msg import FieldData, GameData, Player
from common.srv import GetColor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MockGamectrl(Node):
    def __init__(self):
        super().__init__("cupcup_mock_gamectrl")
        self.cupcup_color = os.environ.get("CUPCUP_MOCK_COLOR", "red")
        # The referee fixture can put any test-only team opposite cupcup.  The
        # real referee remains authoritative in competition; this switch only
        # makes scripted-opponent regression runs use the same color service.
        self.opponent_team = os.environ.get("CUPCUP_MOCK_OPPONENT", "unirobot")
        self.publisher = self.create_publisher(GameData, "/sensor/game", 5)
        self.service = self.create_service(GetColor, "gamectrl/get_color", self.get_color)
        self.field_subscription = self.create_subscription(
            FieldData, "/sensor/field", self.field_update, 5)
        self.latest_field = None
        self.player_wait_started = {"red": [None, None], "blue": [None, None]}
        self.started = time.monotonic()
        self.timer = self.create_timer(0.05, self.publish_game)
        self.red_score = 0
        self.blue_score = 0
        self.restart_started = None
        self.last_field_event = FieldData.BALL_NORMAL
        self.last_reported_scores = (-1, -1)

    def field_update(self, message):
        self.latest_field = message
        event_states = {
            FieldData.BALL_OUT,
            FieldData.BALL_NOMOVE,
            FieldData.BALL_GOAL,
            FieldData.BALL_OUTLINE,
        }
        if message.ball_state not in event_states:
            self.last_field_event = FieldData.BALL_NORMAL
            return
        if message.ball_state == self.last_field_event:
            return
        self.last_field_event = message.ball_state
        self.red_score = message.red_score
        self.blue_score = message.blue_score
        self.get_logger().info(
            f"field event={message.ball_state} score={self.red_score}:{self.blue_score}; restart")
        if self.restart_started is None:
            self.restart_started = time.monotonic()

    def get_color(self, request, response):
        if request.team == "cupcup":
            response.color = self.cupcup_color
        elif request.team == self.opponent_team:
            response.color = "blue" if self.cupcup_color == "red" else "red"
        else:
            response.color = "invalid"
        return response

    def publish_game(self):
        elapsed = time.monotonic() - self.started
        restart_elapsed = None if self.restart_started is None else (
            time.monotonic() - self.restart_started)
        message = GameData()
        message.mode = GameData.MODE_NORM
        message.remain_time = max(0, 900 - int(elapsed))
        message.second_time = int(elapsed)
        message.red_score = self.red_score
        message.blue_score = self.blue_score
        if restart_elapsed is not None:
            if restart_elapsed < 2.0:
                message.state = GameData.STATE_PAUSE
            elif restart_elapsed < 4.0:
                message.state = GameData.STATE_INIT
            elif restart_elapsed < 6.0:
                message.state = GameData.STATE_READY
            else:
                self.restart_started = None
                message.state = GameData.STATE_PLAY
                self.get_logger().info("restart -> PLAY")
        else:
            message.state = (
                GameData.STATE_INIT
                if elapsed < 4.0
                else GameData.STATE_READY
                if elapsed < 6.0 or os.environ.get("CUPCUP_PERCEPTION_CALIBRATION")
                else GameData.STATE_PLAY
            )
        scores = (self.red_score, self.blue_score)
        if scores != self.last_reported_scores:
            self.last_reported_scores = scores
            self.get_logger().info(f"score={scores[0]}:{scores[1]}")
        for index in range(2):
            message.red_players[index].name = f"red_{index + 1}"
            message.blue_players[index].name = f"blue_{index + 1}"
            message.red_players[index].state = Player.PLAYER_NORMAL
            message.blue_players[index].state = Player.PLAYER_NORMAL
            message.red_players[index].wait_time = 0
            message.blue_players[index].wait_time = 0
        if message.state == GameData.STATE_PLAY and self.latest_field is not None:
            for color in ("red", "blue"):
                field_players = getattr(self.latest_field, f"{color}_players")
                game_players = getattr(message, f"{color}_players")
                for index, field_player in enumerate(field_players):
                    wait_started = self.player_wait_started[color][index]
                    if wait_started is not None:
                        if elapsed - wait_started < 30.0:
                            game_players[index].state = Player.PLAYER_WAIT
                            continue
                        self.player_wait_started[color][index] = None
                    if field_player.state == Player.PALYER_OUT:
                        self.player_wait_started[color][index] = elapsed
                        game_players[index].state = Player.PLAYER_WAIT
                        game_players[index].wait_time = message.remain_time
                        self.get_logger().info(
                            f"player_out team={color} id={index + 1} penalty_seconds=30")
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = MockGamectrl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
