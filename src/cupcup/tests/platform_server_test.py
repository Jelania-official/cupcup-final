#!/usr/bin/env python3
"""Unit checks for read-only live telemetry decoding."""

import sys
import json
import unittest
import tempfile
from pathlib import Path
from threading import Thread
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

SANDBOX_BINARY = sys.argv.pop(1)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from platform_server import LiveState, make_handler, experiment_options  # noqa: E402


class PlatformServerTest(unittest.TestCase):
    def test_custom_scene_requires_complete_finite_input(self) -> None:
        with self.assertRaises(ValueError):
            experiment_options({"initial_state_mode": ["custom"]})
        form = {"initial_state_mode": ["custom"]}
        for entity in ("red1", "red2", "blue1", "blue2", "ball"):
            for axis in (("x", "z", "vx", "vz") if entity == "ball" else ("x", "z", "yaw")):
                form[f"initial_{entity}_{axis}"] = ["0"]
        options = experiment_options(form)
        self.assertEqual(len(options[options.index("--initial-state") + 1].split()), 16)
        form["initial_ball_vx"] = ["nan"]
        with self.assertRaises(ValueError):
            experiment_options(form)

    def test_effective_ball_map_does_not_fabricate_a_direct_observation(self) -> None:
        state = LiveState()
        wire = "cupcup|ball=0|mbx=.2|mbz=-.3|mbage=.1|mbsource=teammate"
        state.receive_talk("red_2", wire)
        robot = state.snapshot()["robots"][0]
        self.assertEqual(robot["mapped_ball"], [.2, -.3])
        self.assertEqual(robot["mapped_ball_source"], "teammate")
        self.assertFalse(robot["sees_ball"])
        self.assertIsNone(robot["ball_x"])
        for suffix in ("|mbx=nan", "|mbz=5", "|mbage=-.1", "|mbage=.7"):
            state.receive_talk("red_2", wire + suffix)
            self.assertIsNone(state.snapshot()["robots"][0]["mapped_ball"])
        state.receive_talk("red_2", "cupcup|ball=1|bx=0|bz=0|bmap_age=.1")
        self.assertIsNone(state.snapshot()["robots"][0]["mapped_ball"])

    def test_effective_peer_geometry_is_separate_filtered_and_recorded(self) -> None:
        wire = ("cupcup|peer_obstacles=1|pobs0x=-0.5|pobs0z=0.1|pobs0conf=0.25|"
                "pobs0unc=1|pobs0age=0.2|pobs0team=blue")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "belief.jsonl"
            state = LiveState(path)
            state.receive_talk("red_1", wire)
            frame = json.loads(path.read_text())
            robot = frame["robots"][0]
            self.assertFalse(frame["truth_available"])
            self.assertEqual(robot["robot_tracks"], [])
            self.assertEqual(robot["peer_obstacle_tracks"], [{
                "x": -.5, "z": .1, "team": "blue", "confidence": .25,
                "uncertainty": 1, "age_s": .2, "source": "teammate_direct_number_square"}])
            for suffix in ("|pobs0x=nan", "|pobs0conf=0.5", "|pobs0age=-1",
                           "|pobs0age=0.7", "|pobs0team=unknown", "|pobs0unc=-1"):
                state.receive_talk("red_1", wire + suffix)
                self.assertEqual(state.snapshot()["robots"][0]["peer_obstacle_tracks"], [])
            state.receive_talk("red_1", "cupcup|id=1")
            self.assertEqual(state.snapshot()["robots"][0]["peer_obstacle_tracks"], [])

    def test_peer_obstacle_count_is_optional_bounded_and_recorded(self) -> None:
        state = LiveState()
        for wire, expected in (("cupcup|peer_obstacles=2", 2),
                               ("cupcup|peer_obstacles=0", 0),
                               ("cupcup|peer_obstacles=nan", None),
                               ("cupcup|peer_obstacles=-1", None),
                               ("cupcup|peer_obstacles=1.5", None),
                               ("cupcup|peer_obstacles=5", None), ("cupcup|id=1", None)):
            state.receive_talk("red_1", wire)
            self.assertEqual(state.snapshot()["robots"][0]["peer_obstacles"], expected)

    def test_controller_heading_is_distinct_from_policy_target_and_old_unknown(self) -> None:
        state = LiveState()
        state.receive_talk("red_1", "cupcup|kx=-2|kz=0|aim_yaw=160|aim_offset=-20")
        robot = state.snapshot()["robots"][0]
        self.assertEqual(robot["kick_target"], [-2, 0])
        self.assertEqual(robot["aim_yaw"], 160)
        self.assertEqual(robot["aim_offset"], -20)
        for wire in ("cupcup|kx=-2|kz=0", "cupcup|aim_yaw=nan|aim_offset=inf"):
            state.receive_talk("red_1", wire)
            robot = state.snapshot()["robots"][0]
            self.assertIsNone(robot["aim_yaw"])
            self.assertIsNone(robot["aim_offset"])

    def test_local_ball_range_source_is_preserved_without_claiming_map_source(self) -> None:
        state = LiveState()
        for source in ("ground_ray", "radius", "invented"):
            state.receive_talk("red_1", "cupcup|bdist=0.2|ball_range_source=" + source)
            robot = state.snapshot()["robots"][0]
            self.assertEqual(robot["ball_distance"], 0.2)
            self.assertEqual(robot["ball_range_source"],
                             source if source != "invented" else "unknown")
        state.receive_talk("red_1", "cupcup|bdist=nan")
        robot = state.snapshot()["robots"][0]
        self.assertIsNone(robot["ball_distance"])
        self.assertEqual(robot["ball_range_source"], "unknown")

    def test_robot_tracks_preserve_team_unknown_and_uncertainty(self) -> None:
        state = LiveState()
        state.receive_talk("red_1", "cupcup|px=1|pz=0|why=ball-owner|robots=2|"
                           "r0id=7|r0team=blue|r0x=2|r0z=1|r0conf=0.2|r0unc=1.5|r0age=0.1|"
                           "r0source=number_square|"
                           "r1id=8|r1team=unknown|r1x=0|r1z=1|r1conf=0.1|r1unc=2|r1age=0.2")
        robot = state.snapshot()["robots"][0]
        self.assertEqual(robot["reason"], "ball-owner")
        self.assertEqual(len(robot["robot_tracks"]), 2)
        self.assertEqual(robot["robot_tracks"][0]["team"], "blue")
        self.assertEqual(robot["robot_tracks"][0]["uncertainty"], 1.5)
        self.assertEqual(robot["robot_tracks"][0]["latest_source"], "number_square")
        self.assertEqual(robot["robot_tracks"][1]["team"], "unknown")
        self.assertEqual(robot["robot_tracks"][1]["latest_source"], "unknown")

    def test_recording_persists_estimates_without_truth(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "belief.jsonl"
            state = LiveState(path)
            state.receive_talk("red_1", "cupcup|px=1|pz=0|pose_age=0.1")
            lines = path.read_text().splitlines()
            self.assertEqual(len(lines), 1)
            frame = json.loads(lines[0])
            self.assertFalse(frame["truth_available"])
            self.assertEqual(frame["robots"][0]["x"], 1)
            self.assertEqual(state.replay()["model"], "live-belief-v1")

    def test_extra_parameters_are_validated(self) -> None:
        args = experiment_options({"opponent_speed": ["0.8"], "mode": ["oracle"]})
        self.assertIn("--opponent-speed", args)
        self.assertIn("oracle", args)
        self.assertIn("--kick-hysteresis", args)
        self.assertIn("--defense-mode", args)
        self.assertIn("--vision-model", args)
        self.assertIn("legacy", experiment_options({"vision_model": ["legacy"]}))
        for values in ({"delay": ["nan"]}, {"mode": ["truth-hack"]}, {"speed": ["20"]},
                       {"kick_hysteresis": ["nan"]}, {"defense_mode": ["magic"]},
                       {"vision_model": ["magic"]}):
            with self.assertRaises(ValueError):
                experiment_options(values)

    def test_only_cupcup_talk_is_decoded_without_truth(self) -> None:
        state = LiveState()
        state.receive_talk(
            "red_1",
            "cupcup|id=1|role=forward|state=APPROACH|tac=CHASE|"
            "ball=1|healthy=1|"
            "px=1.0|pz=-0.5|pyaw=10|pose_age=0.1|bx=0.8|bz=-0.3|"
            "bmap_age=0.2|conf=0.8|tx=0.7|tz=-0.2|kx=-1.5|kz=0.4|"
            "rx=-1.0|rz=0.4|rc=0.2|ra=0.3",
        )
        state.receive_talk("blue_1", "unirobot|px=0|pz=0")
        snapshot = state.snapshot()
        self.assertEqual(len(snapshot["robots"]), 1)
        self.assertEqual(snapshot["robots"][0]["name"], "red_1")
        self.assertEqual(snapshot["robots"][0]["role"], "forward")
        self.assertEqual(snapshot["robots"][0]["ball_x"], 0.8)
        self.assertEqual(snapshot["robots"][0]["action"], "CHASE")
        self.assertEqual(snapshot["robots"][0]["target"], [0.7, -0.2])
        self.assertEqual(snapshot["robots"][0]["kick_target"], [-1.5, 0.4])
        self.assertEqual(
            snapshot["robots"][0]["robot_candidate"]["confidence"], 0.2
        )
        self.assertFalse(snapshot["truth_available"])

    def test_malformed_numbers_become_missing_estimates(self) -> None:
        state = LiveState()
        state.receive_talk(
            "red_2", "cupcup|id=2|role=defender|px=nan|pz=bad|tx=inf|tz=0|"
            "rx=nan|rz=inf|kx=nan|kz=1"
        )
        robot = state.snapshot()["robots"][0]
        self.assertIsNone(robot["x"])
        self.assertIsNone(robot["z"])
        self.assertIsNone(robot["target"])
        self.assertIsNone(robot["kick_target"])
        self.assertIsNone(robot["robot_candidate"])

    def test_symlink_installed_assets_and_path_restriction(self) -> None:
        source = Path(__file__).resolve().parents[1] / "tools" / "web"
        with tempfile.TemporaryDirectory() as directory:
            web_root = Path(directory)
            for name in ("index.html", "app.js", "style.css"):
                (web_root / name).symlink_to(source / name)
            server = ThreadingHTTPServer(
                ("127.0.0.1", 0),
                make_handler(LiveState(), web_root, Path(SANDBOX_BINARY)),
            )
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            base = f"http://127.0.0.1:{server.server_port}"
            try:
                for path, name in (("/", "index.html"),
                                   ("/index.html", "index.html"),
                                   ("/app.js", "app.js"),
                                   ("/style.css", "style.css")):
                    with urlopen(base + path) as response:
                        self.assertEqual(response.read(), (source / name).read_bytes())
                for path in ("/../package.xml", "/%2e%2e/package.xml",
                             "//etc/passwd", "/secret.txt"):
                    with self.assertRaises(HTTPError) as error:
                        urlopen(base + path)
                    self.assertEqual(error.exception.code, 403)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)

    def test_http_modes_serve_live_data_sandbox_and_static_page(self) -> None:
        web_root = Path(__file__).resolve().parents[1] / "tools" / "web"
        binary = Path(SANDBOX_BINARY)
        server = ThreadingHTTPServer(
            ("127.0.0.1", 0), make_handler(LiveState(), web_root, binary)
        )
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(f"{base}/") as response:
                self.assertIn(b"cupcup", response.read())
            with urlopen(f"{base}/api/live") as response:
                live = json.loads(response.read())
                self.assertFalse(live["truth_available"])
            request = Request(
                f"{base}/api/sandbox",
                data=urlencode({
                    "duration": "1", "seed": "9", "opponent": "block",
                    "noise": "0.12", "kick_speed": "2",
                    "kick_hysteresis": "0", "defense_mode": "fixed",
                }).encode(),
                method="POST",
            )
            with urlopen(request) as response:
                self.assertEqual(response.status, 200)
                scenario = json.loads(response.read())
                self.assertTrue(scenario["frames"])
                self.assertEqual(scenario["cupcup_color"], "red")
                self.assertEqual(scenario["parameters"]["kick_direction_hysteresis"], 0)
                self.assertEqual(scenario["parameters"]["defense_mode"], "fixed")
                self.assertEqual(scenario["abstract_opponent_revision"], 2)
            batch = Request(
                f"{base}/api/evaluate",
                data=urlencode({
                    "duration": "5", "seed": "9", "trials": "2",
                    "opponent": "shared", "noise": "0.12",
                    "kick_speed": "2",
                }).encode(),
                method="POST",
            )
            with urlopen(batch) as response:
                report = json.loads(response.read())
                self.assertEqual(report["trials"], 2)
                self.assertEqual(report["matches"], 4)
                self.assertEqual(len(report["results"]), 4)
                self.assertEqual(
                    [item["color"] for item in report["results"]],
                    ["red", "blue", "red", "blue"],
                )
            bad_request = Request(
                f"{base}/api/sandbox",
                data=b"duration=1&seed=1&opponent=../../bad",
                method="POST",
            )
            with self.assertRaises(HTTPError) as error:
                urlopen(bad_request)
            self.assertEqual(error.exception.code, 400)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
