#!/usr/bin/env python3
"""Optional real-browser checks; browser dependencies never enter the robot package."""

import argparse
import hashlib
import importlib.util
import json
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

from playwright.sync_api import sync_playwright, expect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--replay", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--web-root", type=Path, help="optional installed web assets to verify")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    tools = Path(__file__).resolve().parents[1] / "tools"
    spec = importlib.util.spec_from_file_location("platform_server", tools / "platform_server.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    state = module.LiveState()
    web = args.web_root or tools / "web"
    web_hashes = {name: hashlib.sha256((web / name).read_bytes()).hexdigest()
                  for name in ("app.js", "index.html", "style.css")}
    replay_bytes = args.replay.read_bytes()
    replay_file = {"name": args.replay.name, "mimeType": "application/octet-stream",
                   "buffer": replay_bytes}
    server = ThreadingHTTPServer(("127.0.0.1", 0), module.make_handler(
        state, web, args.binary))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    errors, checks = [], []
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch()
            try:
                page = browser.new_page(viewport={"width": 1280, "height": 1000})
                page.on("pageerror", lambda error: errors.append(str(error)))
                # Hold the initial live response until after import, reproducing
                # a pending request without depending on network timing.
                page.add_init_script("""(() => {
                    const fetchLive = window.fetch;
                    let first = true;
                    window.fetch = (...args) => {
                        if (first && args[0] === '/api/live') {
                            first = false;
                            return new Promise(resolve => {
                                window.releaseInitialLive = () => resolve(new Response(
                                    JSON.stringify({ros_available: false, robots: [],
                                                    score: {red: 0, blue: 0}}),
                                    {headers: {'Content-Type': 'application/json'}}));
                            });
                        }
                        return fetchLive(...args);
                    };
                })();""")
                page.goto(f"http://127.0.0.1:{server.server_port}")
                expect(page.locator("#connection")).to_contain_text("正在连接")
                page.locator("#live-import").set_input_files(replay_file)
                expect(page.locator("#live-status")).to_contain_text("已加载")
                page.evaluate("window.releaseInitialLive()")
                page.locator("#live-timeline").press("End")
                expect(page.locator("#connection")).to_contain_text("离线观测回放")
                expect(page.locator("#live-status")).to_contain_text("已加载")
                expect(page.locator("#live-replay-time")).to_contain_text("s")
                page.locator("#live-import").set_input_files({
                    "name": "invalid.json", "mimeType": "application/json",
                    "buffer": json.dumps({"frames": [{"mode": "live", "truth_available": False,
                                                      "robots": [{"name": 42}],
                                                      "score": {}}]}).encode()})
                expect(page.locator("#live-status")).to_contain_text("不是本平台")
                fixture = {"frames": [{"mode": "live", "truth_available": False,
                                       "elapsed_s": 1, "score": {"red": 0, "blue": 0},
                                       "robots": [{"name": "red_1", "x": 1, "z": 0,
                                                   "action": "CHASE", "ball_x": 0,
                                                   "ball_z": 0, "ball_age": 0.1,
                                                   "kick_x": -1.5, "kick_z": 0.5}]}]}
                page.locator("#live-import").set_input_files({
                    "name": "legacy-kick.json", "mimeType": "application/json",
                    "buffer": json.dumps(fixture).encode()})
                expect(page.locator("#live-status")).to_contain_text("已加载")
                assert page.locator("#live-field").evaluate("""canvas => {
                    const pixels = canvas.getContext('2d').getImageData(
                        0, 0, canvas.width, canvas.height).data;
                    for (let i = 0; i < pixels.length; i += 4)
                        if (pixels[i] > 240 && pixels[i+1] > 170 && pixels[i+1] < 205 &&
                            pixels[i+2] > 85 && pixels[i+2] < 125) return true;
                    return false;
                }""")
                fixture["frames"][0]["robots"][0].update({
                    "aim_yaw": 90, "aim_offset": 20, "peer_obstacles": 1,
                    "peer_obstacle_tracks": [{"x": -.5, "z": -.5, "team": "blue",
                                              "age_s": .1, "confidence": .25,
                                              "uncertainty": .3}]})
                page.locator("#live-import").set_input_files({
                    "name": "execution-heading.json", "mimeType": "application/json",
                    "buffer": json.dumps(fixture).encode()})
                expect(page.locator("#live-status")).to_contain_text("对准目标 90")
                expect(page.locator("#live-status")).to_contain_text("队友障碍 1")
                purple_pixels = """canvas => {
                    const pixels = canvas.getContext('2d').getImageData(
                        0, 0, canvas.width, canvas.height).data;
                    for (let i = 0; i < pixels.length; i += 4)
                        if (pixels[i] > 200 && pixels[i] < 225 &&
                            pixels[i+1] > 150 && pixels[i+1] < 180 &&
                            pixels[i+2] > 240) return true;
                    return false;
                }"""
                assert page.locator("#live-field").evaluate(purple_pixels)
                assert page.locator("#live-field").evaluate("""canvas => {
                    const pixels = canvas.getContext('2d').getImageData(
                        0, 0, canvas.width, canvas.height).data;
                    for (let i = 0; i < pixels.length; i += 4)
                        if (pixels[i] > 115 && pixels[i] < 140 &&
                            pixels[i+1] > 215 && pixels[i+2] > 225) return true;
                    return false;
                }""")
                page.screenshot(path=str(args.output / "execution-heading.png"), full_page=True)
                second = json.loads(json.dumps(fixture["frames"][0]))
                second["elapsed_s"] = 2
                second["robots"][0].update({
                    "aim_yaw": -90, "aim_offset": -20, "peer_obstacles": 0,
                    "peer_obstacle_tracks": []})
                fixture["frames"].append(second)
                page.locator("#live-import").set_input_files({
                    "name": "execution-heading-timeline.json", "mimeType": "application/json",
                    "buffer": json.dumps(fixture).encode()})
                page.locator("#live-timeline").press("End")
                expect(page.locator("#live-status")).to_contain_text("对准目标 -90")
                expect(page.locator("#live-status")).to_contain_text("队友障碍 0")
                assert not page.locator("#live-field").evaluate(purple_pixels)
                page.locator("#live-timeline").press("Home")
                expect(page.locator("#live-status")).to_contain_text("对准目标 90")
                page.locator("#live-step").click()
                expect(page.locator("#live-timeline")).to_have_value("1")
                page.locator("#live-start").click()
                expect(page.locator("#live-timeline")).to_have_value("0")
                page.locator("#live-speed").select_option("4")
                page.locator("#live-play").click()
                expect(page.locator("#live-timeline")).to_have_value("1")
                expect(page.locator("#live-play")).to_have_text("播放")
                with page.expect_download() as download:
                    page.locator("#live-export").click()
                exported_live = args.output / "live-export.json"
                download.value.save_as(str(exported_live))
                assert json.loads(exported_live.read_text())["frames"] == fixture["frames"]
                checks.append("timestamp replay controls and export of viewed recording")
                checks.append("controller target heading distinct from shared policy destination")
                assert page.evaluate("""() => {
                    const canvas = document.createElement('canvas');
                    const ctx = canvas.getContext('2d');
                    let squares = 0;
                    const strokeRect = ctx.strokeRect.bind(ctx);
                    ctx.strokeRect = (...args) => {
                        if (args[2] === 8 && args[3] === 8) squares++;
                        strokeRect(...args);
                    };
                    const robot = {x: 0, z: 0, yaw: 0, mapped_ball: [.2, .3],
                        mapped_ball_age: .1, message_age_s: .1, sees_ball: false};
                    const paint = () => drawRobot(ctx, 10, (x,z) => [x*10,z*10],
                        robot, '#ff7c75', true);
                    paint();
                    if (squares !== 1) return false;
                    robot.message_age_s = .6;
                    paint();
                    return squares === 1;
                }""")
                checks.append("effective ball map renders without local sight and expires")
                layered = {"frames": [{"mode": "live", "truth_available": False,
                                       "elapsed_s": 1, "score": {"red": 0, "blue": 0},
                                       "robots": [{"name": "red_1", "x": 1, "z": 0,
                                                   "pose_age": .1, "message_age_s": .1,
                                                   "mapped_ball": [.2, .3],
                                                   "mapped_ball_age": .1,
                                                   "mapped_ball_source": "teammate"},
                                                  {"name": "blue_2", "x": -1, "z": 0,
                                                   "pose_age": 3, "message_age_s": .1}]}]}
                page.locator("#live-import").set_input_files({
                    "name": "layers.json", "mimeType": "application/json",
                    "buffer": json.dumps(layered).encode()})
                page.locator("#live-observer").select_option("red_1")
                expect(page.locator("#live-details .object-card")).to_have_count(1)
                expect(page.locator("#live-details")).to_contain_text("原始球观测坐标：未记录")
                expect(page.locator("#live-details")).to_contain_text("可见时发布的球图：未知")
                expect(page.locator("#live-details")).to_contain_text("末次球图来源：teammate")
                page.locator("#live-layer-map").uncheck()
                with page.expect_download() as download:
                    page.locator("#live-snapshot").click()
                snapshot = args.output / "live-analysis.json"
                download.value.save_as(str(snapshot))
                analysis = json.loads(snapshot.read_text())
                assert analysis["truth_available"] is False
                assert analysis["view"]["map"] is False
                assert analysis["view"]["observer"] == "red_1"
                assert analysis["context"]["recording"] == "layers.json"
                assert analysis["frame"] == layered["frames"][0]
                page.locator("#live-layer-map").check()
                page.locator("#live-observer").select_option("blue_2")
                expect(page.locator("#live-details")).to_contain_text("过期")
                expect(page.locator("#live-details")).to_contain_text("年龄未知")
                page.locator("#live-observer").select_option("all")
                position = page.locator("#live-field").evaluate("""canvas => {
                    const r = canvas.getBoundingClientRect();
                    const s = Math.min((r.width-50)/9, (r.height-50)/6);
                    return {x: (r.width-9*s)/2 + 5.5*s, y: r.height/2};
                }""")
                page.locator("#live-field").click(position=position)
                expect(page.locator("#live-observer")).to_have_value("red_1")
                page.locator("#live-observer").select_option("all")
                checks.append("live layers, provenance, unknown/stale details and snapshot")
                page.locator("#live-import").set_input_files(replay_file)
                expect(page.locator("#live-status")).to_contain_text("已加载")
                page.screenshot(path=str(args.output / "live-replay.png"), full_page=True)
                checks.append("recorded belief import, timeline and rendering")
                page.get_by_role("button", name="独立策略沙盒", exact=True).click()
                page.locator("#calibration-import").set_input_files(str(args.calibration))
                expect(page.locator("#calibration-status")).to_contain_text("已导入")
                page.locator('[name="duration"]').fill("20")
                page.locator('[name="kick_hysteresis"]').fill("0")
                page.locator('[name="defense_mode"]').select_option("fixed")
                page.locator(".scene-editor summary").click()
                page.locator("#scene-preview").click(position={"x": 300, "y": 200})
                expect(page.locator('[name="initial_state_mode"]')).to_have_value("custom")
                page.locator('[name="initial_ball_vx"]').fill(".6")
                page.locator('[name="initial_red1_yaw"]').fill("120")
                assert page.locator("#sandbox-controls").evaluate("form => form.checkValidity()")
                page.get_by_role("button", name="运行 / 重置", exact=True).click()
                expect(page.locator("#play-toggle")).to_be_enabled()
                expect(page.locator("#sandbox-metrics")).to_contain_text("平均球位推进")
                expect(page.locator("#sandbox-metrics")).to_contain_text("机器人被阻挡")
                expect(page.locator("#sandbox-metrics")).to_contain_text("准备被打断")
                page.get_by_role("button", name="单步", exact=True).click()
                expect(page.locator("#timeline")).to_have_value("1")
                page.locator("#timeline").press("End")
                expect(page.locator("#sim-time")).not_to_have_text("0.0 s · PLAY")
                with page.expect_download() as download:
                    page.get_by_role("button", name="导出实验", exact=True).click()
                exported = args.output / "sandbox-export.json"
                download.value.save_as(str(exported))
                experiment = json.loads(exported.read_text())
                assert experiment["frames"] and experiment["calibrated"] is False
                assert experiment["model"] == "bounded-2d-v3"
                assert experiment["abstract_opponent_revision"] == 2
                assert experiment["ball_contact_revision"] == 2
                assert experiment["horizontal_vision_revision"] == 2
                assert experiment["parameters"]["vision_model"] == "pan"
                assert experiment["parameters"]["kick_direction_hysteresis"] == 0
                assert experiment["parameters"]["defense_mode"] == "fixed"
                state_values = experiment["parameters"]["initial_state"]
                assert state_values[2] == 120 and state_values[14] == .6
                assert experiment["frames"][0]["ball"] == state_values[12:14]
                assert experiment["frames"][0]["ball_velocity"] == [.6, 0]
                checks.append("canvas scene editing, custom positions, heading and ball velocity")
                assert "mapped_ball" in experiment["frames"][-1]["red"][0]
                assert "peer_obstacle_tracks" in experiment["frames"][-1]["red"][0]
                assert experiment["parameter_provenance"]["sha256"] == hashlib.sha256(
                    args.calibration.read_bytes()).hexdigest()
                expected = json.loads(args.calibration.read_text())["nominal_sandbox_parameters"]
                assert abs(experiment["parameters"]["max_speed_mps"] - expected["speed"]) < 1e-7
                checks.append("sandbox run, step, timeline and exported experiment")
                page.locator('[name="seed"]').fill("999")
                page.locator("#sandbox-import").set_input_files(exported)
                expect(page.locator("#calibration-status")).to_contain_text("已恢复实验输入")
                expect(page.locator('[name="seed"]')).to_have_value(
                    experiment["experiment_inputs"]["seed"])
                expect(page.locator("#timeline")).to_have_value("0")
                page.locator("#sandbox-speed").select_option("1")
                page.locator("#play-toggle").click()
                page.wait_for_timeout(650)
                page.locator("#play-toggle").click()
                assert 2 <= int(page.locator("#timeline").input_value()) <= 5
                page.locator("#sandbox-start").click()
                expect(page.locator("#timeline")).to_have_value("0")
                page.get_by_role("button", name="运行 / 重置", exact=True).click()
                expect(page.locator("#sandbox-metrics")).to_contain_text("策略内核")
                expect(page.get_by_role("button", name="运行 / 重置", exact=True)).to_be_enabled()
                with page.expect_download() as download:
                    page.locator("#sandbox-export").click()
                rerun = args.output / "sandbox-rerun.json"
                download.value.save_as(str(rerun))
                assert json.loads(rerun.read_text()) == experiment
                bad = json.loads(exported.read_text())
                bad["experiment_inputs"]["speed"] = "-1"
                page.locator("#sandbox-import").set_input_files({
                    "name": "bad-inputs.json", "mimeType": "application/json",
                    "buffer": json.dumps(bad).encode()})
                expect(page.locator("#sandbox-metrics")).to_contain_text("实验参数无效")
                with page.expect_download() as download:
                    page.locator("#sandbox-export").click()
                unchanged = args.output / "sandbox-after-invalid.json"
                download.value.save_as(str(unchanged))
                assert json.loads(unchanged.read_text()) == experiment
                checks.append("experiment reload, timed playback, exact rerun, "
                              "invalid import isolation")
                page.locator("#sandbox-layer-truth").uncheck()
                expect(page.locator("#sandbox-details")).not_to_contain_text("模拟真值")
                assert page.evaluate("""() => {
                    const canvas = document.querySelector('#sandbox-field');
                    const ctx = canvas.getContext('2d');
                    const labels = [], original = ctx.fillText;
                    ctx.fillText = (...args) => { labels.push(args[0]); };
                    try { drawSandbox(); } finally { ctx.fillText = original; }
                    return labels.length === 0; // Initial map is unknown, no truth fallback.
                }""")
                mapped_index = page.evaluate("""sandbox.frames.findIndex(f =>
                    Array.isArray(f.red[0].mapped_self))""")
                assert mapped_index > 0
                page.locator("#timeline").evaluate("""(input, value) => {
                    input.value = value; input.dispatchEvent(new Event('input'));
                }""", mapped_index)
                page.locator("#sandbox-observer").select_option("red_1")
                expect(page.locator("#sandbox-details .object-card")).to_have_count(1)
                expect(page.locator("#sandbox-details")).to_contain_text("自机估计")
                with page.expect_download() as download:
                    page.locator("#sandbox-snapshot").click()
                snapshot = args.output / "sandbox-analysis.json"
                download.value.save_as(str(snapshot))
                analysis = json.loads(snapshot.read_text())
                assert analysis["view"]["truth"] is False
                assert analysis["view"]["observer"] == "red_1"
                assert analysis["context"]["seed"] == experiment["seed"]
                assert analysis["context"]["parameters"] == experiment["parameters"]
                assert analysis["frame"] == experiment["frames"][mapped_index]
                with page.expect_download() as download:
                    page.locator("#sandbox-export").click()
                filtered = args.output / "sandbox-after-view-change.json"
                download.value.save_as(str(filtered))
                assert json.loads(filtered.read_text()) == experiment
                page.locator("#sandbox-layer-truth").check()
                page.locator("#sandbox-observer").select_option("all")
                checks.append("sandbox truth separation, unknown map, observer, read-only layers")
                historical = json.loads(exported.read_text())
                historical.pop("experiment_inputs")
                historical["recording_mode"] = "final"
                historical["frames"] = historical["frames"][-1:]
                page.locator("#sandbox-import").set_input_files({
                    "name": "historical-final.json", "mimeType": "application/json",
                    "buffer": json.dumps(historical).encode()})
                expect(page.locator("#calibration-status")).to_contain_text("仅供回放")
                expect(page.locator("#calibration-status")).to_contain_text("仅末帧快照")
                malformed = json.loads(exported.read_text())
                malformed["frames"][0]["red"][0]["peer_obstacle_tracks"] = {}
                page.locator("#sandbox-import").set_input_files({
                    "name": "malformed-tracks.json", "mimeType": "application/json",
                    "buffer": json.dumps(malformed).encode()})
                expect(page.locator("#sandbox-metrics")).to_contain_text("不是有效")
                with page.expect_download() as download:
                    page.locator("#sandbox-export").click()
                kept = args.output / "sandbox-kept-history.json"
                download.value.save_as(str(kept))
                assert json.loads(kept.read_text()) == historical
                page.locator("#sandbox-import").set_input_files(exported)
                expect(page.locator("#calibration-status")).to_contain_text("已恢复实验输入")
                checks.append("historical final snapshot and malformed optional data isolation")
                page.evaluate("""data => {
                    const originalFetch = window.fetch;
                    window.fetch = (...args) => {
                        if (args[0] !== '/api/sandbox') return originalFetch(...args);
                        window.fetch = originalFetch;
                        return new Promise(resolve => {
                            window.releaseSandbox = () => resolve(new Response(
                                JSON.stringify({...data, seed: 999}),
                                {headers: {'Content-Type': 'application/json'}}));
                        });
                    };
                }""", experiment)
                page.get_by_role("button", name="运行 / 重置", exact=True).click()
                expect(page.get_by_role("button", name="模拟中…", exact=True)).to_be_disabled()
                request_version = page.evaluate("sandboxRequest")
                page.locator("#sandbox-import").set_input_files(exported)
                page.wait_for_function("version => sandboxRequest > version", arg=request_version)
                expect(page.locator("#calibration-status")).to_contain_text("已恢复实验输入")
                page.evaluate("window.releaseSandbox()")
                expect(page.get_by_role("button", name="运行 / 重置", exact=True)).to_be_enabled()
                with page.expect_download() as download:
                    page.locator("#sandbox-export").click()
                raced = args.output / "sandbox-after-pending-response.json"
                download.value.save_as(str(raced))
                assert json.loads(raced.read_text()) == experiment
                checks.append("pending sandbox response cannot replace imported experiment")
                page.locator('[name="trials"]').fill("2")
                page.get_by_role("button", name="多种子评测", exact=True).click()
                expect(page.locator("#evaluation-result")).to_contain_text("4 场", timeout=30000)
                checks.append("paired seed evaluation")
                page.screenshot(path=str(args.output / "sandbox-desktop.png"), full_page=True)
                page.set_viewport_size({"width": 390, "height": 844})
                page.screenshot(path=str(args.output / "sandbox-narrow.png"), full_page=True)
                assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
                button = page.locator("#sandbox-export").bounding_box()
                assert button["height"] < 60  # Not a compressed vertical column of characters.
                page.get_by_role("button", name="比赛观察", exact=True).click()
                expect(page.locator("#live-panel")).to_be_visible()
                page.get_by_role("button", name="恢复实时", exact=True).click()
                expect(page.locator("#live-timeline")).to_be_disabled()
                state.ros_available = True
                for source, caption in (("ground_ray", "地面投影"), ("radius", "半径代理"),
                                        ("unknown", "来源未知")):
                    state.receive_talk("red_1", "cupcup|px=1|pz=0|bdist=0.2|"
                                       "ball_range_source=" + source)
                    expect(page.locator("#live-status")).to_contain_text(caption)
                    expect(page.locator("#live-status")).to_contain_text("本机球距 0.20m")
                checks.append("narrow layout, mode switching and resume live")
                # A separate browser page exercises actual fetch failure/timeout,
                # without replacing the recording or changing robot state.
                monitor = browser.new_page(viewport={"width": 1280, "height": 1000})
                monitor.on("pageerror", lambda error: errors.append(str(error)))
                feed = {"kind": "fresh", "hung": []}
                observed = {"mode": "live", "truth_available": False, "elapsed_s": 1,
                            "ros_available": True, "score": {"red": 0, "blue": 0},
                            "robots": [{"name": "red_1", "x": 1, "z": 0, "yaw": 0,
                                        "pose_age": 0, "message_age_s": 0,
                                        "mapped_ball": [.2, .3], "mapped_ball_age": 0,
                                        "mapped_ball_source": "ground_ray"}]}

                def live_feed(route):
                    if feed["kind"] == "fresh":
                        route.fulfill(json=observed)
                    elif feed["kind"] == "invalid":
                        route.fulfill(json={"robots": "broken"})
                    elif feed["kind"] == "hang":
                        feed["hung"].append(route)
                    else:
                        route.abort("failed")

                monitor.route("**/api/live", live_feed)
                monitor.goto(f"http://127.0.0.1:{server.server_port}")
                expect(monitor.locator("#connection")).to_contain_text("ROS 2 已连接")
                expect(monitor.locator("#live-details")).to_contain_text("ground_ray")
                feed["kind"] = "failed"
                expect(monitor.locator("#connection")).to_contain_text("服务失联")
                monitor.route("**/api/live/history", lambda route: route.abort("failed"))
                monitor.locator("#live-export").click()
                expect(monitor.locator("#live-file-status")).to_contain_text("导出失败")
                expect(monitor.locator("#live-export")).to_be_enabled()
                expect(monitor.locator("#live-details")).to_contain_text("过期", timeout=5000)
                monitor.wait_for_function("presentedLive().robots[0].message_age_s > 2")
                assert monitor.evaluate("latestLive.robots[0].message_age_s === 0")
                assert monitor.evaluate("""() => {
                    const ctx = document.querySelector('#live-field').getContext('2d');
                    const labels = [], original = ctx.fillText;
                    ctx.fillText = (...args) => labels.push(args[0]);
                    try { drawLive(); } finally { ctx.fillText = original; }
                    return labels.length === 0;
                }""")
                with monitor.expect_download() as download:
                    monitor.locator("#live-snapshot").click()
                failed_snapshot = args.output / "live-disconnected-analysis.json"
                download.value.save_as(str(failed_snapshot))
                analysis = json.loads(failed_snapshot.read_text())
                assert analysis["context"]["client_age_s"] > 2
                assert analysis["frame"] == observed
                monitor.screenshot(path=str(args.output / "live-disconnected.png"), full_page=True)
                feed["kind"] = "invalid"
                monitor.wait_for_timeout(700)
                assert monitor.evaluate("latestLive.robots[0].name === 'red_1'")
                feed["kind"] = "fresh"
                expect(monitor.locator("#connection")).to_contain_text("ROS 2 已连接")
                monitor.wait_for_function("presentedLive().robots[0].message_age_s < .65")
                monitor.locator("#live-import").set_input_files(replay_file)
                expect(monitor.locator("#connection")).to_contain_text("离线观测回放")
                replay_before = monitor.evaluate("JSON.stringify(presentedLive())")
                monitor.wait_for_timeout(750)
                assert monitor.evaluate("JSON.stringify(presentedLive())") == replay_before
                feed["kind"] = "failed"
                monitor.locator("#live-resume").click()
                expect(monitor.locator("#live-red-score")).to_have_text("—")
                assert monitor.evaluate("latestLive === null")
                checks.append("live disconnect ageing, invalid response, "
                              "recovery and replay isolation")
                feed["kind"] = "fresh"
                expect(monitor.locator("#connection")).to_contain_text("ROS 2 已连接")
                feed["kind"] = "hang"
                monitor.wait_for_timeout(650)
                assert len(feed["hung"]) == 1
                monitor.wait_for_timeout(650)
                assert len(feed["hung"]) == 1  # No overlapping requests every 500 ms.
                monitor.wait_for_timeout(2000)
                assert len(feed["hung"]) >= 2  # Timed out and retried, not stuck forever.
                expect(monitor.locator("#live-details")).to_contain_text("过期")
                feed["kind"] = "failed"
                for route in feed["hung"]:
                    route.abort("failed")
                monitor.unroute_all(behavior="wait")
                monitor.close()
                checks.append("hung live request expires and retries without overlapping polls")
            finally:
                browser.close()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=5)
        assets_changed = any(hashlib.sha256((web / name).read_bytes()).hexdigest() != value
                             for name, value in web_hashes.items())
        report = {"checks": checks, "page_errors": errors,
                  "passed": len(checks) == 15 and not errors and not assets_changed,
                  "assets_changed_during_test": assets_changed,
                  "replay_snapshot_bytes": len(replay_bytes),
                  "replay_sha256": hashlib.sha256(replay_bytes).hexdigest(),
                  "calibration_sha256": hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
                  "sandbox_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                  "web_sha256": web_hashes}
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if not report["passed"]:
        raise AssertionError(report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
