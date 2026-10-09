const $ = (selector) => document.querySelector(selector);
const liveCanvas = $("#live-field");
const sandboxCanvas = $("#sandbox-field");
let sandbox = null;
let frameIndex = 0;
let liveReplay = null;
let liveReplaySource = null;
let liveReceivedAt = null;
let liveGeneration = 0;
let livePending = false;
let calibrationInfo = null;
let sandboxRequest = 0;
const mapViews = {};
for (const mode of ["live", "sandbox"]) {
  const root = $(`#${mode}-layers`);
  const layers = mode === "live" ? [["pose", "自机估计"]] : [["truth", "模拟真值（仅对照）"]];
  layers.push(["observations", mode === "live" ? "本机机器人候选" : "本机球观测"], ["map", "决策地图"], ["intent", "行动意图"],
    ["shared", "队友障碍"]);
  const view = mapViews[mode] = { observer: "all" };
  const redraw = () => mode === "live" ? drawLive() : drawSandbox();
  for (const [key, title] of layers) {
    view[key] = true;
    const label = document.createElement("label"), input = document.createElement("input");
    input.type = "checkbox"; input.checked = true; input.id = `${mode}-layer-${key}`;
    input.addEventListener("change", () => { view[key] = input.checked; redraw(); });
    label.append(input, title); root.append(label);
  }
  const label = document.createElement("label"); label.append("观察者 ");
  const selector = document.createElement("select"); selector.id = `${mode}-observer`;
  selector.add(new Option("全部", "all")); label.append(selector); root.append(label);
  selector.addEventListener("change", () => { view.observer = selector.value; redraw(); });
  const button = document.createElement("button"); button.type = "button";
  button.id = `${mode}-snapshot`; button.textContent = "导出当前分析帧";
  button.addEventListener("click", () => {
    const frame = mode === "live" ? latestLive : currentFrame(); if (!frame) return;
    const context = mode === "live" ? {recording: liveReplaySource || "live",
      frame_index: liveReplay ? Number($("#live-timeline").value) : null,
      client_age_s: liveClientAge()} : {
      model: sandbox.model, seed: sandbox.seed, mode: sandbox.mode,
      cupcup_color: sandbox.cupcup_color, opponent: sandbox.opponent,
      parameters: sandbox.parameters, parameter_provenance: sandbox.parameter_provenance || null,
      frame_index: frameIndex,
    };
    download({schema: "cupcup-map-analysis-v1", mode, view: {...view},
      truth_available: mode === "sandbox", context, frame}, `cupcup-${mode}-analysis.json`);
  }); root.append(button);
  const note = document.createElement("p"); note.className = "map-note";
  note.textContent = "左 −x（蓝方球门），右 +x（红方球门），上 +z。筛选仅改变显示，不修改记录或决策。" +
    (mode === "sandbox" ? " 观察者筛选不隐藏真值背景，请单独关闭真值图层。" : "");
  root.append(note);
}

for (const [entity, title, defaults] of [
  ["red1", "红1", [1.5, -.55, 180]], ["red2", "红2", [2.1, .55, 180]],
  ["blue1", "蓝1", [-1.5, .55, 0]], ["blue2", "蓝2", [-2.1, -.55, 0]],
  ["ball", "球", [0, 0, 0, 0]],
]) {
  const axes = entity === "ball" ? ["x", "z", "vx", "vz"] : ["x", "z", "yaw"];
  axes.forEach((axis, i) => {
    const label = document.createElement("label"); label.textContent = `${title} ${axis}`;
    const input = document.createElement("input"); input.type = "number"; input.step = "any";
    input.name = `initial_${entity}_${axis}`; input.value = defaults[i]; input.required = true;
    input.max = { x: entity === "ball" ? 4.5 : 4.3, z: entity === "ball" ? 3 : 2.8,
      yaw: 180, vx: 8, vz: 8 }[axis]; input.min = -Number(input.max);
    label.append(input); $("#initial-state-fields").append(label);
  });
}

// Both modes use recording timestamps, not an arbitrary frame-rate timer.
function playback(button, speed, frames, index, seek, time) {
  let timer = null;
  const stop = () => { clearTimeout(timer); timer = null; $(button).textContent = "播放"; };
  const step = () => {
    const data = frames();
    if (data && index() < data.length - 1) seek(index() + 1);
  };
  const schedule = () => {
    const data = frames(), i = index();
    if (!data || i >= data.length - 1) { stop(); return; }
    const delay = Math.max(0, (time(data[i + 1]) - time(data[i])) * 1000 / Number($(speed).value));
    timer = setTimeout(() => { step(); schedule(); }, delay);
  };
  $(button).addEventListener("click", () => {
    if (timer !== null) { stop(); return; }
    if (!frames()?.length) return;
    if (index() >= frames().length - 1) seek(0);
    $(button).textContent = "暂停"; schedule();
  });
  $(speed).addEventListener("change", () => { if (timer !== null) { stop(); $(button).click(); } });
  return { stop, step: () => { stop(); step(); } };
}

const sandboxPlayback = playback("#play-toggle", "#sandbox-speed", () => sandbox?.frames,
  () => frameIndex, (i) => { frameIndex = i; drawSandbox(); }, (frame) => frame.t);
const livePlayback = playback("#live-play", "#live-speed", () => liveReplay,
  () => Number($("#live-timeline").value), (i) => {
    $("#live-timeline").value = i; latestLive = liveReplay[i]; describeReplay(); drawLive();
  }, (frame) => frame.elapsed_s);

for (const button of document.querySelectorAll(".tab")) {
  button.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((tab) => tab.classList.toggle("active", tab === button));
    $("#live-panel").classList.toggle("hidden", button.dataset.mode !== "live");
    $("#sandbox-panel").classList.toggle("hidden", button.dataset.mode !== "sandbox");
    drawLive();
    drawSandbox();
  });
}

function fieldTransform(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
  const margin = 25;
  const scale = Math.min((rect.width - margin * 2) / 9, (rect.height - margin * 2) / 6);
  const ox = (rect.width - 9 * scale) / 2;
  const oy = (rect.height - 6 * scale) / 2;
  return { ctx, rect, scale, ox, oy, xy: (x, z) => [ox + (x + 4.5) * scale, oy + (3 - z) * scale] };
}

function drawFieldBase(canvas) {
  const bounds = canvas.getBoundingClientRect();
  if (bounds.width <= 50 || bounds.height <= 50) return null;
  const { ctx, rect, scale, ox, oy, xy } = fieldTransform(canvas);
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = "#18352d";
  ctx.fillRect(ox, oy, 9 * scale, 6 * scale);
  ctx.strokeStyle = "#90cdb1";
  ctx.lineWidth = 1.2;
  ctx.strokeRect(ox, oy, 9 * scale, 6 * scale);
  const [cx, cy] = xy(0, 0);
  ctx.beginPath(); ctx.moveTo(cx, oy); ctx.lineTo(cx, oy + 6 * scale); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx, cy, 0.75 * scale, 0, Math.PI * 2); ctx.stroke();
  for (const x of [-4.5, 4.5]) {
    const [gx1, gy1] = xy(x, -1.3); const [gx2, gy2] = xy(x, 1.3);
    ctx.strokeStyle = "#c3dccb"; ctx.lineWidth = 4;
    ctx.beginPath(); ctx.moveTo(gx1, gy1); ctx.lineTo(gx2, gy2); ctx.stroke();
    // Model markings: large box 2 m × 5 m, small box 1 m × 3 m.
    for (const [depth, halfWidth] of [[2, 2.5], [1, 1.5]]) {
      const innerX = x < 0 ? x + depth : x - depth;
      const [px1, py1] = xy(innerX, -halfWidth); const [px2, py2] = xy(x, -halfWidth);
      const [px3, py3] = xy(x, halfWidth); const [px4, py4] = xy(innerX, halfWidth);
      ctx.strokeStyle = "#668f78"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px1, py1); ctx.lineTo(px2, py2); ctx.lineTo(px3, py3); ctx.lineTo(px4, py4); ctx.stroke();
    }
    const sign = x < 0 ? -1 : 1;
    const [rx, ry] = xy(sign * 2.7, 2.0);
    ctx.strokeStyle = "#e7b66177"; ctx.setLineDash([3, 4]);
    ctx.strokeRect(Math.min(rx, gx1), ry, 1.8 * scale, 4 * scale); ctx.setLineDash([]);
  }
  return { ctx, scale, xy };
}

function drawRobot(ctx, scale, xy, robot, color, live = false, layers = {}) {
  const enabled = key => layers[key] !== false;
  const hasPose = Number.isFinite(robot.x) && Number.isFinite(robot.z);
  const [x, y] = xy(robot.x, robot.z);
  const radius = Math.max(8, scale * 0.17);
  if (hasPose && enabled("pose")) {
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, radius, 0, 2 * Math.PI); ctx.fill();
    const angle = (Number(robot.yaw) || 0) * Math.PI / 180;
    ctx.strokeStyle = "#101518"; ctx.lineWidth = 2;
    if (Number.isFinite(robot.yaw)) {
      ctx.beginPath(); ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(angle) * radius * 1.5, y + Math.sin(angle) * radius * 1.5); ctx.stroke();
    }
    if (!live && Number.isFinite(robot.yaw) && Number.isFinite(robot.head_pan_deg)) {
      const viewAngle = angle + robot.head_pan_deg * Math.PI / 180;
      ctx.strokeStyle = "#e7e7c2"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(viewAngle) * radius * 2.3, y + Math.sin(viewAngle) * radius * 2.3);
      ctx.stroke();
    }
    ctx.fillStyle = "#ecf2ef"; ctx.font = "11px system-ui"; ctx.textAlign = "center";
    ctx.fillText(`${live ? robot.name : `#${robot.id}`} ${robot.action || robot.state || ""}`, x, y - radius - 5);
  }
  if (enabled("intent") && hasPose && validPoint(robot.target)) {
    const [tx, ty] = xy(robot.target[0], robot.target[1]);
    ctx.strokeStyle = "#84d5a5"; ctx.setLineDash([4, 4]); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(tx, ty); ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(tx, ty, 5, 0, 2 * Math.PI); ctx.stroke();
  }
  if (enabled("observations") && validPoint(robot.observed_ball)) {
    const [bx, by] = xy(robot.observed_ball[0], robot.observed_ball[1]);
    ctx.strokeStyle = "#ffe486aa"; ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.arc(bx, by, Math.max(5, scale * 0.1), 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
  }
  const mappedBall = Array.isArray(robot.mapped_ball) && robot.mapped_ball.length === 2 &&
    robot.mapped_ball.every(Number.isFinite) && (!live ||
    (robot.message_age_s || 0) + (robot.mapped_ball_age ?? 99) <= .65) ? robot.mapped_ball : null;
  if (enabled("map") && mappedBall) {
    const [mx, my] = xy(mappedBall[0], mappedBall[1]);
    ctx.strokeStyle = "#84d5a5"; ctx.lineWidth = 1.5;
    ctx.strokeRect(mx - 4, my - 4, 8, 8);
  }
  const ballOrigin = mappedBall || (live && (robot.message_age_s || 0) +
    (robot.ball_age ?? 99) <= .65 ? [robot.ball_x, robot.ball_z] : null);
  const kickDestination = robot.kick_target || (live ? [robot.kick_x, robot.kick_z] : null);
  // Controller target heading, NOT measured yaw or a predicted ball trajectory.
  // Keep distinct from the shared-policy destination and unknown in old logs.
  if (enabled("intent") && live && Array.isArray(ballOrigin) && ballOrigin.every(Number.isFinite) &&
      Number.isFinite(robot.aim_yaw)) {
    const angle = robot.aim_yaw * Math.PI / 180;
    const [bx, by] = xy(...ballOrigin);
    const [ax, ay] = xy(ballOrigin[0] + 0.6 * Math.cos(angle),
      ballOrigin[1] - 0.6 * Math.sin(angle));
    ctx.strokeStyle = "#7fe6ed"; ctx.lineWidth = 2; ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(ax, ay); ctx.stroke(); ctx.setLineDash([]);
  }
  if (enabled("intent") && ["CHASE", "CLEAR"].includes(robot.action) && Array.isArray(ballOrigin) &&
      Array.isArray(kickDestination) && [...ballOrigin, ...kickDestination].every(Number.isFinite)) {
    const [bx, by] = xy(...ballOrigin); const [kx, ky] = xy(...kickDestination);
    ctx.strokeStyle = "#ffbe6b"; ctx.lineWidth = 1.5; ctx.setLineDash([6, 3]);
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(kx, ky); ctx.stroke(); ctx.setLineDash([]);
    const angle = Math.atan2(ky - by, kx - bx);
    ctx.beginPath(); ctx.moveTo(kx - 8 * Math.cos(angle - 0.45), ky - 8 * Math.sin(angle - 0.45));
    ctx.lineTo(kx, ky); ctx.lineTo(kx - 8 * Math.cos(angle + 0.45), ky - 8 * Math.sin(angle + 0.45));
    ctx.stroke();
  }
  for (const track of enabled("shared") && Array.isArray(robot.peer_obstacle_tracks) ? robot.peer_obstacle_tracks : [])
    if ((robot.message_age_s || 0) + (track?.age_s ?? 99) <= 0.65)
      drawUnknownCandidate(ctx, scale, xy, track, robot.name || `#${robot.id}`);
}

function drawUnknownCandidate(ctx, scale, xy, candidate, observer = null) {
  if (!candidate || !Number.isFinite(candidate.x) || !Number.isFinite(candidate.z)) return;
  const [x, y] = xy(candidate.x, candidate.z);
  const radius = Math.max(7, scale * 0.16);
  const team = candidate.team || "unknown";
  const color = observer ? "#d5a8ff" : team === "red" ? "#ff7c75" : team === "blue" ? "#68baff" : "#ffc27a";
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash([3, 3]);
  ctx.beginPath();
  if (observer) {
    ctx.moveTo(x, y - radius); ctx.lineTo(x + radius, y);
    ctx.lineTo(x, y + radius); ctx.lineTo(x - radius, y); ctx.closePath();
  } else ctx.arc(x, y, radius, 0, 2 * Math.PI);
  ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = color; ctx.font = "11px system-ui"; ctx.textAlign = "center";
  ctx.fillText(`${team === "unknown" ? "未分类" : team === "red" ? "红方候选" : "蓝方候选"} ${candidate.track_id ?? ""}`, x, y - radius - 4);
  const source = observer ? `队友→${observer}` : candidate.latest_source === "number_square" ? "号码板" : candidate.latest_source === "box_height" ? "框高" : "来源未知";
  ctx.fillText(source, x, y + radius + 12);
  if (Number.isFinite(candidate.uncertainty) && candidate.uncertainty >= 0) {
    ctx.strokeStyle = color + "44"; ctx.setLineDash([2, 5]);
    ctx.beginPath(); ctx.arc(x, y, candidate.uncertainty * scale, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
  }
}

function drawBall(ctx, xy, ball) {
  if (!Array.isArray(ball)) return;
  const [x, y] = xy(ball[0], ball[1]);
  ctx.fillStyle = "#ffd45e"; ctx.strokeStyle = "#fff2bc"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
}

function drawScenePreview() {
  const field = drawFieldBase($("#scene-preview")); if (!field) return;
  const { ctx, scale, xy } = field;
  const value = (entity, axis) => Number($(`[name="initial_${entity}_${axis}"]`).value);
  for (const entity of ["red1", "red2", "blue1", "blue2"])
    drawRobot(ctx, scale, xy, {id: entity, x: value(entity, "x"), z: value(entity, "z"),
      yaw: value(entity, "yaw")}, entity.startsWith("red") ? "#ff7c75" : "#68baff");
  drawBall(ctx, xy, [value("ball", "x"), value("ball", "z")]);
}
$(".scene-editor").addEventListener("toggle", drawScenePreview);
$("#initial-state-fields").addEventListener("input", drawScenePreview);
$("#scene-preview").addEventListener("click", (event) => {
  const { rect, scale, ox, oy } = fieldTransform(event.currentTarget);
  const entity = $("#scene-selected").value;
  const coordinates = {x: (event.clientX - rect.left - ox) / scale - 4.5,
    z: 3 - (event.clientY - rect.top - oy) / scale};
  for (const axis of ["x", "z"]) {
    const input = $(`[name="initial_${entity}_${axis}"]`);
    input.value = Math.max(Number(input.min), Math.min(Number(input.max), coordinates[axis])).toFixed(2);
  }
  $('[name="initial_state_mode"]').value = "custom"; drawScenePreview();
});

let latestLive = null;
function liveClientAge() {
  return liveReplay || liveReceivedAt === null ? 0 : Math.max(0, (performance.now() - liveReceivedAt) / 1000);
}
function presentedLive() {
  if (!latestLive || liveReplay) return latestLive;
  const delta = liveClientAge();
  return {...latestLive, robots: latestLive.robots.map(robot => ({...robot,
    message_age_s: Number.isFinite(robot.message_age_s) ? robot.message_age_s + delta : null}))};
}
function describeLiveRobots(robots) {
  return (robots || []).map((r) =>
    `${r.name}: ${r.role || "—"}/${r.state || "—"} · ${r.reason || "—"} · 位姿 ${fmt(r.x)}, ${fmt(r.z)} · 本机球距 ${fmt(r.ball_distance)}m（${({ground_ray: "地面投影", radius: "半径代理"})[r.ball_range_source] || "来源未知"}） · 球图末次更新 ${r.mapped_ball_source || "unknown"} / ${fmt(r.mapped_ball_age)}s · 对准目标 ${fmt(r.aim_yaw)}°（偏角 ${fmt(r.aim_offset)}°，青色短线非球轨迹） · 队友障碍 ${fmt(r.peer_obstacles, 0)} · 球图龄 ${fmt(r.ball_age)}s · Talk ${fmt(r.message_age_s)}s`
  ).join("　 |　 ");
}

function describeReplay() {
  $("#live-status").textContent = `已加载 ${liveReplay.length} 帧，拖动时间轴查看地图与意图。 ${describeLiveRobots(latestLive.robots)}`;
  $("#live-replay-time").textContent = `${latestLive.elapsed_s.toFixed(1)} s`;
}

function drawLive() {
  const frame = presentedLive();
  inspectObjects("live", frame?.robots || []);
  const field = drawFieldBase(liveCanvas);
  if (!field) return;
  if (!frame) {
    $("#live-red-score").textContent = "—"; $("#live-blue-score").textContent = "—";
    return;
  }
  const { ctx, scale, xy } = field;
  const layers = mapViews.live;
  for (const robot of frame.robots || []) {
    if (layers.observer !== "all" && robot.name !== layers.observer) continue;
    // Age the estimates, not just the bridge connection; missing is not zero.
    const poseFresh = (robot.message_age_s || 0) + (robot.pose_age || 0) <= 2;
    drawRobot(ctx, scale, xy, poseFresh ? robot : {...robot, x: null, z: null},
      robot.name.startsWith("red") ? "#ff7c75" : "#68baff", true, layers);
    if (layers.map && (robot.message_age_s || 0) + (robot.ball_age ?? 99) <= 0.65 &&
        Number.isFinite(robot.ball_x) && Number.isFinite(robot.ball_z)) drawBall(ctx, xy, [robot.ball_x, robot.ball_z]);
    const tracks = robot.robot_tracks?.length ? robot.robot_tracks : [robot.robot_candidate];
    for (const track of layers.observations ? tracks : []) if ((robot.message_age_s || 0) + (track?.age_s ?? 99) <= 0.65)
      drawUnknownCandidate(ctx, scale, xy, track);
  }
  $("#live-red-score").textContent = frame.score.red;
  $("#live-blue-score").textContent = frame.score.blue;
}

function refreshLive() {
  if (liveReplay) return;
  drawLive(); // Estimates keep ageing even while a request hangs or fails.
  if (livePending) return;
  livePending = true;
  const generation = liveGeneration;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2000);
  fetch("/api/live", {signal: controller.signal}).then((response) => {
    if (!response.ok) throw new Error("live service error");
    return response.json();
  }).then((data) => {
    // An already pending request must not overwrite a newly imported replay.
    if (liveReplay || generation !== liveGeneration) return;
    if (!validLiveFrame(data)) throw new Error("invalid live frame");
    latestLive = data; liveReceivedAt = performance.now();
    $("#connection").textContent = data.ros_available ? "ROS 2 已连接 · 只读观察" : "未连接 ROS 2 · 比赛观察待命";
    $("#live-red-score").textContent = data.score.red;
    $("#live-blue-score").textContent = data.score.blue;
    if (!data.ros_available) {
      $("#live-status").textContent = "启动命令：ros2 run cupcup cupcup_visualizer。当前页面不会连接或控制机器人。";
    } else if (!data.robots.length) {
      $("#live-status").textContent = "ROS 已连接，等待 cupcup Talk 状态；当前界面不以仿真真值补齐缺失估计。";
    } else {
      $("#live-status").textContent = describeLiveRobots(data.robots);
    }
    drawLive();
  }).catch(() => {
    if (!liveReplay && generation === liveGeneration) {
      $("#connection").textContent = "平台服务失联 · 旧估计继续计龄";
      $("#live-status").textContent = "等待服务恢复；旧位置按信息年龄过期隐藏，不以真值补齐。";
      drawLive();
    }
  }).finally(() => { clearTimeout(timeout); livePending = false; });
}

function fmt(value, digits = 2) { return Number.isFinite(value) ? value.toFixed(digits) : "—"; }
function validPoint(point) { return Array.isArray(point) && point.length === 2 && point.every(Number.isFinite); }
function validRobotRecord(robot) {
  if (!robot || typeof robot !== "object" || Array.isArray(robot)) return false;
  for (const key of ["x", "z", "yaw", "pose_age", "message_age_s", "ball_age",
    "ball_x", "ball_z", "aim_yaw", "mapped_ball_age", "observation_age_s", "head_pan_deg"])
    if (robot[key] != null && !Number.isFinite(robot[key])) return false;
  for (const key of ["target", "kick_target", "observed_ball", "mapped_ball", "mapped_self"])
    if (robot[key] != null && !validPoint(robot[key])) return false;
  const validTrack = track => track && typeof track === "object" &&
    Number.isFinite(track.x) && Number.isFinite(track.z) &&
    ["age_s", "uncertainty", "confidence"].every(key => track[key] == null ||
      (Number.isFinite(track[key]) && track[key] >= 0));
  for (const key of ["robot_tracks", "peer_obstacle_tracks"])
    if (robot[key] != null && (!Array.isArray(robot[key]) || robot[key].length > 64 ||
        !robot[key].every(validTrack))) return false;
  return robot.robot_candidate == null || validTrack(robot.robot_candidate);
}
function validLiveFrame(frame) {
  return frame && frame.mode === "live" && frame.truth_available === false &&
    Number.isFinite(frame.elapsed_s) && frame.elapsed_s >= 0 &&
    Array.isArray(frame.robots) && frame.robots.length <= 16 &&
    frame.robots.every(robot => validRobotRecord(robot) && typeof robot.name === "string") &&
    Number.isFinite(frame.score?.red) && Number.isFinite(frame.score?.blue);
}
function pointText(point) { return validPoint(point) ? `${fmt(point[0])}, ${fmt(point[1])} m` : "未知"; }
function ageText(robot, key, live, ttl) {
  const age = robot[key];
  if (!Number.isFinite(age) || age < 0) return "年龄未知";
  const total = age + (live && Number.isFinite(robot.message_age_s) ? robot.message_age_s : 0);
  return `${fmt(total)} s${total > ttl ? "（过期）" : ""}` +
    (live && !Number.isFinite(robot.message_age_s) ? "（传输龄未知）" : "");
}
function inspectObjects(mode, robots) {
  const view = mapViews[mode], selector = $(`#${mode}-observer`);
  const names = [...new Set(robots.map(r => r.name))];
  if (view.observer !== "all" && !names.includes(view.observer)) names.push(view.observer);
  const values = ["all", ...names];
  if (Array.from(selector.options, o => o.value).join("|") !== values.join("|")) {
    selector.replaceChildren(...values.map(name => new Option(name === "all" ? "全部" : name, name)));
  }
  selector.value = view.observer;
  const selected = robots.filter(r => view.observer === "all" || r.name === view.observer);
  const cards = selected.map(r => {
    const live = mode === "live";
    const lines = [r.name,
      `自机估计：${pointText(live ? [r.x, r.z] : r.mapped_self)} · ${live ? ageText(r, "pose_age", true, 2) : "年龄未记录"}`,
      `状态：${r.state || r.action || "未知"} · 原因：${r.reason || "未知"}`,
      live ? "原始球观测坐标：未记录（不能用球图冒充）" :
        `直接球观测：${pointText(r.observed_ball)} · ${ageText(r, "observation_age_s", false, .65)}`,
      `决策球图：${pointText(r.mapped_ball)} · ${ageText(r, "mapped_ball_age", live, .65)}`,
      `末次球图来源：${r.mapped_ball_source || "unknown"}`,
      `观测置信度：${fmt(live ? r.confidence : r.belief_confidence)}`,
      `行动目标：${pointText(r.target)} · 踢球目标：${pointText(r.kick_target)}`,
    ];
    if (live) lines.push(`可见时发布的球图：${pointText([r.ball_x, r.ball_z])} · ${ageText(r, "ball_age", true, .65)}（非原始观测）`,
      `身体朝向估计：${fmt(r.yaw)}° · Talk 龄：${fmt(r.message_age_s)} s`,
      `执行对准目标：${fmt(r.aim_yaw)}°（不是实测朝向）`);
    else if (view.truth) lines.unshift(`模拟真值：${pointText([r.x, r.z])} · ${fmt(r.yaw)}°（仅离线对照）`);
    const tracks = (r.robot_tracks?.length ? r.robot_tracks : [r.robot_candidate]).filter(Boolean);
    for (const [label, candidates] of [["本机候选", tracks], ["队友障碍", r.peer_obstacle_tracks || []]]) {
      lines.push(`${label}：${candidates.length}${!candidates.length ? "（未记录或未知，不代表不存在）" : ""}`);
      for (const t of candidates) lines.push(`  ${t.team || "unknown"} / ${t.track_id ?? "身份未知"} · ${pointText([t.x, t.z])} · ` +
        `${ageText({...t, message_age_s: r.message_age_s}, "age_s", live, .65)} · 置信 ${fmt(t.confidence)} · 不确定范围 ${fmt(t.uncertainty)}m · ${t.latest_source || t.source || "来源未知"}`);
    }
    const card = document.createElement("div"); card.className = "object-card";
    card.textContent = lines.join("\n"); return card;
  });
  if (!cards.length) {
    const card = document.createElement("div"); card.className = "object-card";
    card.textContent = "本帧没有所选观察者的信息，不能以真值或其他机器人填补。"; cards.push(card);
  }
  $(`#${mode}-details`).replaceChildren(...cards);
}

$("#calibration-import").addEventListener("change", async (event) => {
  const file = event.target.files[0]; if (!file) return;
  event.target.value = ""; // Selecting the same file again must trigger change.
  try {
    const bytes = await file.arrayBuffer();
    const report = JSON.parse(new TextDecoder().decode(bytes));
    const params = report.nominal_sandbox_parameters;
    if (report.schema !== "cupcup-controlled-calibration-v1" || !params)
      throw new Error("不是受控测量报告");
    const bounds = [["speed", 0, 1.5], ["turn", 1, 360],
                    ["kick-speed", 0, 6], ["ball-decay", 0.05, 5]];
    for (const [key, low, high] of bounds) {
      if (!Number.isFinite(params[key]) || params[key] < low || params[key] > high)
        throw new Error("测量参数无效或与当前二维模型不兼容");
    }
    const hash = Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256", bytes)))
      .map((value) => value.toString(16).padStart(2, "0")).join("");
    for (const [key] of bounds) {
      const input = $(`#sandbox-controls [name="${key.replaceAll("-", "_")}"]`);
      if (input) input.value = params[key];
    }
    for (const key of ["speed", "turn", "kick_speed"])
      $(`[name="opponent_${key}"]`).value = params[key.replaceAll("_", "-")];
    calibrationInfo = { file: file.name, sha256: hash, nominal_parameters: params };
    $("#calibration-status").textContent = "已导入速度、转向、左脚球速与滚动衰减；其余参数仍未标定。";
  } catch (error) { $("#calibration-status").textContent = error.message; }
});

function download(data, name) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(data)], { type: "application/json" }));
  const anchor = document.createElement("a"); anchor.href = url; anchor.download = name;
  anchor.click(); URL.revokeObjectURL(url);
}
$("#sandbox-export").addEventListener("click", () => {
  if (sandbox) download(sandbox, `cupcup-${sandbox.seed}-${sandbox.cupcup_color}.json`);
});
$("#live-export").addEventListener("click", async () => {
  const button = $("#live-export"), generation = liveGeneration;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2000);
  button.disabled = true;
  try {
    let data = {frames: liveReplay};
    if (!liveReplay) {
      const response = await fetch("/api/live/history", {signal: controller.signal});
      if (!response.ok) throw new Error("观测服务不可用");
      data = await response.json();
      if (!Array.isArray(data.frames) || !data.frames.length || !data.frames.every(validLiveFrame))
        throw new Error("没有可导出的有效观测记录");
    }
    if (generation !== liveGeneration) throw new Error("视图已变化，请重新导出");
    download(data, "cupcup-live-belief.json");
    $("#live-file-status").textContent = `已导出 ${data.frames.length} 帧` +
      (liveReplay ? "当前回放。" : "最近观测；完整实录见服务终端显示的 JSONL 路径。");
  } catch (error) {
    $("#live-file-status").textContent = `导出失败：${error.message}。当前地图和记录未改变。`;
  } finally { clearTimeout(timeout); button.disabled = false; }
});
$("#live-import").addEventListener("change", async (event) => {
  const file = event.target.files[0]; if (!file) return;
  event.target.value = "";
  const generation = ++liveGeneration;
  $("#live-file-status").textContent = `读取 ${file.name}…`;
  try {
    const text = await file.text(); let data;
    try { data = JSON.parse(text); }
    catch { data = { frames: text.trim().split("\n").map((line) => JSON.parse(line)) }; }
    const frames = data.frames;
    if (!Array.isArray(frames) || !frames.length || frames.length > 100000 ||
        !frames.every(validLiveFrame))
      throw new Error("不是本平台的观测回放");
    if (frames.some((frame, i) => i > 0 && frame.elapsed_s < frames[i - 1].elapsed_s))
      throw new Error("回放时间必须单调递增");
    if (generation !== liveGeneration) return;
    livePlayback.stop(); liveReplay = frames; liveReplaySource = file.name;
    liveReceivedAt = null;
    for (const id of ["#live-play", "#live-step", "#live-start"]) $(id).disabled = false;
    $("#live-timeline").max = frames.length - 1;
    $("#live-timeline").value = 0; $("#live-timeline").disabled = false;
    latestLive = frames[0]; $("#connection").textContent = "离线观测回放 · 无真值";
    $("#live-file-status").textContent = `已导入 ${file.name}，${frames.length} 帧。`;
    describeReplay();
    drawLive();
  } catch (error) {
    if (generation === liveGeneration) {
      $("#live-status").textContent = error.message;
      $("#live-file-status").textContent = `导入失败：${error.message}。原记录保留。`;
    }
  }
});
$("#live-timeline").addEventListener("input", (event) => {
  if (!liveReplay) return;
  livePlayback.stop();
  latestLive = liveReplay[Number(event.target.value)];
  describeReplay();
  drawLive();
});
$("#live-resume").addEventListener("click", () => {
  ++liveGeneration;
  livePlayback.stop();
  for (const id of ["#live-play", "#live-step", "#live-start"]) $(id).disabled = true;
  liveReplay = null; liveReplaySource = null; $("#live-timeline").disabled = true;
  latestLive = null; liveReceivedAt = null;
  $("#connection").textContent = "等待实时观测";
  $("#live-file-status").textContent = "";
  $("#live-status").textContent = "等待新的实时状态；离线回放不作为当前比赛信息。";
  $("#live-replay-time").textContent = "实时"; refreshLive();
});

function currentFrame() { return sandbox?.frames?.[frameIndex] || null; }
function mapRobots(mode) {
  if (mode === "live") return presentedLive()?.robots || [];
  const frame = currentFrame(); if (!frame) return [];
  return ["red", "blue"].flatMap(color => frame[color].map(robot =>
    ({...robot, name: `${color}_${robot.id}`})));
}

for (const [mode, canvas] of [["live", liveCanvas], ["sandbox", sandboxCanvas]]) {
  canvas.addEventListener("click", (event) => {
    const { rect, scale, ox, oy } = fieldTransform(canvas);
    const mouse = [(event.clientX - rect.left - ox) / scale - 4.5,
      3 - (event.clientY - rect.top - oy) / scale];
    const view = mapViews[mode];
    const candidates = mapRobots(mode).map(robot => {
      let point = null;
      if ((mode === "live" || !view.truth) && view.observer !== "all" && view.observer !== robot.name)
        return {robot, distance: Infinity};
      if (mode === "sandbox") point = view.truth ? [robot.x, robot.z] : view.map ? robot.mapped_self : null;
      else if (view.pose && (robot.message_age_s || 0) + (robot.pose_age || 0) <= 2)
        point = [robot.x, robot.z];
      return {robot, distance: validPoint(point) ? Math.hypot(point[0] - mouse[0], point[1] - mouse[1]) : Infinity};
    }).filter(item => item.distance <= .4).sort((a, b) => a.distance - b.distance);
    if (candidates.length) mapViews[mode].observer = candidates[0].robot.name;
    // Coordinate calculation resized the canvas; always redraw, including a miss.
    if (mode === "live") drawLive(); else drawSandbox();
  });
}

function drawSandbox() {
  const frame = currentFrame();
  if (frame) inspectObjects("sandbox", mapRobots("sandbox"));
  const field = drawFieldBase(sandboxCanvas);
  if (!field) return;
  const { ctx, scale, xy } = field;
  if (!frame) return;
  const view = mapViews.sandbox;
  for (const robot of mapRobots("sandbox")) {
    const color = robot.name.startsWith("red") ? "#ff7c75" : "#68baff";
    if (view.truth) drawRobot(ctx, scale, xy, robot, color, false,
      {observations: false, map: false, intent: false, shared: false});
    if (view.observer !== "all" && view.observer !== robot.name) continue;
    const estimated = validPoint(robot.mapped_self) ? robot.mapped_self : [null, null];
    if (view.map && validPoint(robot.mapped_self)) {
      const [x, y] = xy(...estimated);
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.arc(x, y, Math.max(10, scale * .22), 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]); ctx.fillStyle = color; ctx.font = "11px system-ui";
      ctx.fillText(`${robot.name} 估计`, x, y + 23);
    }
    drawRobot(ctx, scale, xy, {...robot, x: estimated[0], z: estimated[1],
      observed_ball: robot.observation_age_s <= .65 ? robot.observed_ball : null},
    color, false, {...view, pose: false});
  }
  if (view.truth) drawBall(ctx, xy, frame.ball);
  $("#sandbox-red-score").textContent = frame.red_score;
  $("#sandbox-blue-score").textContent = frame.blue_score;
  $("#sim-time").textContent = `${frame.t.toFixed(1)} s · ${frame.phase || "PLAY"}`;
  $("#timeline").value = frameIndex;
}

function stopPlayback() {
  sandboxPlayback.stop();
}

$("#sandbox-controls").addEventListener("submit", async (event) => {
  event.preventDefault(); stopPlayback();
  const request = ++sandboxRequest;
  const inputs = Object.fromEntries(new FormData(event.currentTarget));
  const provenance = calibrationInfo ? structuredClone(calibrationInfo) : null;
  const button = event.currentTarget.querySelector("button[type=submit]");
  button.disabled = true; button.textContent = "模拟中…";
  try {
    const response = await fetch("/api/sandbox", { method: "POST", body: new URLSearchParams(inputs) });
    const data = await response.json();
    if (request !== sandboxRequest) return;
    if (!response.ok) throw new Error(data.error || "模拟失败");
    data.experiment_inputs = inputs;
    if (provenance) data.parameter_provenance = provenance;
    installSandbox(data);
  } catch (error) {
    if (request === sandboxRequest) $("#sandbox-metrics").textContent = error.message;
  } finally {
    button.disabled = false; button.textContent = "运行 / 重置";
  }
});

function installSandbox(data) {
    stopPlayback(); sandbox = data; frameIndex = 0;
    $("#timeline").max = Math.max(0, sandbox.frames.length - 1);
    $("#timeline").value = 0;
    $("#timeline").disabled = false;
    $("#play-toggle").disabled = false;
    $("#step-button").disabled = false;
    $("#sandbox-start").disabled = false;
    const metrics = [
      ["策略内核", "共享比赛意图"], ["场景种子", String(data.seed)],
      ["对手", data.opponent], ["信息", data.mode],
      ["踢球 红/蓝", `${data.metrics.red_kicks}/${data.metrics.blue_kicks}`],
      ["罚站 红/蓝", `${data.metrics.red_penalties}/${data.metrics.blue_penalties}`],
      ["重开", String(data.metrics.restarts)], ["接触步数", String(data.metrics.collision_steps)],
      ["平均球位推进 红/蓝（m）", ["red", "blue"].map(c =>
        Number(data.metrics[c + "_mean_ball_attack_m"]).toFixed(2)).join(" / ")],
      ["无争抢近球机会 红/蓝（s）", ["red", "blue"].map(c =>
        Number(data.metrics[c + "_uncontested_ball_access_s"]).toFixed(1)).join(" / ")],
      ["机器人被阻挡 红/蓝（s）", ["red", "blue"].map(c =>
        Number(data.metrics[c + "_robot_blocked_s"]).toFixed(1)).join(" / ")],
      ["球静止（s）", Number(data.metrics.stationary_ball_s).toFixed(1)],
      ["触球范围内 红/蓝（机器人秒）", ["red", "blue"].map(c =>
        Number(data.metrics[c + "_kick_reach_s"]).toFixed(1)).join(" / ")],
      ["朝向球且对准球路 红/蓝（机器人秒）", ["red", "blue"].map(c =>
        Number(data.metrics[c + "_kick_aim_s"]).toFixed(1)).join(" / ")],
      ["准备被打断 红/蓝", `${data.metrics.red_kick_setup_resets}/${data.metrics.blue_kick_setup_resets}`],
      ["最长连续准备 红/蓝（s）", ["red", "blue"].map(c =>
        Number(data.metrics[c + "_max_kick_preparing_s"]).toFixed(2)).join(" / ")],
    ];
    $("#sandbox-metrics").replaceChildren(...metrics.map(([name, value]) => {
      const card = document.createElement("div"); card.className = "metric";
      const number = document.createElement("strong"); number.textContent = value;
      const label = document.createElement("span"); label.textContent = name;
      card.append(number, label); return card;
    }));
    drawSandbox();
}

$("#sandbox-import").addEventListener("change", async (event) => {
  const file = event.target.files[0]; if (!file) return;
  event.target.value = "";
  const request = ++sandboxRequest;
  try {
    const data = JSON.parse(await file.text());
    if (!Array.isArray(data.frames) || !data.frames.length || data.frames.length > 100000 ||
        !data.metrics || typeof data.metrics !== "object" || Array.isArray(data.metrics) ||
        typeof data.model !== "string" || !data.model.startsWith("bounded-2d-") ||
        !data.frames.every((f, i) => Number.isFinite(f.t) && f.t >= 0 &&
          (!i || f.t >= data.frames[i - 1].t) && validPoint(f.ball) &&
          Number.isFinite(f.red_score) && Number.isFinite(f.blue_score) &&
          [f.red, f.blue].every(team => Array.isArray(team) && team.length === 2 &&
            team.every((r, index) => validRobotRecord(r) && r.id === index + 1 &&
              Number.isFinite(r.x) && Number.isFinite(r.z) && Number.isFinite(r.yaw)))))
      throw new Error("不是有效的二维实验记录");
    // Validate on a detached form before changing either the scene or controls.
    const form = $("#sandbox-controls"), draft = form.cloneNode(true);
    let restorable = Boolean(data.experiment_inputs);
    if (data.experiment_inputs) {
      for (const control of draft.elements) if (control.name) {
        const value = data.experiment_inputs[control.name];
        if (value == null) { restorable = false; continue; }
        if (typeof value !== "string") throw new Error("实验参数类型无效");
        control.value = value;
        if (!control.checkValidity() || control.value !== value)
          throw new Error(`实验参数无效：${control.name}`);
      }
    }
    if (request !== sandboxRequest) return;
    installSandbox(data);
    if (restorable) {
      for (const control of form.elements) if (control.name)
        control.value = data.experiment_inputs[control.name];
      calibrationInfo = data.parameter_provenance || null;
      drawScenePreview();
      $("#calibration-status").textContent = "已恢复实验输入；可按相同种子重新运行。";
    } else $("#calibration-status").textContent = "历史记录仅供回放：未保存完整输入，不能保证重新运行一致。";
    if (data.recording_mode === "final") $("#calibration-status").textContent += " 仅末帧快照，没有连续过程。";
  } catch (error) { if (request === sandboxRequest) $("#sandbox-metrics").textContent = error.message; }
});

$("#evaluate-button").addEventListener("click", async () => {
  const form = $("#sandbox-controls");
  const button = $("#evaluate-button");
  button.disabled = true; button.textContent = "多种子评测中…";
  $("#evaluation-result").textContent = "运行固定连续种子；参数仍属暂定模型…";
  try {
    const response = await fetch("/api/evaluate", {
      method: "POST", body: new URLSearchParams(new FormData(form)),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "评测失败");
    const scores = data.results.map((item) =>
      `#${item.seed} ${item.color === "red" ? "红" : "蓝"} cupcup ` +
      `${item.cupcup_goals}:${item.opponent_goals}`
    ).join("　");
    const red = data.by_color.red;
    const blue = data.by_color.blue;
    $("#evaluation-result").textContent =
      `暂定模型 · ${data.trials} 个配对种子 / ${data.matches} 场 × ${data.duration_s}s · ` +
      `cupcup 胜 ${data.cupcup_wins} / 平 ${data.draws} / 负 ${data.cupcup_losses} · ` +
      `场均进球 ${data.mean_goals_per_match.toFixed(2)}\n` +
      `红方 ${red.wins}-${red.draws}-${red.losses}；` +
      `蓝方 ${blue.wins}-${blue.draws}-${blue.losses}\n${scores}`;
  } catch (error) {
    $("#evaluation-result").textContent = error.message;
  } finally {
    button.disabled = false; button.textContent = "多种子评测";
  }
});

$("#step-button").addEventListener("click", () => sandboxPlayback.step());
$("#live-step").addEventListener("click", () => livePlayback.step());
$("#sandbox-start").addEventListener("click", () => {
  stopPlayback(); frameIndex = 0; drawSandbox();
});
$("#live-start").addEventListener("click", () => {
  livePlayback.stop(); if (!liveReplay) return;
  $("#live-timeline").value = 0; latestLive = liveReplay[0]; describeReplay(); drawLive();
});
$("#timeline").addEventListener("input", (event) => {
  stopPlayback(); frameIndex = Number(event.target.value) || 0; drawSandbox();
});
window.addEventListener("resize", () => { drawLive(); drawSandbox(); drawScenePreview(); });
refreshLive(); setInterval(refreshLive, 500);
