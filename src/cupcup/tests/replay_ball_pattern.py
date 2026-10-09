#!/usr/bin/env python3
"""Replay the production appearance check on capture-matched recorded candidates."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import subprocess
from pathlib import Path

from analyze_world_trace import number, parse_talk


def truth_pixel(row: dict[str, str], robot: str,
                fallback_height: float = 0.07) -> tuple[float, float] | None:
    """Project trace-only ball truth into the measured camera for evaluation."""
    origin = [number(row, f"{robot}_camera_{axis}") for axis in "xyz"]
    matrix = [number(row, f"{robot}_camera_r{i}") for i in range(9)]
    height = number(row, "ball_y")
    ball = [number(row, "ball_x"), fallback_height if height is None else height,
            number(row, "ball_z")]
    if None in origin + matrix + ball:
        return None
    delta = [ball[i] - origin[i] for i in range(3)]
    local = [sum(matrix[3 * k + i] * delta[k] for k in range(3)) for i in range(3)]
    if local[0] <= 0.0:
        return None
    scale = 2.0 * math.tan(1.3613 / 2.0)
    return (0.5 - local[1] / (local[0] * scale),
            0.5 - local[2] / (local[0] * scale * 480.0 / 640.0))


def evaluate(trace: Path, color: str, binary: Path) -> dict[str, object]:
    """Run ONE batched C++ replay, so offline and production filters are identical."""
    with trace.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    times = [float(row["time"]) for row in rows]
    inputs = []
    labels = []
    previous = {f"{color}_1": -1, f"{color}_2": -1}
    for row in rows:
        for robot in previous:
            fields = parse_talk(row.get(f"{robot}_talk", ""))
            stamp = number(fields, "bstamp_ms")
            if stamp is None or stamp <= previous[robot]:
                continue
            previous[robot] = stamp
            image = trace.parent / "images" / f"{robot}_{int(stamp):09d}.jpg"
            u, v, radius = [number(fields, key) for key in ("iu", "iv", "ir")]
            if not image.is_file() or None in (u, v, radius) or radius <= 0:
                continue
            index = min(bisect.bisect_left(times, stamp / 1000.0), len(rows) - 1)
            if index > 0 and abs(times[index - 1] - stamp / 1000.0) < abs(
                    times[index] - stamp / 1000.0):
                index -= 1
            if abs(times[index] - stamp / 1000.0) > 0.04:
                continue
            pixel = truth_pixel(rows[index], robot)
            label = "unscored"
            if pixel is not None:
                error = math.hypot((u - pixel[0]) * 640, (v - pixel[1]) * 480)
                if error <= max(6.0, radius * 640.0 * 0.6):
                    label = "near_truth_pixel"
                elif error > max(12.0, radius * 640.0 * 2.0):
                    label = "off_target"
                else:
                    label = "ambiguous"
            inputs.append(f'{json.dumps(str(image), ensure_ascii=False)} {u} {v} {radius}')
            labels.append((robot, label))
    replay = subprocess.run([str(binary), "--replay"], input="\n".join(inputs) + "\n",
                            capture_output=True, text=True, check=True)
    decisions = replay.stdout.splitlines()
    if len(decisions) != len(labels):
        raise ValueError("C++ replay count does not match input candidates")
    groups = {}
    for (robot, label), decision in zip(labels, decisions):
        key = f"{robot}:{label}"
        group = groups.setdefault(key, {"n": 0, "kept": 0, "rejected": 0})
        group["n"] += 1
        group["kept" if decision == "1" else "rejected"] += 1
    return {"trace": str(trace), "candidates": len(labels), "groups": groups,
            "caveat": "Pixel proximity is not a manual semantic label; JPEG/pose errors remain."}


def main() -> int:
    """Save repeatable diagnostic counts beside the input match log."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("binary", type=Path, help="built ball_appearance_test executable")
    parser.add_argument("--color", choices=("red", "blue"), required=True)
    args = parser.parse_args()
    result = evaluate(args.trace, args.color, args.binary)
    output = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    (args.trace.parent / "pattern_replay.json").write_text(output, encoding="utf-8")
    print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
