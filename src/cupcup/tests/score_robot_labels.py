#!/usr/bin/env python3
"""Score production detections against frozen visually reviewed pixel labels."""

import argparse
import collections
import hashlib
import json
import math
import subprocess
from pathlib import Path


def iou(a, b):
    """Compute intersection/union of clipped xyxy boxes."""
    def area(box):
        return max(0, box[2] - box[0]) * max(0, box[3] - box[1])
    intersection = area([max(a[0], b[0]), max(a[1], b[1]),
                         min(a[2], b[2]), min(a[3], b[3])])
    return intersection / max(1e-9, area(a) + area(b) - intersection)


def match_boxes(predictions, labels, threshold):
    """Maximize one-to-one matches, then IoU, for at most four field robots."""
    if len(predictions) > 4 or len(labels) > 4:
        raise ValueError("four-robot bounded matcher contract exceeded")
    if not math.isfinite(threshold) or not 0 < threshold <= 1:
        raise ValueError("IoU threshold must be finite, positive and at most one")

    def search(index, used, pairs):
        if index == len(predictions):
            return pairs
        best = search(index + 1, used, pairs)
        for j, label in enumerate(labels):
            overlap = iou(predictions[index], label)
            if j in used or overlap < threshold:
                continue
            candidate = search(index + 1, used | {j}, pairs + [(index, j, overlap)])
            if (len(candidate), sum(p[2] for p in candidate)) > (
                    len(best), sum(p[2] for p in best)):
                best = candidate
        return best

    return search(0, set(), [])


def validate_labels(data):
    """Reject corrupt labels rather than silently ignoring hard cases."""
    if data.get("schema") != "cupcup-visible-robot-labels-v1":
        raise ValueError("unsupported label schema")
    width, height = data["width"], data["height"]
    if not 0 < width <= 4096 or not 0 < height <= 4096:
        raise ValueError("invalid image dimensions")
    names = set()
    for frame in data["frames"]:
        name = frame["image"]
        if Path(name).name != name or name in names:
            raise ValueError("image names must be unique basenames")
        names.add(name)
        if frame["split"] not in ("development", "holdout") or len(frame["robots"]) > 4:
            raise ValueError("invalid split or target count")
        for label in frame["robots"]:
            box = label["xyxy"]
            if (len(box) != 4 or not all(math.isfinite(v) for v in box)
                    or not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height
                    or label["team"] not in ("red", "blue", "unknown")
                    or label["visibility"] not in ("full", "clipped", "occluded")):
                raise ValueError("invalid visible robot annotation")
    if not names:
        raise ValueError("empty labelled dataset")


def evaluate(labels, frames, threshold, split=None):
    """Separate box recall, false positives, and known-color coverage."""
    validate_labels(labels)
    predictions = {Path(frame["image"]).name: frame["robots"] for frame in frames}
    if len(predictions) != len(frames) or set(predictions) != {
            frame["image"] for frame in labels["frames"]}:
        raise ValueError("replay must cover every labelled image exactly once")
    tp = fp = fn = correct = wrong = unclassified = known = 0
    details, confusion = [], collections.Counter()
    by_visibility = collections.defaultdict(lambda: {"targets": 0, "matched": 0})
    by_team = collections.defaultdict(lambda: {"targets": 0, "matched": 0, "correct_team": 0})
    width, height = labels["width"], labels["height"]
    for frame in labels["frames"]:
        if split and frame["split"] != split:
            continue
        boxes = predictions[frame["image"]]
        pixels = []
        for box in boxes:
            if (not all(math.isfinite(box[key]) for key in ("x", "y", "width", "height"))
                    or box["width"] <= 0 or box["height"] <= 0
                    or box["team"] not in ("red", "blue", "unknown")):
                raise ValueError("invalid replay box")
            pixels.append([max(0, (box["x"] - box["width"] / 2) * width),
                           max(0, (box["y"] - box["height"] / 2) * height),
                           min(width, (box["x"] + box["width"] / 2) * width),
                           min(height, (box["y"] + box["height"] / 2) * height)])
        truth = frame["robots"]
        pairs = match_boxes(pixels, [label["xyxy"] for label in truth], threshold)
        matched = {j: i for i, j, _ in pairs}
        tp += len(pairs)
        fp += len(boxes) - len(pairs)
        fn += len(truth) - len(pairs)
        for j, label in enumerate(truth):
            team, visibility = label["team"], label["visibility"]
            known += team != "unknown"
            by_visibility[visibility]["targets"] += 1
            by_visibility[visibility]["matched"] += j in matched
            by_team[team]["targets"] += 1
            by_team[team]["matched"] += j in matched
            if j not in matched:
                confusion[team + ":missed"] += 1
                continue
            detected = boxes[matched[j]]["team"]
            confusion[team + ":" + detected] += 1
            if team != "unknown":
                correct += detected == team
                wrong += detected not in (team, "unknown")
                unclassified += detected == "unknown"
                by_team[team]["correct_team"] += detected == team
        details.append({"image": frame["image"], "split": frame["split"], "predictions": boxes,
                        "labels": truth, "matches": pairs,
                        "missed_label_indices": [j for j in range(len(truth)) if j not in matched],
                        "unmatched_prediction_indices": [i for i in range(len(boxes))
                                                         if i not in {p[0] for p in pairs}]})
    return {"images": len(details), "targets": tp + fn, "true_positives": tp,
            "false_positives": fp, "false_negatives": fn,
            "precision": tp / (tp + fp) if tp + fp else None,
            "recall": tp / (tp + fn) if tp + fn else None,
            "known_team_targets": known, "correct_team": correct, "wrong_team": wrong,
            "unclassified_known_team": unclassified,
            "correct_known_team_detection_recall": correct / known if known else None,
            "team_confusion": dict(confusion), "by_visibility": dict(by_visibility),
            "by_team": dict(by_team), "frames": details}


def evaluate_number_landmarks(labels, frames, split=None):
    """Score panel-centre containment separately, without claiming full-body IoU."""
    validate_labels(labels)
    predictions = {Path(frame["image"]).name: frame.get("number_patches", []) for frame in frames}
    expected_images = {f["image"] for f in labels["frames"]}
    if len(predictions) != len(frames) or set(predictions) != expected_images:
        raise ValueError("landmark replay must cover all labelled images exactly once")
    counts = collections.Counter()
    details = []
    for frame in labels["frames"]:
        if split and frame["split"] != split:
            continue
        used = set()
        items = []
        patches = predictions[frame["image"]]
        if len(patches) > 4:
            raise ValueError("bounded number landmark contract exceeded")
        for patch in patches:
            if (not all(math.isfinite(patch[k]) for k in ("x", "y", "width", "height"))
                    or not 0 <= patch["x"] <= 1 or not 0 <= patch["y"] <= 1
                    or patch["width"] <= 0 or patch["height"] <= 0
                    or patch["team"] not in ("red", "blue")):
                raise ValueError("invalid number landmark")
            x, y = patch["x"] * labels["width"], patch["y"] * labels["height"]
            candidates = [j for j, label in enumerate(frame["robots"])
                          if label["xyxy"][0] <= x <= label["xyxy"][2]
                          and label["xyxy"][1] <= y <= label["xyxy"][3]]
            if not candidates:
                status = "outside_labelled_robots"
            elif len(candidates) > 1:
                status = "ambiguous"
            elif candidates[0] in used:
                status = "duplicate"
            else:
                used.add(candidates[0])
                status = "assigned"
                expected = frame["robots"][candidates[0]]["team"]
                counts["unknown_label_team" if expected == "unknown" else
                       "correct_team" if expected == patch["team"] else "wrong_team"] += 1
            counts[status] += 1
            items.append({"patch": patch, "status": status, "label_candidates": candidates})
        counts["landmarks"] += len(patches)
        counts["images"] += 1
        counts["labelled_targets"] += len(frame["robots"])
        details.append({"image": frame["image"], "landmarks": items})
    return {key: counts[key] for key in (
        "images", "labelled_targets", "landmarks", "assigned", "outside_labelled_robots",
        "ambiguous", "duplicate", "correct_team", "wrong_team", "unknown_label_team")} | {
            "frames": details,
            "caveat": "Panel-centre containment is not body IoU, pose accuracy or marker recall"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("labels", type=Path)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confidence", type=float,
                        help="Offline detector gate probe, not ROS config")
    args = parser.parse_args()
    if args.confidence is not None and (
            not math.isfinite(args.confidence) or not .1 <= args.confidence <= 1):
        parser.error("confidence must be finite and between 0.1 and 1")
    label_content = args.labels.read_bytes()
    label_hash = hashlib.sha256(label_content).hexdigest()
    data = json.loads(label_content)
    validate_labels(data)
    paths = [args.images / frame["image"] for frame in data["frames"]]
    for frame, path in zip(data["frames"], paths):
        if hashlib.sha256(path.read_bytes()).hexdigest() != frame["sha256"]:
            parser.error("image content differs from frozen labels: " + str(path))
    binary_hash = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    model_hash = hashlib.sha256(args.model.read_bytes()).hexdigest()
    command = [str(args.binary), "--replay", str(args.model)]
    if args.confidence is not None:
        command += ["--confidence", str(args.confidence)]
    replay = subprocess.run(command, check=True,
                            input="\n".join(json.dumps(str(p), ensure_ascii=False) for p in paths),
                            capture_output=True, text=True)
    frames = [json.loads(line) for line in replay.stdout.splitlines()]
    if binary_hash != hashlib.sha256(args.binary.read_bytes()).hexdigest():
        parser.error("replay executable changed during run")
    if model_hash != hashlib.sha256(args.model.read_bytes()).hexdigest():
        parser.error("model changed during run")
    if label_hash != hashlib.sha256(args.labels.read_bytes()).hexdigest():
        parser.error("labels changed during run")
    report = {"schema": "cupcup-labelled-robot-evaluation-v1", "label_metadata": {
        key: value for key, value in data.items() if key != "frames"},
        "labels_sha256": label_hash,
        "replay_command": command, "offline_confidence_override": args.confidence,
        "binary_sha256": binary_hash, "model_sha256": model_hash,
        "analyzer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "number_landmarks": {split: evaluate_number_landmarks(
            data, frames, None if split == "all" else split)
            for split in ("all", "development", "holdout")},
        "results": {str(t): {split: evaluate(data, frames, t, None if split == "all" else split)
                             for split in ("all", "development", "holdout")} for t in (.3, .5)}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    summary = {}
    for threshold, splits in report["results"].items():
        summary[threshold] = {}
        for split, values in splits.items():
            summary[threshold][split] = {k: v for k, v in values.items() if k != "frames"}
    print(json.dumps(summary, indent=2))
    landmark_summary = {key: value for key, value in report["number_landmarks"]["all"].items()
                        if key != "frames"}
    print(json.dumps({"number_landmarks": landmark_summary}, indent=2))


if __name__ == "__main__":
    main()
