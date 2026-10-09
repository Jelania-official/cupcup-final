#!/usr/bin/env python3
"""Replay the shared production robot detector on uniformly sampled saved images."""

import argparse
import collections
import hashlib
import json
import re
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", type=Path, nargs="+")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--verifier-model", type=Path,
                        help="optional patch verifier; weights are not bundled")
    parser.add_argument("--samples", type=int, default=80)
    parser.add_argument("--image-pattern", choices=("*.jpg", "*.ppm"), default="*.jpg")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.samples <= 1000:
        parser.error("samples must be between 1 and 1000")
    paths = sorted(path for root in args.images for path in root.glob(args.image_pattern))
    if not paths:
        parser.error("no images found for requested pattern")
    count = min(args.samples, len(paths))
    chosen = [paths[i * len(paths) // count] for i in range(count)]
    artifacts = [args.binary, args.model] + chosen
    if args.verifier_model:
        artifacts.append(args.verifier_model)
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in artifacts}
    result = subprocess.run(
        ([str(args.binary), "--replay-verified", str(args.model), str(args.verifier_model)]
         if args.verifier_model else [str(args.binary), "--replay", str(args.model)]),
        input="\n".join(json.dumps(str(path), ensure_ascii=False) for path in chosen),
        capture_output=True, text=True, check=True,
    )
    frames = [json.loads(line) for line in result.stdout.splitlines()]
    if [frame["image"] for frame in frames] != [str(path) for path in chosen]:
        parser.error("replay did not cover the sampled images exactly once in order")
    changed = [p for p in artifacts
               if hashlib.sha256(p.read_bytes()).hexdigest() != hashes[str(p)]]
    if changed:
        parser.error("binary, model or sampled image changed during replay")
    teams = collections.Counter(box["team"] for frame in frames for box in frame["robots"])
    output = {
        "samples": len(frames), "images_available": len(paths),
        "frames_with_multiple_robots": sum(len(frame["robots"]) > 1 for frame in frames),
        "frames_without_robots": sum(not frame["robots"] for frame in frames),
        "team_candidates": dict(teams),
        "model_sha256": hashes[str(args.model)],
        "binary_sha256": hashes[str(args.binary)],
        "verifier_model_sha256": hashes[str(args.verifier_model)] if args.verifier_model else None,
        "image_sha256": {str(path): hashes[str(path)] for path in chosen},
        "replay_arguments": {key: [str(p) for p in value] if isinstance(value, list)
                             else str(value) if isinstance(value, Path) else value
                             for key, value in vars(args).items()},
        "receipt_clock_frames": sum("_receipt_" in Path(frame["image"]).name
                                    for frame in frames),
        "untimestamped_frames": sum(
            "_receipt_" not in Path(frame["image"]).name
            and re.fullmatch(r"(?:red|blue)_[12]_\d+\.jpg", Path(frame["image"]).name) is None
            for frame in frames),
        "number_landmarks": sum(len(frame.get("number_patches", [])) for frame in frames),
        "capture_metadata_sha256": {
            str(root): hashlib.sha256((root / "capture.jsonl").read_bytes()).hexdigest()
            if (root / "capture.jsonl").is_file() else None for root in args.images},
        "caveat": ("Unlabelled replay: counts prove pipeline execution, "
                   "NOT precision, recall or position accuracy. "
                   "Receipt-clock images cannot be synchronized as "
                   "simulation capture timestamps."),
        "frames": frames,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {key: value for key, value in output.items()
               if key not in ("frames", "image_sha256")}
    summary["hashed_images"] = len(chosen)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
