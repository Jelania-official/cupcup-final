#!/usr/bin/env python3
"""Capture test-only RGB evidence, named by robot and image capture timestamp."""

from __future__ import annotations

import argparse
import json
import time
from functools import partial
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageRecorder(Node):
    """Read camera topics without sending any command or truth to players."""

    def __init__(self, color: str, directory: Path):
        super().__init__("cupcup_test_image_recorder")
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        self.last_stamp: dict[str, tuple[str, int]] = {}
        self.started_ns = time.monotonic_ns()
        for robot in (f"{color}_1", f"{color}_2"):
            self.create_subscription(Image, f"/{robot}/sensor/image",
                                     partial(self.capture, robot), qos_profile_sensor_data)

    def capture(self, robot: str, message: Image) -> None:
        """Record original zero-stamped inputs without inventing capture time."""
        received_ns = time.monotonic_ns()
        header_ms = message.header.stamp.sec * 1000 + message.header.stamp.nanosec // 1000000
        basis = "capture_ros_header" if header_ms > 0 else "receipt_monotonic"
        stamp = header_ms if header_ms > 0 else (received_ns - self.started_ns) // 1000000
        previous_basis, previous_stamp = self.last_stamp.get(robot, ("", -1000))
        if basis == previous_basis and stamp - previous_stamp < 500:
            return
        if (not 0 < message.width <= 4096 or not 0 < message.height <= 4096
                or message.step < message.width * 3
                or len(message.data) < message.height * message.step
                or message.encoding not in {"rgb8", "bgr8"}):
            return
        rgb = np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                         buffer=message.data, strides=(message.step, 3, 1))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else rgb
        suffix = f"{stamp:09d}" if header_ms > 0 else f"receipt_{stamp:09d}"
        path = self.directory / f"{robot}_{suffix}.jpg"
        if cv2.imwrite(str(path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            self.last_stamp[robot] = (basis, stamp)
            metadata = {"image": path.name, "robot": robot, "timestamp_basis": basis,
                        "header_stamp_ms": header_ms if header_ms > 0 else None,
                        "received_monotonic_ns": received_ns,
                        "receipt_elapsed_ms": (received_ns - self.started_ns) // 1000000,
                        "width": message.width, "height": message.height,
                        "source_encoding": message.encoding}
            with (self.directory / "capture.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(metadata) + "\n")


def main() -> int:
    """Run an optional evidence recorder alongside a regression match."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("color", choices=("red", "blue"))
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = ImageRecorder(args.color, args.directory)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
