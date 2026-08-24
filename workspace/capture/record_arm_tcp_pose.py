#!/usr/bin/env python3
"""Append the current arm TCP pose from TF to a JSON Lines file.

Run this in a terminal where the ROS 2 workspace has been sourced:
    python3 workspace/capture/record_arm_tcp_pose.py
"""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TcpPoseRecorder(Node):
    def __init__(self) -> None:
        super().__init__("record_arm_tcp_pose")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


def parse_args() -> argparse.Namespace:
    capture_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Append the latest TF pose of arm_tcp as JSON Lines."
    )
    parser.add_argument("--frame", default="base_link", help="Reference TF frame.")
    parser.add_argument("--tcp", default="arm_tcp", help="TCP TF frame.")
    parser.add_argument(
        "--output",
        type=Path,
        default=capture_dir / "arm_tcp_pose.jsonl",
        help="Destination JSON Lines file (one pose object per line).",
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="Seconds to wait for TF."
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TcpPoseRecorder()

    deadline = time.monotonic() + args.timeout
    transform = None
    latest_error = ""
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            # Time() means the most recent transform in the TF buffer.
            transform = node.buffer.lookup_transform(
                args.frame, args.tcp, Time(), timeout=Duration(seconds=0.1)
            )
            break
        except TransformException as error:
            latest_error = str(error)

    if transform is None:
        node.get_logger().error(
            f"Could not find TF transform {args.frame} -> {args.tcp}: {latest_error}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    t = transform.transform.translation
    q = transform.transform.rotation
    pose = {
        "frame_id": transform.header.frame_id,
        "child_frame_id": transform.child_frame_id,
        "stamp": {
            "sec": transform.header.stamp.sec,
            "nanosec": transform.header.stamp.nanosec,
        },
        "position_m": {"x": t.x, "y": t.y, "z": t.z},
        "orientation_xyzw": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("a", encoding="utf-8") as output:
        output.write(json.dumps(pose) + "\n")
    node.get_logger().info(f"Appended {args.frame} -> {args.tcp} pose to {args.output}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
