#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import yaml

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped

class GripperDebugger(Node):

    def __init__(self):
        super().__init__('gripper_debugger')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer for periodic printing
        self.create_timer(1.0, self.on_timer)

        self.get_logger().info("\n=== Full System Gripper Debugger Started ===\n")

    def on_timer(self):
        self.print_topics()
        self.print_transforms()

    def print_topics(self):
        self.get_logger().info("\n==== Active Topics ====\n")
        try:
            # CLI fallback to list topics
            topics = subprocess.check_output(["ros2", "topic", "list"]).decode().splitlines()
            for t in topics:
                self.get_logger().info(t)
        except Exception as e:
            self.get_logger().warn(f"Failed to list topics: {e}")

    def print_transforms(self):
        self.get_logger().info("\n==== TF Transforms ====\n")
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info("Known TF Frames (YAML):\n" + frames_yaml.strip() + "\n")

            # Parse YAML to get frame names
            frames_dict = yaml.safe_load(frames_yaml) or {}
            frames = list(frames_dict.keys())

            base_frame = 'robotiq_85_base_link'  # adjust to your base link
            for child_frame in frames:
                if child_frame == base_frame:
                    continue
                try:
                    t: TransformStamped = self.tf_buffer.lookup_transform(
                        base_frame,
                        child_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05)
                    )
                    trans = t.transform.translation
                    rot = t.transform.rotation
                    msg = (
                        f"Transform → {child_frame} → {base_frame}\n"
                        f"  Translation: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}\n"
                        f"  Rotation (quat): x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}\n"
                    )
                    self.get_logger().info(msg)
                except (LookupException, ConnectivityException, ExtrapolationException):
                    continue
                except Exception as e:
                    self.get_logger().warn(f"[TF] Other error: {e}")

        except Exception as e:
            self.get_logger().warn(f"[TF] Could not get frame list: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nShutting down Gripper Debugger")
    finally:
        node.destroy_node()
        rclpy.shutdown()