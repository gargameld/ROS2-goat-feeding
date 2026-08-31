"""Convert grasp pose candidates into individually selectable RViz markers."""

import math

from geometry_msgs.msg import Point
from grasp_pose_interface.msg import GraspPoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_POSES_TOPIC = '/grasp_pose_candidates'
DEFAULT_MARKERS_TOPIC = '/grasp_rvis/grasp_markers'

_AXES = (
    ((1.0, 0.0, 0.0), (1.0, 0.1, 0.1, 1.0)),
    ((0.0, 1.0, 0.0), (0.1, 1.0, 0.1, 1.0)),
    ((0.0, 0.0, 1.0), (0.1, 0.4, 1.0, 1.0)),
)


class GraspMarkerNode(Node):
    """Publish one marker namespace per grasp pose."""

    def __init__(self):
        super().__init__('grasp_marker_node')
        self.declare_parameter('poses_topic', DEFAULT_POSES_TOPIC)
        self.declare_parameter('markers_topic', DEFAULT_MARKERS_TOPIC)
        self.declare_parameter('axis_length', 0.04)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('markers_topic').value,
            qos,
        )
        self._subscription = self.create_subscription(
            GraspPoseArray,
            self.get_parameter('poses_topic').value,
            self._poses_callback,
            qos,
        )
    def _poses_callback(self, message):
        markers = MarkerArray()
        for index, pose in enumerate(message.poses):
            markers.markers.extend(self._pose_markers(index, pose))

        # Clear candidates from the previous result before publishing the new
        # static batch.
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._publisher.publish(MarkerArray(markers=[clear, *markers.markers]))
        self.get_logger().info(
            f'Visualizing {len(message.poses)} static grasp candidates'
        )

    def _pose_markers(self, index, pose):
        namespace = f'grasp_{index:03d}'
        markers = []
        axis_length = float(self.get_parameter('axis_length').value)

        for marker_id, (axis, color) in enumerate(_AXES):
            direction = _rotate_vector(pose, axis)
            marker = Marker()
            marker.header.frame_id = pose.header.frame_id
            # Intentionally leave stamp at zero: the candidate is static and
            # should be transformed using the latest available TF.
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = [
                _point_at(pose, (0.0, 0.0, 0.0)),
                _point_at(pose, tuple(axis_length * x for x in direction)),
            ]
            marker.scale.x = 0.008
            marker.scale.y = 0.016
            marker.scale.z = 0.025
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            markers.append(marker)

        label = Marker()
        label.header.frame_id = pose.header.frame_id
        label.ns = namespace
        label.id = len(_AXES)
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = _point_at(
            pose, (0.0, 0.0, axis_length + 0.025)
        )
        label.pose.orientation.w = 1.0
        label.scale.z = 0.035
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f'grasp {index}'
        markers.append(label)
        return markers


def _point_at(pose, offset):
    position = pose.pose.position
    return Point(
        x=position.x + offset[0],
        y=position.y + offset[1],
        z=position.z + offset[2],
    )


def _rotate_vector(pose, vector):
    """Rotate ``vector`` by ``pose``'s quaternion without using timestamps."""
    q = pose.pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1e-12:
        return vector
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    vx, vy, vz = vector
    # q * v * q^-1, expanded to avoid another runtime dependency.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def main(args=None):
    rclpy.init(args=args)
    node = GraspMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
