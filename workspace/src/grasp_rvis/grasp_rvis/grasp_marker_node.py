"""Convert grasp pose candidates into individually selectable RViz markers."""

import math

from geometry_msgs.msg import Point, PoseStamped
from grasp_pose_interface.msg import GraspPoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_POSES_TOPIC = '/grasp_pose_candidates'
DEFAULT_MARKERS_TOPIC = '/grasp_rvis/grasp_markers'
DEFAULT_GRIPPER_MARKERS_TOPIC = '/grasp_rvis/gripper_markers'

_AXES = (
    ((1.0, 0.0, 0.0), (1.0, 0.1, 0.1, 1.0)),
    ((0.0, 1.0, 0.0), (0.1, 1.0, 0.1, 1.0)),
    ((0.0, 0.0, 1.0), (0.1, 0.4, 1.0, 1.0)),
)


class GraspMarkerNode(Node):
    """Publish one permanent, toggleable marker namespace per grasp pose."""

    def __init__(self):
        super().__init__('grasp_marker_node')
        self.declare_parameter('poses_topic', DEFAULT_POSES_TOPIC)
        self.declare_parameter('markers_topic', DEFAULT_MARKERS_TOPIC)
        self.declare_parameter(
            'gripper_markers_topic', DEFAULT_GRIPPER_MARKERS_TOPIC
        )
        self.declare_parameter('axis_length', 0.04)
        self.declare_parameter('republish_period_sec', 0.5)
        self.declare_parameter('finger_width', 0.012)
        self.declare_parameter('hand_outer_diameter', 0.102)
        self.declare_parameter('hand_depth', 0.037)
        self.declare_parameter('hand_height', 0.022)
        self.declare_parameter('finger_tip_from_tcp', 0.0139)

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('markers_topic').value,
            static_qos,
        )
        self._gripper_publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('gripper_markers_topic').value,
            static_qos,
        )
        self._subscription = self.create_subscription(
            GraspPoseArray,
            self.get_parameter('poses_topic').value,
            self._poses_callback,
            static_qos,
        )
        self._cached_pose_markers = None
        self._cached_gripper_markers = None
        period = float(self.get_parameter('republish_period_sec').value)
        if period <= 0.0:
            raise ValueError('republish_period_sec must be positive.')
        self._republish_timer = self.create_timer(period, self._republish)

    def _poses_callback(self, message):
        pose_markers = MarkerArray()
        gripper_markers = MarkerArray()

        for index, pose in enumerate(message.poses):
            pose_markers.markers.extend(self._pose_markers(index, pose))
            gripper_markers.markers.extend(
                self._gripper_markers(index, pose)
            )

        # Clear candidates from the previous result once, then cache only ADD
        # markers. Re-sending the ADD markers lets RViz restore a namespace
        # after its checkbox is disabled and enabled again.
        self._publish_with_clear(self._publisher, pose_markers)
        self._publish_with_clear(self._gripper_publisher, gripper_markers)
        self._cached_pose_markers = pose_markers
        self._cached_gripper_markers = gripper_markers
        self.get_logger().info(
            f'Visualizing {len(message.poses)} static grasp candidates'
        )

    @staticmethod
    def _publish_with_clear(publisher, markers):
        update = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        update.markers = [clear, *markers.markers]
        publisher.publish(update)

    def _republish(self):
        """Re-send ADD markers so RViz namespace toggles are reversible."""
        if self._cached_pose_markers is not None:
            self._publisher.publish(self._cached_pose_markers)
        if self._cached_gripper_markers is not None:
            self._gripper_publisher.publish(self._cached_gripper_markers)

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

    def _gripper_markers(self, index, pose):
        """Create a parallel-jaw gripper behind an ``arm_tcp`` pose.

        ``arm_tcp`` has +Z along approach, +X along the finger-closing
        direction, and +Y across the finger height. The pads extend beyond
        the TCP by ``finger_tip_from_tcp``.
        """
        namespace = f'grasp_{index:03d}'
        finger_width = float(self.get_parameter('finger_width').value)
        outer_diameter = float(
            self.get_parameter('hand_outer_diameter').value
        )
        hand_depth = float(self.get_parameter('hand_depth').value)
        hand_height = float(self.get_parameter('hand_height').value)
        finger_tip_from_tcp = float(
            self.get_parameter('finger_tip_from_tcp').value
        )
        half_spacing = 0.5 * (outer_diameter - finger_width)
        finger_center = finger_tip_from_tcp - 0.5 * hand_depth
        finger_base = finger_tip_from_tcp - hand_depth

        specs = (
            # Center offsets and dimensions in TCP [closing, height, approach].
            ((-half_spacing, 0.0, finger_center),
             (finger_width, hand_height, hand_depth)),
            ((half_spacing, 0.0, finger_center),
             (finger_width, hand_height, hand_depth)),
            ((0.0, 0.0, finger_base - 0.01),
             (2.0 * half_spacing, hand_height, 0.02)),
        )
        markers = []
        for marker_id, (offset, dimensions) in enumerate(specs):
            marker = Marker()
            marker.header.frame_id = pose.header.frame_id
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            rotated_offset = _rotate_vector(pose, offset)
            marker.pose.position = _point_at(pose, rotated_offset)
            marker.pose.orientation = pose.pose.orientation
            marker.scale.x, marker.scale.y, marker.scale.z = dimensions
            marker.color.r = 0.08
            marker.color.g = 0.35
            marker.color.b = 1.0
            marker.color.a = 0.65
            markers.append(marker)
        return markers


def _point_at(pose, offset):
    point = Point()
    point.x = pose.pose.position.x + offset[0]
    point.y = pose.pose.position.y + offset[1]
    point.z = pose.pose.position.z + offset[2]
    return point


def _rotate_vector(pose: PoseStamped, vector):
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
