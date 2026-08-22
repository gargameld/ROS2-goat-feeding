"""The grasp pose provider node and its construction.

Defines :class:`GraspPoseProviderNode`, a ``rclpy`` node that offers the
``provide_grasp_pose`` action. Handling a goal runs
:func:`grasp_pose_provider.provide_grasp_pose.provide_grasp_pose`, forwards its
progress as action feedback, and returns the resulting grasp poses as the
action result.
"""

from grasp_pose_interface.action import ProvideGraspPose
from grasp_pose_interface.msg import GraspPoseArray
from grasp_pose_provider import camera_transforms
from grasp_pose_provider import provide_grasp_pose as grasp_pose_pipeline
from grasp_pose_provider import stored_model
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


ACTION_NAME = 'provide_grasp_pose'
DEFAULT_GRASP_POSES_TOPIC = '/grasp_pose_candidates'


class GraspPoseProviderNode(Node):
    """Node offering the ``provide_grasp_pose`` action."""

    def __init__(self):
        super().__init__('grasp_pose_provider')

        # The directory holding one stored empty-plate dump per camera, named
        # after the camera that recorded it. Defaults to the copy shipped with
        # the package sources.
        self.declare_parameter(
            'stored_pointcloud_dir',
            stored_model.DEFAULT_STORED_POINTCLOUD_DIR,
        )
        self.declare_parameter(
            'captured_topics',
            list(grasp_pose_pipeline.DEFAULT_CAPTURED_TOPICS),
        )
        self.declare_parameter('grasp_poses_topic', DEFAULT_GRASP_POSES_TOPIC)

        # The latest candidates describe a static scene. Transient-local
        # durability lets RViz receive them even when it starts after the
        # action has completed.
        grasp_poses_topic = (
            self.get_parameter('grasp_poses_topic')
            .get_parameter_value()
            .string_value
        )
        self._grasp_poses_publisher = self.create_publisher(
            GraspPoseArray,
            grasp_poses_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Kept for the node's lifetime: the tf2 buffer only answers lookups for
        # transforms it has already received, so the listener has to be up long
        # before the first goal arrives.
        self._transform_resolver = camera_transforms.CameraTransformResolver(self)

        # A reentrant group so the executor can keep delivering the GPD service
        # response while this callback is blocked waiting on it.
        self._action_server = ActionServer(
            self,
            ProvideGraspPose,
            ACTION_NAME,
            self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _execute_callback(self, goal_handle):
        stored_pointcloud_dir = (
            self.get_parameter('stored_pointcloud_dir')
            .get_parameter_value()
            .string_value
        )
        captured_topics = (
            self.get_parameter('captured_topics')
            .get_parameter_value()
            .string_array_value
        )

        def feedback_cb(stage):
            self.get_logger().info(stage)
            feedback = ProvideGraspPose.Feedback()
            feedback.stage = stage
            goal_handle.publish_feedback(feedback)

        try:
            grasp_poses = grasp_pose_pipeline.provide_grasp_pose(
                self,
                stored_pointcloud_dir,
                self._transform_resolver,
                captured_topics=list(captured_topics),
                feedback_cb=feedback_cb,
            )
        except Exception as exc:  # noqa: BLE001 - report failure via the action
            self.get_logger().error(f'Grasp pose detection failed: {exc}')
            goal_handle.abort()
            return ProvideGraspPose.Result()

        goal_handle.succeed()
        candidates = GraspPoseArray()
        candidates.poses = grasp_poses
        self._grasp_poses_publisher.publish(candidates)
        self.get_logger().info(
            f'Published {len(grasp_poses)} static grasp candidates'
        )
        result = ProvideGraspPose.Result()
        result.grasp_poses = grasp_poses
        return result

    def destroy_node(self):
        """Tear the tf2 listener down before the node itself goes away."""
        self._transform_resolver.destroy()
        return super().destroy_node()


def create_node():
    """Construct and return the grasp pose provider node."""
    return GraspPoseProviderNode()
