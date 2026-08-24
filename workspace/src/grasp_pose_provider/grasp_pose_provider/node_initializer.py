"""The grasp pose provider node and its construction.

Defines :class:`GraspPoseProviderNode`, a ``rclpy`` node that offers the
``provide_grasp_pose`` action. Handling a goal runs
:func:`grasp_pose_provider.provide_grasp_pose.provide_grasp_pose`, validates
the top candidates with MoveIt, and returns the first reachable pose.
"""

from grasp_pose_interface.action import ProvideGraspPose
from grasp_pose_interface.msg import GraspPoseArray
from grasp_pose_provider import camera_transforms
from grasp_pose_provider import combine_pointclouds
from grasp_pose_provider import grasp_config_conversion
from grasp_pose_provider import grasp_reachability
from grasp_pose_provider import provide_grasp_pose as grasp_pose_pipeline
from grasp_pose_provider import simulation_control
from grasp_pose_provider import stored_model
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


ACTION_NAME = 'provide_grasp_pose'
DEFAULT_GRASP_POSES_TOPIC = '/grasp_pose_candidates'
MAX_RVIZ_CANDIDATES = 5


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
        self.declare_parameter(
            'min_food_point_count',
            grasp_pose_pipeline.DEFAULT_MIN_FOOD_POINT_COUNT,
        )
        self.declare_parameter(
            'tcp_from_finger_base',
            grasp_config_conversion.DEFAULT_TCP_FROM_FINGER_BASE,
        )

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
        self._transform_resolver = (
            camera_transforms.CameraTransformResolver(self)
        )

        # A reentrant group lets the executor deliver GPD and arm-action
        # responses while this callback waits on them.
        self._callback_group = ReentrantCallbackGroup()
        self._pointcloud_snapshotter = (
            combine_pointclouds.PointCloudSnapshotter(
                self,
                topics=list(
                    self.get_parameter('captured_topics')
                    .get_parameter_value()
                    .string_array_value
                ),
                callback_group=self._callback_group,
            )
        )
        self._reachability_checker = (
            grasp_reachability.GraspReachabilityChecker(
                self, callback_group=self._callback_group
            )
        )
        self._action_server = ActionServer(
            self,
            ProvideGraspPose,
            ACTION_NAME,
            self._execute_callback,
            callback_group=self._callback_group,
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
        min_food_point_count = (
            self.get_parameter('min_food_point_count')
            .get_parameter_value()
            .integer_value
        )
        tcp_from_finger_base = (
            self.get_parameter('tcp_from_finger_base')
            .get_parameter_value()
            .double_value
        )

        def feedback_cb(stage):
            self.get_logger().info(stage)
            feedback = ProvideGraspPose.Feedback()
            feedback.stage = stage
            goal_handle.publish_feedback(feedback)

        simulation_paused = False

        def pause_after_snapshot():
            nonlocal simulation_paused
            feedback_cb('Pausing physics after capturing camera point clouds')
            simulation_control.pause_simulation(self)
            simulation_paused = True

        try:
            grasp_poses = grasp_pose_pipeline.provide_grasp_pose(
                self,
                stored_pointcloud_dir,
                self._transform_resolver,
                captured_topics=list(captured_topics),
                min_food_point_count=min_food_point_count,
                tcp_from_finger_base=tcp_from_finger_base,
                feedback_cb=feedback_cb,
                pointcloud_snapshotter=self._pointcloud_snapshotter,
                snapshot_captured_cb=pause_after_snapshot,
            )
            # Keep RViz focused on the same highest-ranked candidates that
            # MoveIt checks. A rejected batch is still useful diagnostic
            # information and remains visible because the topic is transient.
            rviz_grasp_poses = grasp_poses[:MAX_RVIZ_CANDIDATES]
            candidates = GraspPoseArray()
            candidates.poses = rviz_grasp_poses
            self._grasp_poses_publisher.publish(candidates)
            self.get_logger().info(
                f'Published {len(rviz_grasp_poses)} static grasp candidates'
            )

            feedback_cb('Resuming physics before arm motion planning')
            simulation_control.resume_simulation(self)
            simulation_paused = False

            feedback_cb('Checking reachability of the top grasp candidates')
            reachable_grasp = self._reachability_checker.first_reachable(
                grasp_poses
            )
        except grasp_pose_pipeline.NoFoodDetectedError as exc:
            self.get_logger().info(f'No food detected: {exc}')
            feedback_cb('No food detected')
            goal_handle.succeed()

            # Clear transient-local candidates from an earlier successful goal.
            candidates = GraspPoseArray()
            candidates.poses = []
            self._grasp_poses_publisher.publish(candidates)

            result = ProvideGraspPose.Result()
            result.food_found = False
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Grasp pose workflow failed: {exc}')
            goal_handle.abort()
            return ProvideGraspPose.Result()
        finally:
            if simulation_paused:
                try:
                    simulation_control.resume_simulation(self)
                    self.get_logger().info(
                        'Resumed physics after the grasp-pose action'
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f'Failed to resume physics: {exc}'
                    )

        goal_handle.succeed()
        result = ProvideGraspPose.Result()
        result.food_found = True
        result.grasp_pose = reachable_grasp.pose
        result.reference_frame = reachable_grasp.header.frame_id
        return result

    def destroy_node(self):
        """Tear the tf2 listener down before the node itself goes away."""
        self._reachability_checker.destroy()
        self._pointcloud_snapshotter.destroy()
        self._transform_resolver.destroy()
        return super().destroy_node()


def create_node():
    """Construct and return the grasp pose provider node."""
    return GraspPoseProviderNode()
