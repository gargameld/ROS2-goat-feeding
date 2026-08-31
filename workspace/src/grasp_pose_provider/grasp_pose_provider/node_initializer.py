"""The grasp pose provider node and its construction.

Defines :class:`GraspPoseProviderNode`, a ``rclpy`` node that offers the
``provide_grasp_pose`` action. The node owns two parts and initializes both:

* ``GraspCandidateGenerator``, from
  :mod:`grasp_pose_provider.grasp_candidate_generation.grasp_candidate_generator`,
  turns the current scene into ranked TCP grasp candidates, and
* ``GraspCandidateValidator``, from
  :mod:`grasp_pose_provider.grasp_candidate_validator`, returns the first of
  those candidates MoveIt can reach.

Handling a goal is then generation, followed by validation, with the physics
pausing the capture needs sequenced around them. The node also holds a
:class:`grasp_pose_provider.grasp_candidate_publisher.GraspCandidatePublisher`
and pushes the generated candidates through it, so RViz shows the same batch
the validator is given.
"""

from grasp_pose_interface.action import ProvideGraspPose
from grasp_pose_provider import grasp_candidate_publisher
from grasp_pose_provider import grasp_candidate_validator
from grasp_pose_provider import simulation_control
from grasp_pose_provider.grasp_candidate_generation import grasp_candidate_generator
from grasp_pose_provider.node_parameters import GraspPoseProviderParameters
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


ACTION_NAME = 'provide_grasp_pose'


class GraspPoseProviderNode(Node):
    """Node offering the ``provide_grasp_pose`` action."""

    def __init__(self):
        super().__init__('grasp_pose_provider')
        self.parameters = GraspPoseProviderParameters(self)
        self.parameters.get_parameters()

        # A reentrant group lets the executor deliver point clouds, GPD and
        # arm-action responses while the action callback waits on them.
        self._callback_group = ReentrantCallbackGroup()

        self._generator = grasp_candidate_generator.GraspCandidateGenerator(
            self,
            self.parameters,
            callback_group=self._callback_group,
        )
        self._validator = grasp_candidate_validator.GraspCandidateValidator(
            self,
            self.parameters,
            callback_group=self._callback_group,
        )
        self._candidate_publisher = (
            grasp_candidate_publisher.GraspCandidatePublisher(
                self, self.parameters
            )
        )

        self._action_server = ActionServer(
            self,
            ProvideGraspPose,
            ACTION_NAME,
            self._handle_grasp_pose_request,
            callback_group=self._callback_group,
        )

    def _handle_grasp_pose_request(self, goal_handle):
        def feedback_cb(stage):
            self.get_logger().info(stage)
            feedback = ProvideGraspPose.Feedback()
            feedback.stage = stage
            goal_handle.publish_feedback(feedback)

        simulation_paused = False

        def pause_after_snapshot():
            nonlocal simulation_paused
            feedback_cb('Pausing physics after capturing camera point clouds')
            simulation_control.pause_simulation(self, self.parameters)
            simulation_paused = True

        try:
            grasp_poses = self._generator.generate(
                feedback_cb=feedback_cb,
                snapshot_captured_cb=pause_after_snapshot,
            )
            self._candidate_publisher.publish(grasp_poses)

            feedback_cb('Resuming physics before arm motion planning')
            simulation_control.resume_simulation(self, self.parameters)
            simulation_paused = False

            reachable_grasp = self._validator.first_reachable(
                grasp_poses, feedback_cb=feedback_cb
            )
        except grasp_candidate_generator.NoFoodDetectedError as exc:
            self.get_logger().info(f'No food detected: {exc}')
            feedback_cb('No food detected')
            goal_handle.succeed()

            # Clear transient-local candidates from an earlier successful goal.
            self._candidate_publisher.clear()

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
                    simulation_control.resume_simulation(
                        self, self.parameters
                    )
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
        """Tear both parts down before the node itself goes away."""
        self._candidate_publisher.destroy()
        self._validator.destroy()
        self._generator.destroy()
        return super().destroy_node()


def create_node():
    """Construct and return the grasp pose provider node."""
    return GraspPoseProviderNode()
