"""Publication of the ranked grasp candidates for RViz.

Making the candidates visible is a reporting concern rather than part of
producing them, so it lives here instead of in
:mod:`grasp_pose_provider.grasp_candidate_generation.grasp_candidate_generator`:
the node publishes what the generator returned.

The topic is transient-local, so RViz still receives the latest batch when it
starts after the action has completed. A batch the validator went on to reject
is useful diagnostic information and stays visible for the same reason.
"""

from grasp_pose_interface.msg import GraspPoseArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GraspCandidatePublisher:
    """Publish the highest-ranked grasp candidates on a latched topic."""

    def __init__(self, node, parameters):
        """Advertise the candidates topic with transient-local durability.

        ``parameters`` is the node's already-populated
        :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`;
        the topic and the candidate limit both come from it.
        """
        self._node = node
        self._max_candidates = parameters.max_rviz_candidates
        # The latest candidates describe a static scene. Transient-local
        # durability lets RViz receive them even when it starts after the
        # action has completed.
        self._publisher = node.create_publisher(
            GraspPoseArray,
            parameters.grasp_poses_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def publish(self, grasp_poses):
        """Publish the top ``max_candidates`` of ``grasp_poses``."""
        # Keep RViz focused on the same highest-ranked candidates the
        # validator checks.
        rviz_grasp_poses = list(grasp_poses[:self._max_candidates])
        candidates = GraspPoseArray()
        candidates.poses = rviz_grasp_poses
        self._publisher.publish(candidates)
        self._node.get_logger().info(
            f'Published {len(rviz_grasp_poses)} static grasp candidates'
        )

    def clear(self):
        """Drop the transient-local candidates left by an earlier goal."""
        candidates = GraspPoseArray()
        candidates.poses = []
        self._publisher.publish(candidates)

    def destroy(self):
        """Release the publisher this helper owns."""
        self._node.destroy_publisher(self._publisher)
