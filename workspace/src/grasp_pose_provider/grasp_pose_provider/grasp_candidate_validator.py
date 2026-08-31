"""Validation of the grasp candidates the generator produced.

The second of the two parts :mod:`grasp_pose_provider.node_initializer` builds
the node from -- the other being ``GraspCandidateGenerator``, in
:mod:`grasp_pose_provider.grasp_candidate_generation.grasp_candidate_generator`.
It takes the ranked candidates and hands back the first one MoveIt can actually
plan a motion to, by asking the arm's reachability action through
:class:`grasp_pose_provider.grasp_reachability.GraspReachabilityChecker`.
"""

from grasp_pose_provider import grasp_reachability


class GraspCandidateValidator:
    """Pick the first reachable candidate out of a ranked list."""

    def __init__(self, node, parameters, callback_group=None):
        """Create the reachability client this part owns.

        ``parameters`` is the node's already-populated
        :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`;
        the action name, the candidate limit and the two timeouts all come
        from it. ``callback_group`` should be reentrant so the executor can
        deliver the arm-action responses while :meth:`first_reachable` waits
        on them.
        """
        self._node = node
        self._reachability_checker = (
            grasp_reachability.GraspReachabilityChecker(
                node,
                parameters,
                callback_group=callback_group,
            )
        )

    def first_reachable(self, grasp_poses, feedback_cb=None):
        """Return the first ``PoseStamped`` in ``grasp_poses`` MoveIt can reach.

        Raises ``RuntimeError`` when the list is empty, when the arm's
        reachability action is unavailable, or when none of the checked
        candidates can be planned to. ``feedback_cb``, if given, is called with
        a short stage string before the checking starts.
        """
        if feedback_cb is not None:
            feedback_cb('Checking reachability of the top grasp candidates')
        return self._reachability_checker.first_reachable(grasp_poses)

    def destroy(self):
        """Release the reachability client this part owns."""
        self._reachability_checker.destroy()
