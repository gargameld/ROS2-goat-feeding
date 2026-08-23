"""Select the first grasp candidate for which MoveIt can produce a plan."""

import time

from arm_interface.action import CheckPoseReachability
import rclpy
from rclpy.action import ActionClient


DEFAULT_ACTION_NAME = '/check_pose_reachability'
DEFAULT_MAX_CANDIDATES = 5
DEFAULT_SERVER_TIMEOUT_SEC = 10.0
DEFAULT_RESULT_TIMEOUT_SEC = 180.0


class GraspReachabilityChecker:
    """Use the arm reachability action to validate ranked grasp candidates."""

    def __init__(self, node, callback_group=None, action_name=DEFAULT_ACTION_NAME):
        self._node = node
        self._action_name = action_name
        self._client = ActionClient(
            node,
            CheckPoseReachability,
            action_name,
            callback_group=callback_group,
        )

    def first_reachable(
        self,
        candidates,
        max_candidates=DEFAULT_MAX_CANDIDATES,
        server_timeout_sec=DEFAULT_SERVER_TIMEOUT_SEC,
        result_timeout_sec=DEFAULT_RESULT_TIMEOUT_SEC,
    ):
        """Return the first reachable ``PoseStamped`` among ranked candidates."""
        candidates_to_check = list(candidates[:max_candidates])
        if not candidates_to_check:
            raise RuntimeError('GPD returned no grasp candidates to validate.')
        if not self._client.wait_for_server(timeout_sec=server_timeout_sec):
            raise RuntimeError(
                f"Reachability action '{self._action_name}' is unavailable."
            )

        for index, candidate in enumerate(candidates_to_check):
            goal = CheckPoseReachability.Goal()
            goal.target_pose = candidate.pose
            goal.reference_frame = candidate.header.frame_id

            send_future = self._client.send_goal_async(goal)
            self._wait_for_future(
                send_future, server_timeout_sec, f'sending candidate {index + 1}'
            )
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError(
                    'Arm rejected the reachability request for grasp candidate '
                    f'{index + 1}; another arm operation may be active.'
                )

            result_future = goal_handle.get_result_async()
            self._wait_for_future(
                result_future,
                result_timeout_sec,
                f'planning for candidate {index + 1}',
            )
            result_response = result_future.result()
            if result_response is not None and result_response.result.success:
                self._node.get_logger().info(
                    f'Grasp candidate {index + 1} is reachable'
                )
                return candidate
            message = (
                result_response.result.message
                if result_response is not None else 'no action result'
            )
            self._node.get_logger().info(
                f'Grasp candidate {index + 1} is unreachable: {message}'
            )

        raise RuntimeError(
            f'None of the top {len(candidates_to_check)} grasp candidates is reachable.'
        )

    @staticmethod
    def _wait_for_future(future, timeout_sec, operation):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f'Timed out while {operation}.')

    def destroy(self):
        """Release the action client owned by this helper."""
        self._client.destroy()
