"""Tests for ranked grasp reachability selection."""

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from grasp_pose_provider.grasp_reachability import GraspReachabilityChecker
import pytest


class _DoneFuture:
    def __init__(self, value):
        self._value = value

    def done(self):
        return True

    def result(self):
        return self._value


class _GoalHandle:
    accepted = True

    def __init__(self, reachable):
        result = SimpleNamespace(success=reachable, message='planned')
        self._response = SimpleNamespace(result=result)

    def get_result_async(self):
        return _DoneFuture(self._response)


class _ActionClient:
    def __init__(self, reachability):
        self._reachability = iter(reachability)
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _DoneFuture(_GoalHandle(next(self._reachability)))


class _Logger:
    def info(self, message):
        pass

    def warning(self, message):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


def _checker(reachability):
    checker = GraspReachabilityChecker.__new__(GraspReachabilityChecker)
    checker._node = _Node()
    checker._action_name = '/check_pose_reachability'
    checker._client = _ActionClient(reachability)
    return checker


def _candidates(count):
    candidates = []
    for index in range(count):
        candidate = PoseStamped()
        candidate.header.frame_id = 'left_camera_frame'
        candidate.pose.position.x = float(index)
        candidate.pose.orientation.w = 1.0
        candidates.append(candidate)
    return candidates


def test_returns_first_reachable_candidate_and_preserves_frame():
    checker = _checker([False, False, True])

    selected = checker.first_reachable(_candidates(6))

    assert selected.pose.position.x == 2.0
    assert len(checker._client.goals) == 3
    assert checker._client.goals[2].reference_frame == 'left_camera_frame'
    assert checker._client.goals[2].target_pose.position.x == 2.0


def test_checks_at_most_top_forty_candidates():
    checker = _checker([False] * 40)

    with pytest.raises(RuntimeError, match='top 40'):
        checker.first_reachable(_candidates(41))

    assert len(checker._client.goals) == 40
