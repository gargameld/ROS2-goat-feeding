"""Update the presentation simulation state from the MuJoCo simulation."""

from abc import ABC
from abc import abstractmethod
from collections.abc import Callable
from concurrent.futures import Future
import threading

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import RobotState
from simulation_interface_gui.presentation.kinematics import quaternion_to_yaw
from simulation_interface_gui.presentation.simulation_state import PoseEstimate
from simulation_interface_gui.presentation.simulation_state import SimulationState
from simulation_interface_gui.ros import MujocoClient


class SimulationStateProvider(ABC):
    """Produce complete simulation states without blocking the GUI thread."""

    @abstractmethod
    def request_simulation_state(self) -> Future[SimulationState]:
        """Start one update and return the future carrying its result."""


class RobotStateDecoder:
    """Decode one robot-state service response into presentation values."""

    base_qpos_count = 7
    arm_point_count = 7

    def decode(
        self,
        state: RobotState,
        *,
        amcl_pose: PoseEstimate,
        odom_pose: PoseEstimate,
    ) -> SimulationState:
        """Convert MuJoCo state and transform samples into one state."""
        qpos = state.qpos
        if len(qpos) < self.base_qpos_count:
            raise ValueError(
                f'Robot state requires at least {self.base_qpos_count} '
                f'qpos values, got {len(qpos)}.'
            )
        if len(state.arm_points_world) != self.arm_point_count:
            raise ValueError(
                f'Robot state requires {self.arm_point_count} arm points, '
                f'got {len(state.arm_points_world)}.'
            )
        values = tuple(float(value) for value in qpos)
        base_position = Point3D(values[0], values[1], values[2])
        base_orientation = Quaternion(
            values[3], values[4], values[5], values[6]
        )
        return SimulationState(
            base_position=base_position,
            base_orientation=base_orientation,
            arm_points_world=state.arm_points_world,
            obstacle=state.obstacle,
            amcl_pose=amcl_pose,
            odom_pose=odom_pose,
            sim_pose=PoseEstimate.of(Pose2D(
                base_position.x,
                base_position.y,
                quaternion_to_yaw(base_orientation),
            )),
        )


class MujocoSimulationStateProvider(SimulationStateProvider):
    """Collect robot state and localisation poses through the MuJoCo client."""

    def __init__(
        self,
        client: MujocoClient,
        *,
        state_decoder: RobotStateDecoder | None = None,
    ) -> None:
        """Store the ROS client used for every simulation-state update."""
        self._client = client
        self._state_decoder = state_decoder or RobotStateDecoder()

    def request_simulation_state(self) -> Future[SimulationState]:
        """Request robot state and poses, then combine them into one state."""
        result: Future[SimulationState] = Future()
        result.set_running_or_notify_cancel()
        try:
            robot_state = self._client.get_robot_state()
            amcl_pose = self._client.get_amcl_pose()
            odom_pose = self._client.get_odom_pose()
        except BaseException as error:
            result.set_exception(error)
            return result

        def complete() -> None:
            self._finish(robot_state, amcl_pose, odom_pose, result)

        _CompletionCounter(
            (robot_state, amcl_pose, odom_pose), complete
        ).watch()
        return result

    def _finish(
        self,
        robot_state: Future[RobotState],
        amcl_pose: Future[Pose2D],
        odom_pose: Future[Pose2D],
        result: Future[SimulationState],
    ) -> None:
        try:
            # Only the MuJoCo state is essential; a missing transform is
            # reported inside its own estimate so the scene still updates.
            result.set_result(self._state_decoder.decode(
                robot_state.result(),
                amcl_pose=_estimate(amcl_pose),
                odom_pose=_estimate(odom_pose),
            ))
        except BaseException as error:
            result.set_exception(error)


class _CompletionCounter:
    """Call one callback after every watched future has finished."""

    def __init__(
        self,
        futures: tuple[Future, ...],
        on_complete: Callable[[], None],
    ) -> None:
        self._futures = futures
        self._on_complete = on_complete
        self._lock = threading.Lock()
        self._pending = len(futures)

    def watch(self) -> None:
        """Subscribe to every watched future."""
        for future in self._futures:
            future.add_done_callback(self._finish_one)

    def _finish_one(self, _future: Future) -> None:
        with self._lock:
            self._pending -= 1
            completed = self._pending == 0
        if completed:
            self._on_complete()


def _estimate(future: Future[Pose2D]) -> PoseEstimate:
    """Return an available or unavailable estimate for one pose source."""
    try:
        return PoseEstimate.of(future.result())
    except BaseException as error:
        return PoseEstimate.unavailable(str(error) or type(error).__name__)
