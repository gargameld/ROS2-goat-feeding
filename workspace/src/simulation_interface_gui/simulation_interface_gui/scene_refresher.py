"""Request simulation state and update the displayed scene."""

from concurrent.futures import Future
import threading

from PyQt5.QtCore import QTimer

from simulation_interface_gui.gui import TopViewWindow
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import RobotState
from simulation_interface_gui.models import SimulationSnapshot
from simulation_interface_gui.presentation import SceneBuilder
from simulation_interface_gui.ros import MujocoClient


class RobotStateDecoder:
    """Decode qpos using the joint order of the current MJCF model."""

    base_qpos_count = 7
    arm_joint_count = 6
    required_qpos_count = base_qpos_count + arm_joint_count

    def decode(self, state: RobotState) -> SimulationSnapshot:
        """Convert MuJoCo qpos into the presentation snapshot contract."""
        qpos = state.qpos
        if len(qpos) < self.required_qpos_count:
            raise ValueError(
                f'Robot state requires at least {self.required_qpos_count} '
                f'qpos values, got {len(qpos)}.'
            )
        values = tuple(float(value) for value in qpos)
        return SimulationSnapshot(
            base_position=Point3D(values[0], values[1], values[2]),
            base_orientation=Quaternion(
                values[3], values[4], values[5], values[6]
            ),
            arm_joint_positions=values[7:13],
            obstacle=state.obstacle,
        )


class SceneRefresher:
    """Coordinate asynchronous simulation-state requests and GUI updates."""

    def __init__(
        self,
        client: MujocoClient,
        window: TopViewWindow,
        *,
        scene_builder: SceneBuilder | None = None,
        state_decoder: RobotStateDecoder | None = None,
        refresh_interval_ms: int = 500,
        timer: QTimer | None = None,
    ) -> None:
        """Create a stopped refresher and its periodic refresh timer."""
        if refresh_interval_ms <= 0:
            raise ValueError('The refresh interval must be positive.')
        self._client = client
        self._window = window
        self._scene_builder = scene_builder or SceneBuilder()
        self._state_decoder = state_decoder or RobotStateDecoder()
        self._request_lock = threading.Lock()
        self._request_in_flight = False
        self._running = False
        self._timer = timer or QTimer(window)
        self._timer.setInterval(refresh_interval_ms)
        self._timer.timeout.connect(self.request_scene_update)

    def start(self) -> None:
        """Start periodic scene refreshes and request the first scene now."""
        if self._running:
            return
        self._running = True
        self._timer.start()
        self.request_scene_update()

    def stop(self) -> None:
        """Stop periodic refreshes and suppress pending GUI updates."""
        self._running = False
        self._timer.stop()

    def request_scene_update(self) -> None:
        """Start one state request unless another request is still pending."""
        if not self._running:
            return
        with self._request_lock:
            if self._request_in_flight:
                return
            self._request_in_flight = True

        try:
            future = self._client.get_robot_state()
            future.add_done_callback(self._finish_scene_update)
            amcl_pose_future = self._client.get_amcl_pose()
            odom_pose_future = self._client.get_odom_pose()
            sim_pose_future = self._client.get_sim_pose()
            for pose_future in (
                amcl_pose_future, odom_pose_future, sim_pose_future,
            ):
                pose_future.add_done_callback(
                    lambda _: self._finish_pose_update(
                        amcl_pose_future, odom_pose_future, sim_pose_future
                    )
                )
        except Exception as error:
            with self._request_lock:
                self._request_in_flight = False
            self._window.set_status(
                f'Could not request robot state: {error}',
                is_error=True,
            )

    def _finish_scene_update(self, future: Future[RobotState]) -> None:
        try:
            state = future.result()
            snapshot = self._state_decoder.decode(state)
            scene = self._scene_builder.build(snapshot)
            if self._running:
                self._window.update_scene(scene)
                self._window.set_obstacle_state(state.obstacle)
                self._window.set_status('Connected to simulation.')
        except Exception as error:
            if self._running:
                self._window.set_status(
                    f'Could not update simulation state: {error}',
                    is_error=True,
                )
        finally:
            with self._request_lock:
                self._request_in_flight = False

    def _finish_pose_update(
        self,
        amcl_pose_future: Future[Pose2D],
        odom_pose_future: Future[Pose2D],
        sim_pose_future: Future[Pose2D],
    ) -> None:
        """Display a pose sample after all three asynchronous results arrive."""
        if not all((
            amcl_pose_future.done(), odom_pose_future.done(), sim_pose_future.done(),
        )):
            return
        try:
            if self._running:
                self._window.set_poses(
                    amcl_pose_future.result(),
                    odom_pose_future.result(),
                    sim_pose_future.result(),
                )
        except Exception as error:
            if self._running:
                self._window.set_status(
                    f'Could not update robot poses: {error}', is_error=True
                )
