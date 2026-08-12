"""Coordinate ROS state requests, scene construction, and the Qt GUI."""

from concurrent.futures import Future
import threading
from typing import Sequence

from PyQt5.QtCore import QTimer

from simulation_interface_gui.gui import TopViewWindow
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import SimulationSnapshot
from simulation_interface_gui.models import VelocityCommand
from simulation_interface_gui.presentation import SceneBuilder
from simulation_interface_gui.ros import MujocoClient


class RobotStateDecoder:
    """Decode qpos using the joint order of the current MJCF model."""

    base_qpos_count = 7
    arm_joint_count = 6
    required_qpos_count = base_qpos_count + arm_joint_count

    def decode(self, qpos: Sequence[float]) -> SimulationSnapshot:
        """Convert MuJoCo qpos into the presentation snapshot contract."""
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
        )


class SimulationInterfaceManager:
    """Manage periodic state refreshes and velocity-command callbacks."""

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
        """Connect a ROS client to the GUI without blocking either thread."""
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
        self._window.velocity_command_requested.connect(
            self.handle_velocity_command
        )

    def start(self) -> None:
        """Begin periodic state requests and request the first scene now."""
        if self._running:
            return
        self._running = True
        self._timer.start()
        self.request_scene_update()

    def stop(self) -> None:
        """Stop scheduling new state requests."""
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
            amcl_pose_future.add_done_callback(
                lambda _: self._finish_pose_update(
                    amcl_pose_future, odom_pose_future, sim_pose_future
                )
            )
            odom_pose_future.add_done_callback(
                lambda _: self._finish_pose_update(
                    amcl_pose_future, odom_pose_future, sim_pose_future
                )
            )
            sim_pose_future.add_done_callback(
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

    def handle_velocity_command(self, command: VelocityCommand) -> None:
        """Forward one command from the GUI to the ROS client."""
        try:
            future = self._client.change_cmd_vel(
                linear_x=command.linear_x,
                linear_y=command.linear_y,
                linear_z=command.linear_z,
                angular_x=command.angular_x,
                angular_y=command.angular_y,
                angular_z=command.angular_z,
            )
            future.add_done_callback(self._finish_velocity_command)
        except Exception as error:
            self._window.set_status(
                f'Could not send velocity command: {error}',
                is_error=True,
            )

    def _finish_scene_update(self, future: Future[list[float]]) -> None:
        try:
            snapshot = self._state_decoder.decode(future.result())
            scene = self._scene_builder.build(snapshot)
            if self._running:
                self._window.update_scene(scene)
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
        """Display a paired pose sample once both asynchronous sources arrive."""
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

    def _finish_velocity_command(self, future: Future[None]) -> None:
        try:
            future.result()
            if self._running:
                self._window.set_status('Velocity command sent.')
        except Exception as error:
            if self._running:
                self._window.set_status(
                    f'Could not send velocity command: {error}',
                    is_error=True,
                )
