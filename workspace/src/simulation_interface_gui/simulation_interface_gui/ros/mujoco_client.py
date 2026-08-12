"""Non-blocking ROS operations needed by the simulation interface GUI."""

from concurrent.futures import Future
import math
from typing import Sequence

from geometry_msgs.msg import TwistStamped
from mujoco_ros2_control_msgs.srv import GetRobotState
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformListener

from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.ros.ros_runtime import RosRuntime


class ServiceUnavailableError(RuntimeError):
    """Indicate that the MuJoCo state service is not currently available."""


class MujocoClient:
    """Publish velocity commands and request live MuJoCo state."""

    def __init__(
        self,
        runtime: RosRuntime,
        *,
        cmd_vel_topic: str = '/mecanum_drive_controller/reference',
        robot_state_service: str = '/simulation_management/get_robot_state',
        map_frame: str = 'map',
        odom_frame: str = 'odom',
        base_frame: str = 'base_link',
        cmd_vel_publish_period: float = 0.05,
    ) -> None:
        """Create the ROS publisher and service client on the ROS thread."""
        if (
            not math.isfinite(cmd_vel_publish_period)
            or cmd_vel_publish_period <= 0.0
        ):
            raise ValueError('The velocity publish period must be positive.')
        self._runtime = runtime
        self._cmd_vel_topic = cmd_vel_topic
        self._robot_state_service = robot_state_service
        self._map_frame = map_frame
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._cmd_vel_publish_period = cmd_vel_publish_period
        self._publisher = None
        self._command_timer = None
        self._state_client = None
        self._tf_buffer = None
        self._tf_listener = None
        self._current_velocity_values: tuple[float, ...] | None = None
        self._closed = False
        self._ready = runtime.submit(self._initialize)

    def change_cmd_vel(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_x: float = 0.0,
        angular_y: float = 0.0,
        angular_z: float = 0.0,
    ) -> Future[None]:
        """
        Queue a new ``TwistStamped`` controller reference for publication.

        Values are copied before scheduling, so callers may safely invoke this
        method from a GUI event handler.
        """
        values = self._validate_values((
            linear_x,
            linear_y,
            linear_z,
            angular_x,
            angular_y,
            angular_z,
        ))

        def publish() -> None:
            self._ensure_open()
            self._current_velocity_values = values
            self._publish_velocity(values)

        return self._runtime.submit(publish)

    def get_robot_state(self) -> Future[list[float]]:
        """Request MuJoCo qpos and return it through a GUI-safe future."""
        result: Future[list[float]] = Future()

        def request_state() -> None:
            if not result.set_running_or_notify_cancel():
                return
            try:
                self._ensure_open()
                if not self._state_client.service_is_ready():
                    raise ServiceUnavailableError(
                        f'ROS service {self._robot_state_service!r} is unavailable.'
                    )

                response_future = self._state_client.call_async(
                    GetRobotState.Request()
                )
                response_future.add_done_callback(finish_request)
            except BaseException as error:
                result.set_exception(error)

        def finish_request(response_future: object) -> None:
            try:
                response = response_future.result()
                if response is None:
                    raise RuntimeError('The robot-state service returned no response.')
                result.set_result([float(value) for value in response.qpos])
            except BaseException as error:
                result.set_exception(error)

        try:
            self._runtime.submit(request_state)
        except BaseException as error:
            result.set_exception(error)
        return result

    def get_amcl_pose(self) -> Future[Pose2D]:
        """Return the latest ``map -> base_link`` pose published by AMCL."""
        return self._runtime.submit(
            lambda: self._lookup_transform_pose(self._map_frame)
        )

    def get_odom_pose(self) -> Future[Pose2D]:
        """Return the latest ``odom -> base_link`` pose from the EKF."""
        return self._runtime.submit(
            lambda: self._lookup_transform_pose(self._odom_frame)
        )

    def get_sim_pose(self) -> Future[Pose2D]:
        """Return the MuJoCo free-joint planar pose from the state plugin."""
        result: Future[Pose2D] = Future()

        def finish_request(state_future: Future[list[float]]) -> None:
            try:
                qpos = state_future.result()
                if len(qpos) < 7:
                    raise RuntimeError('MuJoCo state does not contain a free joint.')
                yaw = math.atan2(
                    2.0 * (qpos[3] * qpos[6] + qpos[4] * qpos[5]),
                    1.0 - 2.0 * (qpos[5] ** 2 + qpos[6] ** 2),
                )
                # The occupancy map is generated directly from the MuJoCo
                # scene, so qpos already uses the map's world coordinates.
                result.set_result(Pose2D(qpos[0], qpos[1], yaw))
            except BaseException as error:
                result.set_exception(error)

        self.get_robot_state().add_done_callback(finish_request)
        return result

    def close(self) -> Future[None]:
        """Destroy this client's ROS entities on the executor thread."""
        return self._runtime.submit(self._close_on_ros_thread)

    def _initialize(self) -> None:
        self._publisher = self._runtime.node.create_publisher(
            TwistStamped,
            self._cmd_vel_topic,
            10,
        )
        self._command_timer = self._runtime.node.create_timer(
            self._cmd_vel_publish_period,
            self._republish_velocity,
        )
        self._state_client = self._runtime.node.create_client(
            GetRobotState,
            self._robot_state_service,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._runtime.node, spin_thread=False
        )

    def _lookup_transform_pose(self, parent_frame: str) -> Pose2D:
        self._ensure_open()
        transform = self._tf_buffer.lookup_transform(
            parent_frame, self._base_frame, Time()
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )
        return Pose2D(float(translation.x), float(translation.y), yaw)

    def _republish_velocity(self) -> None:
        if self._current_velocity_values is not None and not self._closed:
            self._publish_velocity(self._current_velocity_values)

    def _publish_velocity(self, values: tuple[float, ...]) -> None:
        message = TwistStamped()
        message.header.stamp = self._runtime.node.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.linear.y = values[:2]
        message.twist.linear.z = values[2]
        message.twist.angular.x, message.twist.angular.y = values[3:5]
        message.twist.angular.z = values[5]
        self._publisher.publish(message)

    def _close_on_ros_thread(self) -> None:
        if self._closed:
            return
        if self._publisher is not None and self._current_velocity_values is not None:
            self._publish_velocity((0.0,) * 6)
            self._current_velocity_values = None
        if self._command_timer is not None:
            self._runtime.node.destroy_timer(self._command_timer)
            self._command_timer = None
        if self._publisher is not None:
            self._runtime.node.destroy_publisher(self._publisher)
            self._publisher = None
        if self._state_client is not None:
            self._runtime.node.destroy_client(self._state_client)
            self._state_client = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        self._tf_buffer = None
        self._closed = True

    def _ensure_open(self) -> None:
        self._ready.result()
        if self._closed:
            raise RuntimeError('The MuJoCo client is closed.')

    @staticmethod
    def _validate_values(values: Sequence[float]) -> tuple[float, ...]:
        converted = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in converted):
            raise ValueError('Velocity values must be finite numbers.')
        return converted
