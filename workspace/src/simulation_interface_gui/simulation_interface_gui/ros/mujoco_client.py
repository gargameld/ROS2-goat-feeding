"""Non-blocking ROS operations needed by the simulation interface GUI."""

from concurrent.futures import Future
import math
from typing import Sequence

from geometry_msgs.msg import Twist
from mujoco_ros2_control_msgs.srv import GetRobotState

from simulation_interface_gui.ros.ros_runtime import RosRuntime


class ServiceUnavailableError(RuntimeError):
    """Indicate that the MuJoCo state service is not currently available."""


class MujocoClient:
    """Publish velocity commands and request live MuJoCo state."""

    def __init__(
        self,
        runtime: RosRuntime,
        *,
        cmd_vel_topic: str = '/cmd_vel',
        robot_state_service: str = '/controller_manager/get_robot_state',
    ) -> None:
        """Create the ROS publisher and service client on the ROS thread."""
        self._runtime = runtime
        self._cmd_vel_topic = cmd_vel_topic
        self._robot_state_service = robot_state_service
        self._publisher = None
        self._state_client = None
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
        Queue a new ``Twist`` message for publication.

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
            message = Twist()
            message.linear.x, message.linear.y, message.linear.z = values[:3]
            message.angular.x, message.angular.y, message.angular.z = values[3:]
            self._publisher.publish(message)

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

    def close(self) -> Future[None]:
        """Destroy this client's ROS entities on the executor thread."""
        return self._runtime.submit(self._close_on_ros_thread)

    def _initialize(self) -> None:
        self._publisher = self._runtime.node.create_publisher(
            Twist,
            self._cmd_vel_topic,
            10,
        )
        self._state_client = self._runtime.node.create_client(
            GetRobotState,
            self._robot_state_service,
        )

    def _close_on_ros_thread(self) -> None:
        if self._closed:
            return
        if self._publisher is not None:
            self._runtime.node.destroy_publisher(self._publisher)
            self._publisher = None
        if self._state_client is not None:
            self._runtime.node.destroy_client(self._state_client)
            self._state_client = None
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
