"""Non-blocking ROS operations needed by the simulation interface GUI."""

from concurrent.futures import Future
import math

from behavior_interface.srv import RequestFood
from mujoco_ros2_control_msgs.srv import GetRobotState
from mujoco_ros2_control_msgs.srv import SetObstacle
from mujoco_ros2_control_msgs.srv import ThrowFood
from rclpy.time import Time

from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Pose2D
from simulation_interface_gui.models import RobotState
from simulation_interface_gui.models import ThrowFoodCommand
from simulation_interface_gui.ros.ros_runtime import RosRuntime
from tf2_ros import Buffer
from tf2_ros import TransformListener


class ServiceUnavailableError(RuntimeError):
    """Indicate that the MuJoCo state service is not currently available."""


class MujocoClient:
    """Throw food, edit obstacles, and request live MuJoCo state."""

    def __init__(
        self,
        runtime: RosRuntime,
        *,
        robot_state_service: str = '/simulation_management/get_robot_state',
        obstacle_service: str = '/simulation_management/set_obstacle',
        throw_food_service: str = '/simulation_management/throw_food',
        food_request_service: str = '/request_food',
        food_body_prefix: str = 'food_',
        map_frame: str = 'map',
        odom_frame: str = 'odom',
        base_frame: str = 'base_link',
    ) -> None:
        """Create the ROS service clients on the ROS thread."""
        self._runtime = runtime
        self._robot_state_service = robot_state_service
        self._obstacle_service = obstacle_service
        self._throw_food_service = throw_food_service
        self._food_request_service = food_request_service
        self._food_body_prefix = food_body_prefix
        self._map_frame = map_frame
        self._odom_frame = odom_frame
        self._base_frame = base_frame
        self._state_client = None
        self._obstacle_client = None
        self._throw_food_client = None
        self._food_request_client = None
        self._tf_buffer = None
        self._tf_listener = None
        self._closed = False
        self._ready = runtime.submit(self._initialize)

    def throw_food(self, command: ThrowFoodCommand) -> Future[None]:
        """Request that a food body be teleported into a parking area."""
        values = self._validate_throw_food(command)
        result: Future[None] = Future()

        def request_throw() -> None:
            if not result.set_running_or_notify_cancel():
                return
            try:
                self._ensure_open()
                if not self._throw_food_client.service_is_ready():
                    raise ServiceUnavailableError(
                        f'ROS service {self._throw_food_service!r} is unavailable.'
                    )
                request = ThrowFood.Request()
                request.parking_index = values[0]
                request.food_name = values[1]
                request.x, request.y = values[2:4]
                request.orientation = list(values[4])
                response_future = self._throw_food_client.call_async(request)
                response_future.add_done_callback(finish_request)
            except BaseException as error:
                result.set_exception(error)

        def finish_request(response_future: object) -> None:
            try:
                response = response_future.result()
                if response is None:
                    raise RuntimeError('The throw-food service returned no response.')
                if not response.success:
                    raise RuntimeError(response.message or 'Throwing food failed.')
                result.set_result(None)
            except BaseException as error:
                result.set_exception(error)

        try:
            self._runtime.submit(request_throw)
        except BaseException as error:
            result.set_exception(error)
        return result

    def request_food(self, parking_number: int) -> Future[None]:
        """Send a parking-specific food request to the behavior node."""
        parking_number = int(parking_number)
        if not 1 <= parking_number <= 4:
            raise ValueError('The parking number must be between 1 and 4.')
        result: Future[None] = Future()

        def send_request() -> None:
            if not result.set_running_or_notify_cancel():
                return
            try:
                self._ensure_open()
                if not self._food_request_client.service_is_ready():
                    raise ServiceUnavailableError(
                        'ROS service '
                        f'{self._food_request_service!r} is unavailable.'
                    )
                request = RequestFood.Request()
                request.parking_number = parking_number
                response_future = self._food_request_client.call_async(request)
                response_future.add_done_callback(finish_request)
            except BaseException as error:
                result.set_exception(error)

        def finish_request(response_future: object) -> None:
            try:
                response = response_future.result()
                if response is None:
                    raise RuntimeError(
                        'The food-request service returned no response.'
                    )
                if not response.success:
                    raise RuntimeError(
                        response.message or 'Food request was rejected.'
                    )
                result.set_result(None)
            except BaseException as error:
                result.set_exception(error)

        try:
            self._runtime.submit(send_request)
        except BaseException as error:
            result.set_exception(error)
        return result

    def get_robot_state(self) -> Future[RobotState]:
        """Request MuJoCo robot and obstacle state through a GUI-safe future."""
        result: Future[RobotState] = Future()

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
                result.set_result(RobotState(
                    qpos=tuple(float(value) for value in response.qpos),
                    obstacle=ObstacleState(
                        position=Point3D(
                            float(response.obstacle_position.x),
                            float(response.obstacle_position.y),
                            float(response.obstacle_position.z),
                        ),
                        width=float(response.obstacle_size.x),
                        length=float(response.obstacle_size.y),
                        height=float(response.obstacle_size.z),
                    ),
                ))
            except BaseException as error:
                result.set_exception(error)

        try:
            self._runtime.submit(request_state)
        except BaseException as error:
            result.set_exception(error)
        return result

    def set_obstacle(self, obstacle: ObstacleState) -> Future[None]:
        """Request a new XY position for the floor box."""
        values = self._validate_obstacle(obstacle)
        result: Future[None] = Future()

        def request_update() -> None:
            if not result.set_running_or_notify_cancel():
                return
            try:
                self._ensure_open()
                if not self._obstacle_client.service_is_ready():
                    raise ServiceUnavailableError(
                        f'ROS service {self._obstacle_service!r} is unavailable.'
                    )
                request = SetObstacle.Request()
                request.position.x, request.position.y, request.position.z = values
                response_future = self._obstacle_client.call_async(request)
                response_future.add_done_callback(finish_request)
            except BaseException as error:
                result.set_exception(error)

        def finish_request(response_future: object) -> None:
            try:
                response = response_future.result()
                if response is None:
                    raise RuntimeError('The obstacle service returned no response.')
                if not response.success:
                    raise RuntimeError(response.message or 'Obstacle update failed.')
                result.set_result(None)
            except BaseException as error:
                result.set_exception(error)

        try:
            self._runtime.submit(request_update)
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

        def finish_request(state_future: Future[RobotState]) -> None:
            try:
                qpos = state_future.result().qpos
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
        self._state_client = self._runtime.node.create_client(
            GetRobotState,
            self._robot_state_service,
        )
        self._obstacle_client = self._runtime.node.create_client(
            SetObstacle,
            self._obstacle_service,
        )
        self._throw_food_client = self._runtime.node.create_client(
            ThrowFood,
            self._throw_food_service,
        )
        self._food_request_client = self._runtime.node.create_client(
            RequestFood,
            self._food_request_service,
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

    def _close_on_ros_thread(self) -> None:
        if self._closed:
            return
        if self._state_client is not None:
            self._runtime.node.destroy_client(self._state_client)
            self._state_client = None
        if self._obstacle_client is not None:
            self._runtime.node.destroy_client(self._obstacle_client)
            self._obstacle_client = None
        if self._throw_food_client is not None:
            self._runtime.node.destroy_client(self._throw_food_client)
            self._throw_food_client = None
        if self._food_request_client is not None:
            self._runtime.node.destroy_client(self._food_request_client)
            self._food_request_client = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        self._tf_buffer = None
        self._closed = True

    def _ensure_open(self) -> None:
        self._ready.result()
        if self._closed:
            raise RuntimeError('The MuJoCo client is closed.')

    def _validate_throw_food(
        self, command: ThrowFoodCommand
    ) -> tuple[int, str, float, float, tuple[float, float, float, float]]:
        name = command.food_name.strip()
        if not name:
            raise ValueError('The food object name must not be empty.')
        body_name = (
            name if name.startswith(self._food_body_prefix)
            else self._food_body_prefix + name
        )
        parking_index = int(command.parking_index)
        planar = (float(command.x), float(command.y))
        if not all(math.isfinite(value) for value in planar):
            raise ValueError('The throw x/y position must be finite numbers.')
        orientation = (
            float(command.orientation.w),
            float(command.orientation.x),
            float(command.orientation.y),
            float(command.orientation.z),
        )
        if not all(math.isfinite(value) for value in orientation):
            raise ValueError('The orientation quaternion must be finite numbers.')
        if all(value == 0.0 for value in orientation):
            raise ValueError('The orientation quaternion must be non-zero.')
        return (parking_index, body_name, planar[0], planar[1], orientation)

    @staticmethod
    def _validate_obstacle(obstacle: ObstacleState) -> tuple[float, ...]:
        values = (
            obstacle.position.x,
            obstacle.position.y,
            obstacle.position.z,
        )
        converted = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in converted):
            raise ValueError('Obstacle position must contain finite numbers.')
        return converted
