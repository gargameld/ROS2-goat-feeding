"""Convenient asynchronous access to ROS actions and services."""

from dataclasses import dataclass
from typing import Any, Callable, Optional

from arm_interface.action import LiftGripper, MoveArmToHomePose, MoveArmToPose
from arm_interface.srv import AttachObjectToGripper
from control_msgs.action import GripperCommand
from grasp_pose_interface.action import ProvideGraspPose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.task import Future
from std_srvs.srv import Empty


Handler = Callable[[Any], None]
ATTACH_OBJECT_TO_GRIPPER = 'attach_object_to_gripper'
ATTACH_OBJECT_TO_GRIPPER_SERVICE = '/attach_object_to_gripper'
CLEAR_OCTOMAP = 'clear_octomap'
CLEAR_OCTOMAP_SERVICE = '/clear_octomap'
CLOSE_GRIPPER = 'close_gripper'
CLOSE_GRIPPER_ACTION = '/gripper_controller/gripper_cmd'
CLOSED_GRIPPER_POSITION = 0.725
GRIPPER_MAX_EFFORT = 5.0
LIFT_DISTANCE_METERS = 0.1
LIFT_GRIPPER = 'lift_gripper'
LIFT_GRIPPER_ACTION = '/lift_gripper'
MOVE_ARM_TO_HOME = 'move_arm_to_home'
MOVE_ARM_TO_HOME_ACTION = '/move_arm_to_home_pose'
MOVE_ARM_TO_POSE = 'move_arm_to_pose'
MOVE_ARM_TO_POSE_ACTION = '/move_arm_to_pose'
NAVIGATE_TO_PARKING = 'navigate_to_parking'
NAVIGATE_TO_POSE_ACTION = '/navigate_to_pose'
PROVIDE_GRASP_POSE = 'provide_grasp_pose'
PROVIDE_GRASP_POSE_ACTION = '/provide_grasp_pose'
READINESS_POLL_PERIOD_SECONDS = 0.1


@dataclass
class _Action:
    client: ActionClient
    goal_response_handler: Optional[Handler] = None
    feedback_handler: Optional[Handler] = None
    result_handler: Optional[Handler] = None


@dataclass
class _Service:
    client: Any
    response_handler: Optional[Handler] = None


class BehaviorClient:
    """Own action and service clients used by robot behavior states.

    Clients are registered under short names so states do not need to know ROS
    endpoint construction details. Calls are asynchronous; configured handlers
    are invoked as responses arrive while the behavior node is spinning.
    """

    def __init__(self, node):
        self.node = node
        self._actions: dict[str, _Action] = {}
        self._services: dict[str, _Service] = {}
        self.register_action(
            MOVE_ARM_TO_HOME,
            MoveArmToHomePose,
            MOVE_ARM_TO_HOME_ACTION,
        )
        self.register_action(
            NAVIGATE_TO_PARKING,
            NavigateToPose,
            NAVIGATE_TO_POSE_ACTION,
        )
        self.register_action(
            PROVIDE_GRASP_POSE,
            ProvideGraspPose,
            PROVIDE_GRASP_POSE_ACTION,
        )
        self.register_action(
            MOVE_ARM_TO_POSE,
            MoveArmToPose,
            MOVE_ARM_TO_POSE_ACTION,
        )
        self.register_action(
            CLOSE_GRIPPER,
            GripperCommand,
            CLOSE_GRIPPER_ACTION,
        )
        self.register_action(
            LIFT_GRIPPER,
            LiftGripper,
            LIFT_GRIPPER_ACTION,
        )
        self.register_service(
            ATTACH_OBJECT_TO_GRIPPER,
            AttachObjectToGripper,
            ATTACH_OBJECT_TO_GRIPPER_SERVICE,
        )
        self.register_service(
            CLEAR_OCTOMAP,
            Empty,
            CLEAR_OCTOMAP_SERVICE,
        )

    def move_arm_to_home(
        self,
        *,
        goal_response_handler: Optional[Handler] = None,
        feedback_handler: Optional[Handler] = None,
        result_handler: Optional[Handler] = None,
    ):
        """Start moving the arm to its configured home pose."""
        self.set_action_handlers(
            MOVE_ARM_TO_HOME,
            goal_response=goal_response_handler,
            feedback=feedback_handler,
            result=result_handler,
        )
        return self.invoke_action(
            MOVE_ARM_TO_HOME,
            MoveArmToHomePose.Goal(),
        )

    def navigate_to_pose(
        self,
        target_pose,
        *,
        goal_response_handler: Optional[Handler] = None,
        feedback_handler: Optional[Handler] = None,
        result_handler: Optional[Handler] = None,
    ):
        """Navigate the mobile base to a map-frame pose."""
        self.set_action_handlers(
            NAVIGATE_TO_PARKING,
            goal_response=goal_response_handler,
            feedback=feedback_handler,
            result=result_handler,
        )
        goal = NavigateToPose.Goal()
        goal.pose = target_pose
        return self.invoke_action(NAVIGATE_TO_PARKING, goal)

    def provide_grasp_pose(
        self,
        *,
        goal_response_handler: Optional[Handler] = None,
        feedback_handler: Optional[Handler] = None,
        result_handler: Optional[Handler] = None,
    ):
        """Ask the grasp-pose provider for a reachable food grasp."""
        self.set_action_handlers(
            PROVIDE_GRASP_POSE,
            goal_response=goal_response_handler,
            feedback=feedback_handler,
            result=result_handler,
        )
        return self.invoke_action(
            PROVIDE_GRASP_POSE,
            ProvideGraspPose.Goal(),
        )

    def move_arm_to_pose(
        self,
        target_pose,
        reference_frame: str,
        *,
        goal_response_handler: Optional[Handler] = None,
        feedback_handler: Optional[Handler] = None,
        result_handler: Optional[Handler] = None,
    ):
        """Move the arm to ``target_pose`` expressed in ``reference_frame``."""
        self.set_action_handlers(
            MOVE_ARM_TO_POSE,
            goal_response=goal_response_handler,
            feedback=feedback_handler,
            result=result_handler,
        )
        goal = MoveArmToPose.Goal()
        goal.target_pose = target_pose
        goal.reference_frame = reference_frame
        return self.invoke_action(MOVE_ARM_TO_POSE, goal)

    def close_gripper(
        self,
        *,
        goal_response_handler: Optional[Handler] = None,
        feedback_handler: Optional[Handler] = None,
        result_handler: Optional[Handler] = None,
    ):
        """Close the gripper until it reaches its limit or stalls on an object."""
        self.set_action_handlers(
            CLOSE_GRIPPER,
            goal_response=goal_response_handler,
            feedback=feedback_handler,
            result=result_handler,
        )
        goal = GripperCommand.Goal()
        goal.command.position = CLOSED_GRIPPER_POSITION
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        return self.invoke_action(CLOSE_GRIPPER, goal)

    def lift_gripper(
        self,
        distance: float = LIFT_DISTANCE_METERS,
        *,
        goal_response_handler: Optional[Handler] = None,
        feedback_handler: Optional[Handler] = None,
        result_handler: Optional[Handler] = None,
    ):
        """Lift the gripper vertically by ``distance`` metres."""
        self.set_action_handlers(
            LIFT_GRIPPER,
            goal_response=goal_response_handler,
            feedback=feedback_handler,
            result=result_handler,
        )
        goal = LiftGripper.Goal()
        goal.distance = distance
        return self.invoke_action(LIFT_GRIPPER, goal)

    def attach_object_to_gripper(
        self,
        *,
        response_handler: Optional[Handler] = None,
    ):
        """Attach the configured payload box to the gripper in MoveIt."""
        self.set_service_handler(
            ATTACH_OBJECT_TO_GRIPPER,
            response_handler,
        )
        return self.invoke_service(
            ATTACH_OBJECT_TO_GRIPPER,
            AttachObjectToGripper.Request(),
        )

    def clear_octomap(
        self,
        *,
        response_handler: Optional[Handler] = None,
    ):
        """Drop every occupied voxel from the move_group octomap."""
        self.set_service_handler(CLEAR_OCTOMAP, response_handler)
        return self.invoke_service(CLEAR_OCTOMAP, Empty.Request())

    def register_action(
        self,
        name: str,
        action_type,
        action_name: str,
        *,
        callback_group=None,
    ) -> None:
        """Create and register an action client."""
        if name in self._actions:
            raise ValueError(f'Action client {name!r} is already registered')

        self._actions[name] = _Action(
            ActionClient(
                self.node,
                action_type,
                action_name,
                callback_group=callback_group,
            )
        )

    def set_action_handlers(
        self,
        name: str,
        *,
        goal_response: Optional[Handler] = None,
        feedback: Optional[Handler] = None,
        result: Optional[Handler] = None,
    ) -> None:
        """Set the default handlers for an action client."""
        action = self._get_action(name)
        action.goal_response_handler = goal_response
        action.feedback_handler = feedback
        action.result_handler = result

    def invoke_action(self, name: str, goal):
        """Send a goal once its action server is ready."""
        action = self._get_action(name)

        def send_goal():
            feedback_callback = None
            if action.feedback_handler is not None:
                feedback_callback = lambda message: action.feedback_handler(
                    message.feedback
                )

            future = action.client.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            )
            future.add_done_callback(
                lambda completed: self._handle_goal_response(
                    action, completed
                )
            )
            return future

        return self._invoke_when_ready(
            'action',
            name,
            action.client.server_is_ready,
            send_goal,
        )

    def register_service(
        self,
        name: str,
        service_type,
        service_name: str,
        *,
        callback_group=None,
    ) -> None:
        """Create and register a service client."""
        if name in self._services:
            raise ValueError(f'Service client {name!r} is already registered')

        client = self.node.create_client(
            service_type,
            service_name,
            callback_group=callback_group,
        )
        self._services[name] = _Service(client)

    def set_service_handler(
        self,
        name: str,
        response: Optional[Handler],
    ) -> None:
        """Set the default response handler for a service client."""
        self._get_service(name).response_handler = response

    def invoke_service(self, name: str, request):
        """Call a service once its server is ready."""
        service = self._get_service(name)

        def call_service():
            future = service.client.call_async(request)
            future.add_done_callback(
                lambda completed: self._handle_service_response(
                    service, completed
                )
            )
            return future

        return self._invoke_when_ready(
            'service',
            name,
            service.client.service_is_ready,
            call_service,
        )

    def action_is_ready(self, name: str) -> bool:
        """Return whether the named action server is ready."""
        return self._get_action(name).client.server_is_ready()

    def service_is_ready(self, name: str) -> bool:
        """Return whether the named service server is ready."""
        return self._get_service(name).client.service_is_ready()

    def _invoke_when_ready(
        self,
        endpoint_kind: str,
        name: str,
        is_ready: Callable[[], bool],
        invoke: Callable[[], Future],
    ) -> Future:
        if is_ready():
            return invoke()

        self.node.get_logger().info(
            f'Waiting for {endpoint_kind} client {name!r} to become ready'
        )
        waiting_future = Future()

        def try_invoke() -> None:
            if not is_ready():
                return

            self.node.destroy_timer(readiness_timer)
            self.node.get_logger().info(
                f'{endpoint_kind.capitalize()} client {name!r} is ready'
            )
            try:
                invocation_future = invoke()
            except Exception as exc:  # noqa: BLE001
                waiting_future.set_exception(exc)
                return

            invocation_future.add_done_callback(
                lambda completed: self._forward_future(
                    completed, waiting_future
                )
            )

        readiness_timer = self.node.create_timer(
            READINESS_POLL_PERIOD_SECONDS,
            try_invoke,
        )
        return waiting_future

    @staticmethod
    def _forward_future(source: Future, destination: Future) -> None:
        try:
            destination.set_result(source.result())
        except Exception as exc:  # noqa: BLE001
            destination.set_exception(exc)

    def _handle_goal_response(self, action: _Action, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error(
                f'Failed to send behavior action goal: {exc}'
            )
            return

        if action.goal_response_handler is not None:
            action.goal_response_handler(goal_handle)

        if not goal_handle.accepted:
            self.node.get_logger().warning('Behavior action goal was rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._handle_action_result(action, completed)
        )

    def _handle_action_result(self, action: _Action, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error(
                f'Failed to receive behavior action result: {exc}'
            )
            return

        if action.result_handler is not None:
            action.result_handler(response.result)

    def _handle_service_response(self, service: _Service, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error(
                f'Behavior service call failed: {exc}'
            )
            return

        if service.response_handler is not None:
            service.response_handler(response)

    def _get_action(self, name: str) -> _Action:
        try:
            return self._actions[name]
        except KeyError as exc:
            raise KeyError(f'Unknown action client {name!r}') from exc

    def _get_service(self, name: str) -> _Service:
        try:
            return self._services[name]
        except KeyError as exc:
            raise KeyError(f'Unknown service client {name!r}') from exc
