"""Receive parking-specific food requests for the behavior state machine."""

from typing import Callable

from behavior_interface.srv import RequestFood

from rclpy.node import Node


class RequestListener:
    """Validate food requests and pass accepted parking numbers onward."""

    MIN_PARKING_NUMBER = 1
    MAX_PARKING_NUMBER = 4

    def __init__(
        self,
        node: Node,
        change_parking_request: Callable[[int], None],
        is_busy: Callable[[], bool],
        service_name: str = 'request_food',
    ) -> None:
        """Offer the food-request service on ``node``."""
        self._node = node
        self._change_parking_request = change_parking_request
        self._is_busy = is_busy
        self._service = node.create_service(
            RequestFood, service_name, self._handle_request
        )

    def _handle_request(
        self, request: RequestFood.Request, response: RequestFood.Response
    ) -> RequestFood.Response:
        parking_number = int(request.parking_number)
        parking_is_valid = (
            self.MIN_PARKING_NUMBER
            <= parking_number
            <= self.MAX_PARKING_NUMBER
        )
        if not parking_is_valid:
            response.success = False
            response.message = 'Parking number must be between 1 and 4.'
            self._node.get_logger().warning(
                f'Rejected food request for parking {parking_number}'
            )
            return response

        if self._is_busy():
            response.success = False
            response.message = 'State machine is busy.'
            self._node.get_logger().warning(
                f'Rejected food request for parking {parking_number}: '
                'state machine is busy'
            )
            return response

        self._change_parking_request(parking_number)
        response.success = True
        response.message = (
            f'Food request accepted for parking {parking_number}.'
        )
        self._node.get_logger().info(response.message)
        return response
