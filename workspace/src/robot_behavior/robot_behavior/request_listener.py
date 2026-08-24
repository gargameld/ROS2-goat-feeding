"""Receive parking-specific food requests for the behavior state machine."""

from threading import Lock
from typing import Optional

from behavior_interface.srv import RequestFood

from rclpy.node import Node


class RequestListener:
    """Store the parking number from the most recent valid food request."""

    MIN_PARKING_NUMBER = 1
    MAX_PARKING_NUMBER = 4

    def __init__(self, node: Node, service_name: str = 'request_food') -> None:
        """Offer the food-request service on ``node``."""
        self._node = node
        self._lock = Lock()
        self._parking_number: Optional[int] = None
        self._service = node.create_service(
            RequestFood, service_name, self._handle_request
        )

    def get_parking_number(self) -> Optional[int]:
        """Return the last valid parking number, or ``None`` while waiting."""
        with self._lock:
            return self._parking_number

    def reset(self) -> None:
        """Forget the current request and wait for the next valid one."""
        with self._lock:
            self._parking_number = None

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

        with self._lock:
            self._parking_number = parking_number
        response.success = True
        response.message = (
            f'Food request accepted for parking {parking_number}.'
        )
        self._node.get_logger().info(response.message)
        return response
