"""Tests for behavior food-request storage."""

from behavior_interface.srv import RequestFood

from robot_behavior.request_listener import RequestListener


class FakeLogger:
    """Accept listener log calls during tests."""

    def info(self, _message):
        """Accept an informational message."""
        pass

    def warning(self, _message):
        """Accept a warning message."""
        pass


class FakeNode:
    """Capture the service callback created by the listener."""

    def __init__(self):
        """Create empty service registration state."""
        self.callback = None
        self.service_name = None
        self.logger = FakeLogger()

    def create_service(self, _service_type, service_name, callback):
        """Record and return a placeholder service."""
        self.service_name = service_name
        self.callback = callback
        return object()

    def get_logger(self):
        """Return the fake logger."""
        return self.logger


def _send(node, parking_number):
    request = RequestFood.Request()
    request.parking_number = parking_number
    return node.callback(request, RequestFood.Response())


def test_listener_waits_then_stores_and_resets_request():
    """The current parking is visible until reset starts a new wait."""
    node = FakeNode()
    listener = RequestListener(node)

    assert node.service_name == 'request_food'
    assert listener.get_parking_number() is None

    response = _send(node, 3)

    assert response.success is True
    assert listener.get_parking_number() == 3

    listener.reset()

    assert listener.get_parking_number() is None
    _send(node, 1)
    assert listener.get_parking_number() == 1


def test_listener_rejects_invalid_request_without_overwriting_value():
    """An out-of-range parking number leaves the last valid request intact."""
    node = FakeNode()
    listener = RequestListener(node)
    _send(node, 2)

    response = _send(node, 5)

    assert response.success is False
    assert listener.get_parking_number() == 2
