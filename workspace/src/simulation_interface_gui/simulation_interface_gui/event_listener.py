"""Listen to GUI events and forward them to ROS operations."""

from concurrent.futures import Future

from simulation_interface_gui.gui import TopViewWindow
from simulation_interface_gui.models import ObstacleState
from simulation_interface_gui.models import ThrowFoodCommand
from simulation_interface_gui.ros import MujocoClient


class EventListener:
    """Forward GUI food-throwing commands and report their completion."""

    def __init__(self, client: MujocoClient, window: TopViewWindow) -> None:
        """Connect GUI command signals without starting event processing."""
        self._client = client
        self._window = window
        self._running = False
        self._window.throw_food_requested.connect(
            self.handle_throw_food
        )
        self._window.obstacle_update_requested.connect(
            self.handle_obstacle_update
        )

    def start(self) -> None:
        """Allow command results to update the GUI."""
        self._running = True

    def stop(self) -> None:
        """Suppress status updates from pending command results."""
        self._running = False

    def handle_throw_food(self, command: ThrowFoodCommand) -> None:
        """Forward one food-throwing command from the GUI to the ROS client."""
        try:
            future = self._client.throw_food(command)
            future.add_done_callback(self._finish_throw_food)
        except Exception as error:
            self._window.set_status(
                f'Could not throw food: {error}',
                is_error=True,
            )

    def _finish_throw_food(self, future: Future[None]) -> None:
        try:
            future.result()
            if self._running:
                self._window.set_status('Food thrown.')
        except Exception as error:
            if self._running:
                self._window.set_status(
                    f'Could not throw food: {error}',
                    is_error=True,
                )

    def handle_obstacle_update(self, obstacle: ObstacleState) -> None:
        """Forward one box edit from the GUI to the simulation plugin."""
        try:
            future = self._client.set_obstacle(obstacle)
            future.add_done_callback(self._finish_obstacle_update)
        except Exception as error:
            self._window.set_status(
                f'Could not update obstacle: {error}', is_error=True
            )

    def _finish_obstacle_update(self, future: Future[None]) -> None:
        try:
            future.result()
            if self._running:
                self._window.set_status('Obstacle updated.')
        except Exception as error:
            if self._running:
                self._window.set_status(
                    f'Could not update obstacle: {error}', is_error=True
                )
