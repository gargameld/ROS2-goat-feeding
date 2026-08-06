"""Own the ROS node, executor, and worker thread used by the GUI."""

from concurrent.futures import Future
import threading
from typing import Callable, TypeVar

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


Result = TypeVar('Result')


class RosRuntime:
    """
    Run one ROS node on a dedicated executor thread.

    GUI code can use :meth:`submit` to queue ROS work without touching the
    executor thread directly. The returned standard-library future is safe to
    observe from a GUI thread.
    """

    def __init__(
        self,
        node_name: str = 'simulation_interface_gui',
        *,
        ros_args: list[str] | None = None,
    ) -> None:
        """Create the ROS infrastructure and start its worker thread."""
        self._state_lock = threading.Lock()
        self._stopping = False
        self._stopped = False

        self._context = Context()
        rclpy.init(args=ros_args, context=self._context)

        try:
            self._node = rclpy.create_node(node_name, context=self._context)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
        except BaseException:
            self._context.shutdown()
            raise

        self._thread = threading.Thread(
            target=self._spin,
            name=f'{node_name}-ros-executor',
            daemon=True,
        )
        self._thread.start()

    @property
    def node(self) -> Node:
        """Return the runtime's node for work executed through ``submit``."""
        return self._node

    @property
    def is_running(self) -> bool:
        """Report whether the runtime accepts new work."""
        with self._state_lock:
            return not self._stopping and not self._stopped

    def submit(self, callback: Callable[[], Result]) -> Future[Result]:
        """
        Schedule ``callback`` on the executor thread.

        Raises
        ------
        RuntimeError
            If shutdown has started.

        """
        result: Future[Result] = Future()

        def execute() -> None:
            if not result.set_running_or_notify_cancel():
                return
            try:
                result.set_result(callback())
            except BaseException as error:
                result.set_exception(error)

        with self._state_lock:
            if self._stopping or self._stopped:
                raise RuntimeError('The ROS runtime is shutting down.')
            self._executor.create_task(execute)

        return result

    def shutdown(self, timeout_sec: float | None = 5.0) -> None:
        """Stop the executor and destroy all owned ROS resources safely."""
        if threading.current_thread() is self._thread:
            raise RuntimeError(
                'RosRuntime.shutdown() cannot run on its executor thread.'
            )

        with self._state_lock:
            if self._stopped:
                return
            if self._stopping:
                thread = self._thread
            else:
                self._stopping = True
                thread = self._thread
                self._executor.shutdown(timeout_sec=timeout_sec)

        thread.join(timeout=timeout_sec)
        if thread.is_alive():
            raise TimeoutError('Timed out while stopping the ROS executor thread.')

        with self._state_lock:
            if self._stopped:
                return
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            if self._context.ok():
                self._context.shutdown()
            self._stopped = True

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception as error:
            self._node.get_logger().error(
                f'ROS executor stopped unexpectedly: {error}'
            )

    def __enter__(self) -> 'RosRuntime':
        """Return this running runtime."""
        return self

    def __exit__(self, *_: object) -> None:
        """Shut down the runtime when leaving a context manager."""
        self.shutdown()
