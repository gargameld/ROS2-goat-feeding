"""Refresh the displayed scene at a fixed rate."""

from concurrent.futures import Future
import threading

from PyQt5.QtCore import QTimer

from simulation_interface_gui.gui import TopViewWindow
from simulation_interface_gui.presentation import SceneComposer
from simulation_interface_gui.presentation import SceneRenderer
from simulation_interface_gui.presentation import SceneUpdate
from simulation_interface_gui.presentation import TopViewSceneRenderer


class SceneRefresher:
    """Drive the presentation pipeline and hand finished scenes to the GUI."""

    def __init__(
        self,
        composer: SceneComposer,
        window: TopViewWindow,
        *,
        renderer: SceneRenderer | None = None,
        refresh_interval_ms: int = 500,
        timer: QTimer | None = None,
    ) -> None:
        """Create a stopped refresher and its periodic refresh timer."""
        if refresh_interval_ms <= 0:
            raise ValueError('The refresh interval must be positive.')
        self._composer = composer
        self._window = window
        self._renderer = renderer or TopViewSceneRenderer()
        self._request_lock = threading.Lock()
        self._request_in_flight = False
        self._running = False
        self._timer = timer or QTimer(window)
        self._timer.setInterval(refresh_interval_ms)
        self._timer.timeout.connect(self.request_scene_update)

    def start(self) -> None:
        """Start periodic scene refreshes and request the first scene now."""
        if self._running:
            return
        self._running = True
        self._timer.start()
        self.request_scene_update()

    def stop(self) -> None:
        """Stop periodic refreshes and suppress pending GUI updates."""
        self._running = False
        self._timer.stop()

    def request_scene_update(self) -> None:
        """Start one scene update unless another one is still pending."""
        if not self._running:
            return
        with self._request_lock:
            if self._request_in_flight:
                return
            self._request_in_flight = True

        try:
            future = self._composer.request_scene_update()
            future.add_done_callback(self._finish_scene_update)
        except Exception as error:
            with self._request_lock:
                self._request_in_flight = False
            self._window.set_status(
                f'Could not request simulation state: {error}',
                is_error=True,
            )

    def _finish_scene_update(self, future: Future[SceneUpdate]) -> None:
        try:
            update = future.result()
            qt_scene = self._renderer.render(update.scene_state)
            if self._running:
                simulation_state = update.simulation_state
                self._window.update_scene(qt_scene)
                self._window.set_obstacle_state(simulation_state.obstacle)
                self._window.set_poses(
                    simulation_state.amcl_pose,
                    simulation_state.odom_pose,
                    simulation_state.sim_pose,
                )
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
