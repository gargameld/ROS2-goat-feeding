"""Headless tests for the lightweight Qt top-view widgets."""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtGui import QImage  # noqa: E402
from PyQt5.QtGui import QPainter  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from simulation_interface_gui.gui import TopViewWindow  # noqa: E402
from simulation_interface_gui.models import ObstacleState  # noqa: E402
from simulation_interface_gui.models import Point3D  # noqa: E402
from simulation_interface_gui.models import Pose2D  # noqa: E402
from simulation_interface_gui.models import Quaternion  # noqa: E402
from simulation_interface_gui.models import SimulationSnapshot  # noqa: E402
from simulation_interface_gui.presentation import SceneBuilder  # noqa: E402


def _application():
    return QApplication.instance() or QApplication([])


def test_window_emits_velocity_values():
    """Numeric controls produce the manager's velocity-command contract."""
    application = _application()
    window = TopViewWindow()
    commands = []
    window.velocity_command_requested.connect(commands.append)
    window._spin_boxes['linear_x'].setValue(1.25)
    window._spin_boxes['angular_z'].setValue(-0.5)

    window._send_velocity()
    application.processEvents()

    assert commands[-1].linear_x == 1.25
    assert commands[-1].angular_z == -0.5
    window.close()


def test_canvas_renders_complete_scene_offscreen():
    """A built scene can be painted into an image without a display server."""
    application = _application()
    window = TopViewWindow()
    scene = SceneBuilder().build(SimulationSnapshot(
        base_position=Point3D(0.0, 0.0, 0.26),
        base_orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
        arm_joint_positions=(0.0,) * 6,
    ))
    window.canvas.resize(500, 650)
    window.update_scene(scene)
    application.processEvents()
    image = QImage(500, 650, QImage.Format_ARGB32)
    painter = QPainter(image)

    window.canvas.render(painter)
    painter.end()

    assert not image.isNull()
    window.close()


def test_window_displays_amcl_and_simulation_poses():
    """Both pose sources are rendered in the control panel."""
    application = _application()
    window = TopViewWindow()

    window.set_poses(
        Pose2D(1.0, 2.0, 0.5),
        Pose2D(2.0, 3.0, 0.25),
        Pose2D(3.0, 4.0, -0.25),
    )
    application.processEvents()

    assert 'x=1.00, y=2.00, yaw=0.50 rad' == window._amcl_pose_label.text()
    assert 'x=2.00, y=3.00, yaw=0.25 rad' == window._odom_pose_label.text()
    assert 'x=3.00, y=4.00, yaw=-0.25 rad' == window._sim_pose_label.text()
    window.close()


def test_window_emits_obstacle_dimensions_and_arrow_position():
    """Dimension and arrow controls emit complete managed-box state."""
    application = _application()
    window = TopViewWindow()
    commands = []
    window.obstacle_update_requested.connect(commands.append)
    window.set_obstacle_state(
        ObstacleState(Point3D(1.0, 2.0, 0.5), 0.8, 1.2, 1.0)
    )
    application.processEvents()
    window._dimension_boxes['height'].setValue(1.5)

    window._apply_obstacle_dimensions()
    window._move_obstacle(window._OBSTACLE_MOVE_STEP, 0.0)
    application.processEvents()

    assert commands[0].height == 1.5
    assert commands[1].position.x == 1.5
    assert commands[1].position.z == 0.75
    window.close()
