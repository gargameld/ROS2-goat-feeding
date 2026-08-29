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


def test_window_emits_throw_food_command():
    """Numeric and text controls produce the throw-food contract."""
    application = _application()
    window = TopViewWindow()
    commands = []
    window.throw_food_requested.connect(commands.append)
    window._food_name_edit.setText('box')
    window._parking_box.setValue(3)
    window._throw_boxes['throw_x'].setValue(0.25)
    window._throw_boxes['throw_y'].setValue(-0.1)
    window._throw_boxes['z'].setValue(0.5)

    window._send_throw_food()
    application.processEvents()

    assert commands[-1].food_name == 'box'
    assert commands[-1].parking_index == 3
    assert commands[-1].x == 0.25
    assert commands[-1].y == -0.1
    assert commands[-1].orientation.w == 1.0
    assert commands[-1].orientation.z == 0.5
    window.close()


def test_window_requires_food_name_before_throwing():
    """Throwing with an empty name reports an error instead of emitting."""
    application = _application()
    window = TopViewWindow()
    commands = []
    window.throw_food_requested.connect(commands.append)

    window._send_throw_food()
    application.processEvents()

    assert commands == []
    assert 'Enter a food object name' in window._status_label.text()
    window.close()


def test_window_emits_food_request_parking_number():
    """The request-food control emits the selected parking number."""
    application = _application()
    window = TopViewWindow()
    requests = []
    window.food_request_requested.connect(requests.append)
    window._request_parking_box.setValue(4)

    window._send_food_request()
    application.processEvents()

    assert requests == [4]
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


def test_window_emits_arrow_position_and_preserves_obstacle_dimensions():
    """Arrow controls move the managed box without changing its dimensions."""
    application = _application()
    window = TopViewWindow()
    commands = []
    window.obstacle_update_requested.connect(commands.append)
    window.set_obstacle_state(
        ObstacleState(Point3D(1.0, 2.0, 0.5), 0.8, 1.2, 1.0)
    )
    application.processEvents()

    window._move_obstacle(window._OBSTACLE_MOVE_STEP, 0.0)
    application.processEvents()

    assert commands[0].position.x == 1.5
    assert commands[0].position.z == 0.5
    assert (commands[0].width, commands[0].length, commands[0].height) == (
        0.8, 1.2, 1.0,
    )
    window.close()
