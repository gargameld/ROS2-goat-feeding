"""Show the localisation and simulation poses of the robot."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.presentation import PoseEstimate


# The widest pose text the panel formats, used to keep each value on one line.
_WIDEST_POSE_TEXT = 'x=-00.00, y=-00.00, yaw=-0.00 rad'


class PosePanel(QGroupBox):
    """Display one label per pose source, updatable from any thread."""

    _poses_received = pyqtSignal(object, object, object)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create one waiting label per pose source."""
        super().__init__('Robot pose', parent)
        self._amcl_label = _pose_label('Waiting for map to base_link...')
        self._odom_label = _pose_label('Waiting for odom to base_link...')
        self._sim_label = _pose_label('Waiting for MuJoCo state...')
        layout = QFormLayout(self)
        layout.addRow('AMCL map to base_link', self._amcl_label)
        layout.addRow('EKF odom to base_link', self._odom_label)
        layout.addRow('MuJoCo simulation', self._sim_label)
        self._poses_received.connect(self._apply_poses, Qt.QueuedConnection)

    def set_poses(
        self,
        amcl_pose: PoseEstimate,
        odom_pose: PoseEstimate,
        sim_pose: PoseEstimate,
    ) -> None:
        """Queue the AMCL, odometry, and MuJoCo pose estimates from any thread."""
        self._poses_received.emit(amcl_pose, odom_pose, sim_pose)

    @pyqtSlot(object, object, object)
    def _apply_poses(
        self,
        amcl_pose: PoseEstimate,
        odom_pose: PoseEstimate,
        sim_pose: PoseEstimate,
    ) -> None:
        self._show_pose(self._amcl_label, amcl_pose)
        self._show_pose(self._odom_label, odom_pose)
        self._show_pose(self._sim_label, sim_pose)

    @staticmethod
    def _show_pose(label: QLabel, estimate: PoseEstimate) -> None:
        pose = estimate.pose
        if pose is None:
            # Transform errors are long, so only the tooltip carries them.
            label.setText('unavailable')
            label.setToolTip(estimate.error or 'no pose received')
            return
        label.setText(f'x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f} rad')
        label.setToolTip('')


def _pose_label(placeholder: str) -> QLabel:
    """Return a label whose text never widens the panel that holds it."""
    label = QLabel(placeholder)
    label.setWordWrap(True)
    label.setMinimumWidth(
        label.fontMetrics().horizontalAdvance(_WIDEST_POSE_TEXT)
    )
    policy = QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    policy.setHeightForWidth(True)
    label.setSizePolicy(policy)
    return label
