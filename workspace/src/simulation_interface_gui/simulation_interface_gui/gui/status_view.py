"""Show the latest status or error message of the interface."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QWidget


class StatusView(QLabel):
    """Display one status line, updatable from any thread."""

    _ERROR_COLOR = '#a12622'
    _NORMAL_COLOR = '#276738'

    _status_received = pyqtSignal(str, bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create the status label in its waiting state."""
        super().__init__('Waiting for simulation state…', parent)
        self.setWordWrap(True)
        self.setMinimumWidth(210)
        # A long ROS error must wrap inside the panel, never widen it, and
        # the label must be given the height its wrapped text needs.
        self.setSizePolicy(_wrapping_policy())
        self._status_received.connect(self._apply_status, Qt.QueuedConnection)

    def set_status(self, message: str, *, is_error: bool = False) -> None:
        """Queue a status message from any thread."""
        self._status_received.emit(message, is_error)

    def hasHeightForWidth(self) -> bool:
        """Report that the wrapped text height depends on the given width."""
        return True

    @pyqtSlot(str, bool)
    def _apply_status(self, message: str, is_error: bool) -> None:
        color = self._ERROR_COLOR if is_error else self._NORMAL_COLOR
        self.setStyleSheet(f'color: {color};')
        self.setToolTip(message)
        self.setText(message)


def _wrapping_policy() -> QSizePolicy:
    """Return a policy that ignores text width but honours wrapped height."""
    policy = QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    policy.setHeightForWidth(True)
    return policy
