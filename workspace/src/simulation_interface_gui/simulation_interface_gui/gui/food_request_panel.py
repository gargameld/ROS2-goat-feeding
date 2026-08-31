"""Ask the behavior node to collect food from one parking area."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QWidget


class FoodRequestPanel(QGroupBox):
    """Emit the parking number the robot should collect food from."""

    food_requested = pyqtSignal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create the parking selector and its request button."""
        super().__init__('Request food', parent)
        layout = QFormLayout(self)
        self._parking_box = QSpinBox(self)
        self._parking_box.setRange(1, 4)
        layout.addRow('Parking number', self._parking_box)
        request_button = QPushButton('Send food request', self)
        request_button.clicked.connect(self.send_request)
        layout.addRow(request_button)

    def send_request(self) -> None:
        """Emit the selected parking number."""
        self.food_requested.emit(self._parking_box.value())
