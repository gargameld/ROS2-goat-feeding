"""Collect and validate one food-throwing command from the operator."""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QWidget

from simulation_interface_gui.models import Quaternion
from simulation_interface_gui.models import ThrowFoodCommand


class ThrowFoodPanel(QGroupBox):
    """Emit throw-food commands built from validated form input."""

    throw_requested = pyqtSignal(object)
    error_reported = pyqtSignal(str)

    _FIELDS = (
        ('Quaternion W', 'w'),
        ('Quaternion X', 'x'),
        ('Quaternion Y', 'y'),
        ('Quaternion Z', 'z'),
        ('Throw X', 'throw_x'),
        ('Throw Y', 'throw_y'),
    )

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create the name, parking, pose, and orientation controls."""
        super().__init__('Throw food', parent)
        self._boxes: dict[str, QDoubleSpinBox] = {}
        layout = QFormLayout(self)

        self._food_name_edit = QLineEdit(self)
        self._food_name_edit.setPlaceholderText('food object name (no prefix)')
        layout.addRow('Food name', self._food_name_edit)

        self._parking_box = QSpinBox(self)
        self._parking_box.setRange(1, 4)
        layout.addRow('Parking number', self._parking_box)

        for label, field_name in self._FIELDS:
            spin_box = QDoubleSpinBox(self)
            spin_box.setRange(-100.0, 100.0)
            spin_box.setDecimals(3)
            spin_box.setSingleStep(0.1)
            if field_name.startswith('throw'):
                spin_box.setSuffix(' m')
            elif field_name == 'w':
                spin_box.setValue(1.0)
            self._boxes[field_name] = spin_box
            layout.addRow(label, spin_box)

        throw_button = QPushButton('Throw food', self)
        throw_button.clicked.connect(self.send_command)
        layout.addRow(throw_button)

    def send_command(self) -> None:
        """Emit the current form contents, or report why they are invalid."""
        food_name = self._food_name_edit.text().strip()
        if not food_name:
            self.error_reported.emit('Enter a food object name to throw.')
            return
        self.throw_requested.emit(ThrowFoodCommand(
            food_name=food_name,
            parking_index=self._parking_box.value(),
            x=self._boxes['throw_x'].value(),
            y=self._boxes['throw_y'].value(),
            orientation=Quaternion(
                w=self._boxes['w'].value(),
                x=self._boxes['x'].value(),
                y=self._boxes['y'].value(),
                z=self._boxes['z'].value(),
            ),
        ))
