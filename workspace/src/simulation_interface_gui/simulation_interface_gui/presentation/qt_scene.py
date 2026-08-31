"""
Qt-ready drawables produced for the top-view canvas.

Every shape stays in world metres: the canvas applies its own world-to-widget
transform before painting, and every pen is cosmetic so widths remain device
pixels at any zoom.
"""

from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass

from PyQt5.QtCore import QLineF
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPolygonF


class QtDrawable(ABC):
    """Paint one part of a scene with an already-configured Qt painter."""

    @abstractmethod
    def paint(self, painter: QPainter) -> None:
        """Draw this item into ``painter``."""


@dataclass(frozen=True)
class QtPolygonItem(QtDrawable):
    """Draw one filled and outlined polygon."""

    polygon: QPolygonF
    pen: QPen
    brush: QBrush

    def paint(self, painter: QPainter) -> None:
        """Draw the polygon with its own pen and brush."""
        painter.setPen(self.pen)
        painter.setBrush(self.brush)
        painter.drawPolygon(self.polygon)


@dataclass(frozen=True)
class QtLinesItem(QtDrawable):
    """Draw a group of line segments sharing one pen."""

    lines: tuple[QLineF, ...]
    pen: QPen

    def paint(self, painter: QPainter) -> None:
        """Draw every segment with the shared pen and no brush."""
        painter.setPen(self.pen)
        painter.setBrush(QBrush())
        for line in self.lines:
            painter.drawLine(line)


@dataclass(frozen=True)
class QtMarkersItem(QtDrawable):
    """Draw round markers sized by a wide, round-capped cosmetic pen."""

    points: tuple[QPointF, ...]
    pen: QPen

    def paint(self, painter: QPainter) -> None:
        """Draw one dot per point."""
        painter.setPen(self.pen)
        for point in self.points:
            painter.drawPoint(point)


@dataclass(frozen=True, slots=True)
class QtScene:
    """Contain every drawable of one scene, in back-to-front paint order."""

    drawables: tuple[QtDrawable, ...]

    def paint(self, painter: QPainter) -> None:
        """Draw the whole scene in order."""
        for drawable in self.drawables:
            drawable.paint(painter)
