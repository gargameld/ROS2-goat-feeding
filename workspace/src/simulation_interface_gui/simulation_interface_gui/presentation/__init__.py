"""
Convert live simulation state into scenes the Qt widgets can draw.

The refresh pipeline runs bottom-up: a :class:`SimulationStateProvider` reads
the simulation into a :class:`SimulationState`, a :class:`SceneComposer` turns
that state into the lines and polygons of a :class:`SceneState`, and a
:class:`SceneRenderer` turns those into the :class:`QtScene` a canvas paints.
"""

from simulation_interface_gui.presentation.qt_renderer import SceneRenderer
from simulation_interface_gui.presentation.qt_renderer import TopViewSceneRenderer
from simulation_interface_gui.presentation.qt_scene import QtDrawable
from simulation_interface_gui.presentation.qt_scene import QtLinesItem
from simulation_interface_gui.presentation.qt_scene import QtMarkersItem
from simulation_interface_gui.presentation.qt_scene import QtPolygonItem
from simulation_interface_gui.presentation.qt_scene import QtScene
from simulation_interface_gui.presentation.scene_composer import SceneComposer
from simulation_interface_gui.presentation.scene_composer import SceneUpdate
from simulation_interface_gui.presentation.scene_composer import TopViewSceneComposer
from simulation_interface_gui.presentation.scene_state import Line2D
from simulation_interface_gui.presentation.scene_state import Point2D
from simulation_interface_gui.presentation.scene_state import Polygon2D
from simulation_interface_gui.presentation.scene_state import SceneState
from simulation_interface_gui.presentation.simulation_state import PoseEstimate
from simulation_interface_gui.presentation.simulation_state import SimulationState
from simulation_interface_gui.presentation.simulation_state_provider import (
    MujocoSimulationStateProvider,
)
from simulation_interface_gui.presentation.simulation_state_provider import (
    RobotStateDecoder,
)
from simulation_interface_gui.presentation.simulation_state_provider import (
    SimulationStateProvider,
)

__all__ = [
    'Line2D',
    'MujocoSimulationStateProvider',
    'Point2D',
    'Polygon2D',
    'PoseEstimate',
    'QtDrawable',
    'QtLinesItem',
    'QtMarkersItem',
    'QtPolygonItem',
    'QtScene',
    'RobotStateDecoder',
    'SceneComposer',
    'SceneRenderer',
    'SceneState',
    'SceneUpdate',
    'SimulationState',
    'SimulationStateProvider',
    'TopViewSceneComposer',
    'TopViewSceneRenderer',
]
