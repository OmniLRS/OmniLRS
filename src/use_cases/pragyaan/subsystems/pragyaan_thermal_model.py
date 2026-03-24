from __future__ import annotations

from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel
from src.subsystems.robot_physics_models.thermal_model import ThermalModel
"""
The above import MUST be at the top of the file, can not be preceded by anything or it crashes
SyntaxError: from __future__ imports must occur at the beginning of the file
"""

__author__ = "Louis Burtz"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

"""Basic thermal model for a six-faced box with a single interior node."""

from dataclasses import dataclass

@dataclass
class PragyaanThermalModel(ThermalModel):
    """Tiny proof-of-concept thermal model.

    The model exposes six exterior faces plus one interior node. Each exterior
    node temperature follows a first-order response toward a sigmoid-derived
    target temperature that depends on the provided view factor (0 - 1). The
    interior node temperature is recomputed as the simple average of the six
    exterior nodes at every step.

    To integrate this model in a simulation, see the example in run_single_sun_test() below.
        inputs / computation / outputs are clearly separated for easy use.
    """

    def __init__(self):
        super().__init__()
        self._node_temps["interior"] = self._initial_temp


    def compute(self, dt: float) -> None:
        super().compute(dt)
        self._node_temps["interior"] = sum(self._node_temps[face] for face in self._faces) / len(self._faces)