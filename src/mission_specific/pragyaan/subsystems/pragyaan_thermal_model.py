__author__ = "Louis Burtz, Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from dataclasses import dataclass

from src.subsystems.robot_physics_models.thermal_model import ThermalModel


@dataclass
class PragyaanThermalModel(ThermalModel):
    def __init__(self):
        super().__init__()
        self._node_temps["interior"] = self._initial_temp

    def compute(self, dt: float) -> None:
        super().compute(dt)
        self._node_temps["interior"] = sum(self._node_temps[face] for face in self._faces) / len(self._faces)
