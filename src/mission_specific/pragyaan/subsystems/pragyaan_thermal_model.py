__author__ = "Louis Burtz, Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from src.subsystems.robot_physics_models.thermal_model import ThermalModel
from dataclasses import dataclass

@dataclass
class PragyaanThermalModel(ThermalModel):

    def __init__(self):
        super().__init__()
        self._node_temps["interior"] = self._initial_temp


    def compute(self, dt: float) -> None:
        super().compute(dt)
        self._node_temps["interior"] = sum(self._node_temps[face] for face in self._faces) / len(self._faces)