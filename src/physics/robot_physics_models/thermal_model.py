from __future__ import annotations

from src.physics.robot_physics_models.robot_physics_model import RobotPhysicsModel
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

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Sequence, Tuple

import numpy as np
import random


def _clamp(value: float, lower: float, upper: float) -> float:
	return max(lower, min(upper, value))


FACE_NORMALS: Dict[str, np.ndarray] = {
	"+X": np.array((1.0, 0.0, 0.0)),
	"-X": np.array((-1.0, 0.0, 0.0)),
	"+Y": np.array((0.0, 1.0, 0.0)),
	"-Y": np.array((0.0, -1.0, 0.0)),
	"+Z": np.array((0.0, 0.0, 1.0)),
	"-Z": np.array((0.0, 0.0, -1.0)),
}

MIN_TEMP: float = -50.0
MAX_TEMP: float = 100.0
TIME_CONSTANT: float = 600.0  # Seconds to reach ~63% of target delta.
SIGMOID_GAIN: float = 8.0  # Controls steepness of the sun-loading curve.
FACES: Iterable[str] = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
INITIAL_TEMP: float = 20.0
MEASUREMENT_NOISE: float = 0.5

@dataclass
class ThermalModel(RobotPhysicsModel):
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
        self._rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._rover_yaw_deg: float = 0.0
        self._sun_position: Tuple[float, float, float] = (1.0, 0.0, 0.0)
        self._faces: Iterable[str] = FACES
        self._node_temps: Dict[str, float] = {}
        #NOTE the following fileds were set to default values, as these values depend on the environment (Moon), not so much on the rover
        # no setters were implemented, but the values may be directly accessed from a subsystems manager if need for customization exists
        self._min_temp: float = MIN_TEMP
        self._max_temp: float = MAX_TEMP
        self._initial_temp: float = INITIAL_TEMP
        self._time_constant: float = TIME_CONSTANT  # Seconds to reach ~63% of target delta.
        self._sigmoid_gain: float = SIGMOID_GAIN  # Controls steepness of the sun-loading curve.
        self._measurement_noise_std: float = MEASUREMENT_NOISE
        self.setup()

    def setup(self):
        self._faces = tuple(self._faces)
        if not self._node_temps:
            self._node_temps = {face: self._initial_temp for face in self._faces}
        else:
            for face in self._faces:
                self._node_temps.setdefault(face, self._initial_temp)

    def update_inputs(self, rover_position, sun_position, rover_yaw_deg):  
        self._rover_position = rover_position
        self._sun_position  = sun_position
        self._rover_yaw_deg = rover_yaw_deg

    def step(self, dt: float) -> None:
        # override in your custom model if needed
        # may call super().step() if you wish to reuse the face logic
        """Advance the model by *dt* seconds using stored rover/sun positions."""

        view_factors = self._compute_view_factors(self._rover_position, self._sun_position)

        for face in self._faces:
            exposure = _clamp(view_factors.get(face, 0.0), 0.0, 1.0)
            target = self._target_temperature(exposure)
            current = self._node_temps[face]
            delta = (target - current) * (dt / self._time_constant)
            self._node_temps[face] = current + delta

    def get_output(self):
        return self.temperatures()

    def temperatures(self) -> Dict[str, float]:
        """Return noisy temperature readings without altering the model state."""

        if self._measurement_noise_std <= 0:
            return dict(self._node_temps)

        return {
            name: value + random.gauss(0.0, self._measurement_noise_std)
            for name, value in self._node_temps.items()
        }

    def set_rover_yaw(self, yaw_deg: float) -> None:
        self._rover_yaw_deg = yaw_deg

    def set_rover_position(self, position: Tuple[float, float, float]) -> None:
        self._rover_position = position

    def set_sun_position(self, position: Tuple[float, float, float]) -> None:
        self._sun_position = position

    def _target_temperature(self, view_factor: float) -> float:
        """Return the sigmoid-based target temperature for a view factor."""
        midpoint = 0.5
        gain = self._sigmoid_gain
        sigmoid = 1.0 / (1.0 + math.exp(-gain * (view_factor - midpoint)))

        return self._min_temp + (self._max_temp - self._min_temp) * sigmoid

    def _compute_view_factors(
        self,
        rover_position: Tuple[float, float, float],
        sun_position: Tuple[float, float, float],
    ) -> Dict[str, float]:
        """Compute cosine-based per-face view factors given rover and sun positions."""

        rover = np.asarray(rover_position, dtype=float)
        sun = np.asarray(sun_position, dtype=float)
        vector = sun - rover
        magnitude = np.linalg.norm(vector)
        if magnitude == 0.0:
            return {face: 0.0 for face in self._faces}

        unit = vector / magnitude
        yaw_rad = math.radians(self._rover_yaw_deg)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        rotation = np.array(
            [
                [cos_yaw, -sin_yaw, 0.0],
                [sin_yaw, cos_yaw, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return {
            face: float(
                _clamp(float(np.dot(unit, rotation @ FACE_NORMALS[face])), 0.0, 1.0)
            )
            for face in self._faces
        }