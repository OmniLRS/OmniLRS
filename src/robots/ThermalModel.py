"""Basic thermal model for a six-faced box with a single interior node."""

from __future__ import annotations

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


@dataclass
class ThermalModel:
	"""Tiny proof-of-concept thermal model.

	The model exposes six exterior faces plus one interior node. Each exterior
	node temperature follows a first-order response toward a sigmoid-derived
	target temperature that depends on the provided view factor (0 - 1). The
	interior node temperature is recomputed as the simple average of the six
	exterior nodes at every step.
 
    To integrate this model in a simulation, see the example in run_single_sun_test() below.
        inputs / computation / outputs are clearly separated for easy use.
	"""

	min_temp: float = -50.0
	max_temp: float = 100.0
	time_constant: float = 600.0  # Seconds to reach ~63% of target delta.
	sigmoid_gain: float = 8.0  # Controls steepness of the sun-loading curve.
	faces: Iterable[str] = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
	node_temps: Dict[str, float] = field(default_factory=dict)
	initial_temp: float = 20.0
	rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
	rover_yaw_deg: float = 0.0
	sun_position: Tuple[float, float, float] = (1.0, 0.0, 0.0)
	measurement_noise_std: float = 0.5

	def __post_init__(self) -> None:
		self.faces = tuple(self.faces)
		if not self.node_temps:
			self.node_temps = {face: self.initial_temp for face in self.faces}
			self.node_temps["interior"] = self.initial_temp
		else:
			for face in self.faces:
				self.node_temps.setdefault(face, self.initial_temp)
			self.node_temps.setdefault("interior", self.initial_temp)

	def step(self, dt: float) -> None:
		"""Advance the model by *dt* seconds using stored rover/sun positions."""

		view_factors = self.compute_view_factors(self.rover_position, self.sun_position)

		for face in self.faces:
			exposure = _clamp(view_factors.get(face, 0.0), 0.0, 1.0)
			target = self._target_temperature(exposure)
			current = self.node_temps[face]
			delta = (target - current) * (dt / self.time_constant)
			self.node_temps[face] = current + delta

		self.node_temps["interior"] = sum(self.node_temps[face] for face in self.faces) / len(self.faces)

	def set_rover_yaw(self, yaw_deg: float) -> None:
		self.rover_yaw_deg = yaw_deg

	def set_rover_position(self, position: Tuple[float, float, float]) -> None:
		self.rover_position = position

	def set_sun_position(self, position: Tuple[float, float, float]) -> None:
		self.sun_position = position

	def _target_temperature(self, view_factor: float) -> float:
		"""Return the sigmoid-based target temperature for a view factor."""

		midpoint = 0.5
		gain = self.sigmoid_gain
		sigmoid = 1.0 / (1.0 + math.exp(-gain * (view_factor - midpoint)))
		return self.min_temp + (self.max_temp - self.min_temp) * sigmoid


	def compute_view_factors(
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
			return {face: 0.0 for face in self.faces}

		unit = vector / magnitude
		yaw_rad = math.radians(self.rover_yaw_deg)
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
			for face in self.faces
		}

	def temperatures(self) -> Dict[str, float]:
		"""Return noisy temperature readings without altering the model state."""

		if self.measurement_noise_std <= 0:
			return dict(self.node_temps)

		return {
			name: value + random.gauss(0.0, self.measurement_noise_std)
			for name, value in self.node_temps.items()
		}

def run_single_sun_test(total_time: float = 600.0, dt: float = 1.0) -> tuple[Sequence[float], Dict[str, Sequence[float]]]:
	"""Simulate +X sun exposure and return timelines for plotting/tests."""

	steps = int(total_time / dt)
	model = ThermalModel()
	times = [0.0]
	initial_snapshot = model.temperatures()
	temps = {name: [value] for name, value in initial_snapshot.items()}

	rover_pos = (0.0, 0.0, 0.0)
	ROVER_YAW_DEG = -58.0 
 
	SUN_DISTANCE = 1000. # m
	SUN_AZYMUTH_DEG = 65.0
	SUN_POSITION = (
		- SUN_DISTANCE * np.sin(np.pi * SUN_AZYMUTH_DEG / 180.0), 
  		SUN_DISTANCE * np.cos(np.pi * SUN_AZYMUTH_DEG / 180.0), 
    	10.0
	)
	print(SUN_POSITION)

	for idx in range(steps):
     
     
        # set inputs
		model.set_rover_position(rover_pos)
		model.set_rover_yaw(ROVER_YAW_DEG)
		model.set_sun_position(SUN_POSITION)

        # perform computation
		model.step(dt)
        # get results
		t = model.temperatures()
  
        # for the plots
		times.append((idx + 1) * dt)
		for name, value in t.items():
			temps[name].append(value)

	return times, temps


def _plot_temperature_profile(times: Sequence[float], temps: Mapping[str, Sequence[float]], filename: str = "thermal_model_demo.png") -> str:
	"""Plot node temperatures versus time; returns output filename."""

	from matplotlib import pyplot as plt

	plt.figure(figsize=(10, 6))
	for name, series in temps.items():
		plt.plot(times, series, label=name)

	plt.xlabel("Time [s]")
	plt.ylabel("Temperature [°C]")
	plt.title("Thermal Model Response")
	plt.legend()
	plt.grid(True, alpha=0.3)
	plt.tight_layout()
	plt.savefig(filename)
	plt.close()
	return filename


def main() -> None:
	times, temps = run_single_sun_test(total_time=600.0, dt=1.0)
	output = _plot_temperature_profile(times, temps)
	print(f"Thermal model plot saved to {output}")


if __name__ == "__main__":
	main()
