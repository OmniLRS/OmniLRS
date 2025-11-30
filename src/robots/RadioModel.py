"""Basic RSSI model relating rover/lander separation to signal strength."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple


@dataclass
class RadioModel:
	"""Proof-of-concept radio model with quadratic distance falloff.
        To integrate this model in a simulation, see the example in sweep_rssi() below.
        inputs / computation / outputs are clearly separated for easy use.
    """

	lander_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
	rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
	best_rssi: float = -100.0  # Strongest signal observed at zero separation.
	worst_rssi: float = -30.0  # Weakest accepted reading at reference distance.
	reference_distance: float = 100.0  # Distance (m) at which worst_rssi applies.
	noise_std: float = 1.0  # Standard deviation of random RSSI noise [dB].

	def distance(self) -> float:
		dx = self.rover_position[0] - self.lander_position[0]
		dy = self.rover_position[1] - self.lander_position[1]
		dz = self.rover_position[2] - self.lander_position[2]
		return math.sqrt(dx * dx + dy * dy + dz * dz)

	def rssi(self) -> float:
		dist = self.distance()
		norm = dist / self.reference_distance
		mean_rssi = self.best_rssi + (self.worst_rssi - self.best_rssi) * (norm ** 2)
		return mean_rssi + random.gauss(0.0, self.noise_std)


def sweep_rssi(
	steps: int = 200,
	rover_end: Tuple[float, float, float] = (50.0, 100.0, 0.0),
	lander_position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Tuple[Sequence[float], Sequence[float]]:
	"""Walk the rover linearly from origin to *rover_end* and track RSSI."""

	model = RadioModel(lander_position=lander_position)
	distances: List[float] = []
	rssi_values: List[float] = []

	for step in range(steps + 1):
		alpha = step / steps
		x = rover_end[0] * alpha
		y = rover_end[1] * alpha
		z = rover_end[2] * alpha
		model.rover_position = (x, y, z)
		distances.append(model.distance())
		rssi_values.append(model.rssi())

	return distances, rssi_values


def plot_rssi_profile(distances: Sequence[float], rssi: Sequence[float], filename: str = "radio_model_rssi.png") -> str:
	"""Plot RSSI vs. distance."""

	from matplotlib import pyplot as plt  # type: ignore[import]

	plt.figure(figsize=(8, 5))
	plt.plot(distances, rssi, label="RSSI")
	plt.xlabel("Distance [m]")
	plt.ylabel("RSSI [dBm]")
	plt.title("Radio Model RSSI vs. Distance")
	plt.grid(True, alpha=0.3)
	plt.legend()
	plt.tight_layout()
	plt.savefig(filename)
	plt.close()
	return filename


def main() -> None:
	distances, rssi_values = sweep_rssi()
	output = plot_rssi_profile(distances, rssi_values)
	print(f"Radio model plot saved to {output}")


if __name__ == "__main__":
	main()
