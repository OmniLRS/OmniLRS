"""Simple power model with battery, solar charging, and device loads."""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, Mapping, Sequence, Tuple

import numpy as np


DEVICE_CURRENT_BOUNDS: Dict[str, Tuple[float, float]] = {
	"current_draw_obc": (0.0, 2.0),
	"current_draw_motor_controller": (0.0, 1.0),
	"current_draw_neutron_spectrometer": (0.0, 1.5),
	"current_draw_apxs": (0.0, 1.0),
	"current_draw_camera": (0.0, 1.5),
	"current_draw_radio": (0.0, 3.0),
	"current_draw_eps": (0.0, 0.8),
}


BATTERY_VOLTAGE_CURVE: Tuple[Tuple[float, float], ...] = (
	(0.0, 11.0),
	(0.1, 12.0),
	(0.2, 13.5),
	(0.4, 15.0),
	(0.6, 15.8),
	(0.8, 16.3),
	(0.9, 16.6),
	(1.0, 16.8),
)

SOLAR_PANEL_NORMAL = np.array((0.0, -1.0, 0.0))


def _clamp(value: float, lower: float, upper: float) -> float:
	return max(lower, min(upper, value))


@dataclass
class PowerModel:
	"""Track battery charge based on solar input and device consumption.
        To integrate this model in a simulation, see the example in run_power_profile_test() below.
        inputs / computation / outputs are clearly separated for easy use.
    """

	battery_capacity_ah: float = 2.0
	battery_charge_ah: float = 2.0
	solar_panel_max_current: float = 2.0
	device_current_bounds: Mapping[str, Tuple[float, float]] = field(
		default_factory=lambda: dict(DEVICE_CURRENT_BOUNDS)
	)
	device_states: Dict[str, bool] = field(default_factory=dict)
	device_current_noise_std: float = 0.05
	battery_percentage_noise_std: float = 0.5
	battery_voltage_noise_std: float = 0.05
	solar_input_noise_std: float = 0.05
	rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
	sun_position: Tuple[float, float, float] = (0.0, -10.0, 0.0)
	solar_input_current: float = 0.0

	def __post_init__(self) -> None:
		for name in self.device_current_bounds:
			self.device_states.setdefault(name, False)
		self.battery_charge_ah = _clamp(self.battery_charge_ah, 0.0, self.battery_capacity_ah)

	def set_device_state(self, name: str, state: bool) -> None:
		if name not in self.device_current_bounds:
			raise KeyError(f"Unknown device '{name}'")
		self.device_states[name] = state

	def set_device_states(self, states: Mapping[str, bool]) -> None:
		for name, state in states.items():
			self.set_device_state(name, state)

	def set_rover_position(self, position: Tuple[float, float, float]) -> None:
		self.rover_position = position

	def set_sun_position(self, position: Tuple[float, float, float]) -> None:
		self.sun_position = position

	def set_all_devices(self, state: bool) -> None:
		for name in self.device_states:
			self.device_states[name] = state

	def _device_current(self, name: str) -> float:
		lo, hi = self.device_current_bounds[name]
		return hi if self.device_states[name] else lo

	def total_load_current(self) -> float:
		return sum(self._device_current(name) for name in self.device_states)

	def step(self, dt: float) -> None:
		"""Advance the battery state by *dt* seconds."""

		view_factor = self.compute_view_factor(self.rover_position, self.sun_position)
		self.solar_input_current = self.solar_panel_max_current * view_factor
		net_current = self.solar_input_current - self.total_load_current()
		self.battery_charge_ah += net_current * (dt / 3600.0)
		self.battery_charge_ah = _clamp(self.battery_charge_ah, 0.0, self.battery_capacity_ah)

	def compute_view_factor(
		self,
		rover_position: Tuple[float, float, float],
		sun_position: Tuple[float, float, float],
	) -> float:
		rover = np.asarray(rover_position, dtype=float)
		sun = np.asarray(sun_position, dtype=float)
		vector = sun - rover
		magnitude = np.linalg.norm(vector)
		if magnitude == 0.0:
			return 0.0
		unit = vector / magnitude
		return float(_clamp(float(np.dot(unit, SOLAR_PANEL_NORMAL)), 0.0, 1.0))

	def battery_percentage(self) -> float:
		return 100.0 * self.battery_charge_ah / self.battery_capacity_ah

	def battery_voltage(self) -> float:
		fraction = _clamp(self.battery_percentage() / 100.0, 0.0, 1.0)
		curve = BATTERY_VOLTAGE_CURVE
		for idx in range(1, len(curve)):
			p_hi, v_hi = curve[idx]
			if fraction <= p_hi:
				p_lo, v_lo = curve[idx - 1]
				if p_hi == p_lo:
					return v_hi
				slope = (v_hi - v_lo) / (p_hi - p_lo)
				return v_lo + slope * (fraction - p_lo)
		return curve[-1][1]

	def measured_battery_percentage(self) -> float:
		value = self.battery_percentage()
		if self.battery_percentage_noise_std <= 0.0:
			return value
		return _clamp(value + random.gauss(0.0, self.battery_percentage_noise_std), 0.0, 100.0)

	def measured_battery_voltage(self) -> float:
		value = self.battery_voltage()
		if self.battery_voltage_noise_std <= 0.0:
			return value
		min_v = BATTERY_VOLTAGE_CURVE[0][1]
		max_v = BATTERY_VOLTAGE_CURVE[-1][1]
		return _clamp(value + random.gauss(0.0, self.battery_voltage_noise_std), min_v, max_v)

	def measured_device_currents(self) -> Dict[str, float]:
		"""Return measured device currents with additive noise."""

		currents = {name: self._device_current(name) for name in self.device_states}
		if self.device_current_noise_std <= 0.0:
			return currents
		return {
			name: max(0.0, value + random.gauss(0.0, self.device_current_noise_std))
			for name, value in currents.items()
		}

	def measured_solar_input_current(self) -> float:
		value = max(0.0, self.solar_input_current)
		if self.solar_input_noise_std <= 0.0:
			return value
		return _clamp(
			value + random.gauss(0.0, self.solar_input_noise_std),
			0.0,
			self.solar_panel_max_current,
		)
  
	def status(self) -> Dict[str, float | Dict[str, float]]:
		status: Dict[str, float | Dict[str, float]] = {
			"battery_charge_ah": self.battery_charge_ah,
			"battery_percentage": self.battery_percentage(),
			"battery_voltage": self.battery_voltage(),
			"net_current": self.solar_input_current - self.total_load_current(),
			"solar_input_current_measured": self.measured_solar_input_current(),
			"battery_percentage_measured": self.measured_battery_percentage(),
			"battery_voltage_measured": self.measured_battery_voltage(),
		}
		status["device_currents_measured"] = self.measured_device_currents()
		return status


def run_power_profile_test(
		on_duration: float = 800.0,
		off_duration: float = 800.0,
		dt: float = 1.0,
	) -> Tuple[Sequence[float], Sequence[float], Sequence[float], Sequence[float]]:
	model = PowerModel()
	times = [0.0]
	status = model.status()
	percentages = [status["battery_percentage_measured"]]
	voltages = [status["battery_voltage_measured"]]
	solar_currents = [status["solar_input_current_measured"]]
	model.set_rover_position((0.0, 0.0, 0.0))

	def simulate(
			duration: float,
			devices_on: bool,
			sun_position: Tuple[float, float, float],
			rover_delta: Tuple[float, float, float],
		) -> None:
     
		steps = int(duration / dt)
		active_states = {name: devices_on for name in model.device_states}
		rover_pos = np.asarray(model.rover_position, dtype=float)

		for _ in range(steps):
			# set inputs
			model.set_device_states(active_states)
			model.set_sun_position(sun_position)
			model.set_rover_position(tuple(rover_pos))

			# perform computation
			model.step(dt)

			# get results
			status = model.status()
   
            # for the plots
			times.append(times[-1] + dt)
			percentages.append(status["battery_percentage_measured"])
			voltages.append(status["battery_voltage_measured"])
			solar_currents.append(status["solar_input_current_measured"])
			rover_pos = rover_pos + np.asarray(rover_delta, dtype=float)

    # step 1 all devices on, sun above
	simulate(on_duration, True, (0.0, -10.0, 0.0), (0.02, 0.0, 0.0))
    # step 2 all devices off, sun above
	simulate(off_duration, False, (0.0, -10.0, 0.0), (0.02, 0.0, 0.0))
	return times, percentages, voltages, solar_currents




def _plot_battery_profile(
		times: Sequence[float],
		percentages: Sequence[float],
		voltages: Sequence[float],
		solar_currents: Sequence[float],
		filename: str = "power_model_battery.png",
	) -> str:
	from matplotlib import pyplot as plt  # type: ignore[import]

	fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

	line_soc, = ax_top.plot(times, percentages, label="Battery %", color="tab:blue")
	ax_top.set_ylabel("Battery [%]", color="tab:blue")
	ax_top.set_ylim(0, 100)
	ax_top.tick_params(axis="y", labelcolor="tab:blue")

	ax_voltage = ax_top.twinx()
	line_voltage, = ax_voltage.plot(times, voltages, label="Voltage", color="tab:red")
	ax_voltage.set_ylabel("Voltage [V]", color="tab:red")
	ax_voltage.tick_params(axis="y", labelcolor="tab:red")
	ax_top.legend([line_soc, line_voltage], ["Battery %", "Voltage"], loc="upper right")

	line_solar, = ax_bottom.plot(times, solar_currents, label="Solar Current", color="tab:green")
	ax_bottom.set_xlabel("Time [s]")
	ax_bottom.set_ylabel("Current [A]")
	ax_bottom.grid(True, alpha=0.3)
	ax_bottom.legend(loc="upper right")

	fig.suptitle("Battery State and Solar Input Over Time")
	fig.tight_layout()
	fig.savefig(filename)
	plt.close(fig)
	return filename


def main() -> None:
	times, percentages, voltages, solar_currents = run_power_profile_test()
	output = _plot_battery_profile(times, percentages, voltages, solar_currents)
	print(f"Power model plot saved to {output}")


if __name__ == "__main__":
	main()
