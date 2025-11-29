"""Simple power model with battery, solar charging, and device loads."""

from __future__ import annotations

import random
from collections.abc import Mapping as MappingABC, Sequence as SequenceABC
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Sequence, Tuple

import numpy as np

# Device settings: yamcs parameter name (keys) and power draw bounds in Watts (values)
DEVICE_SETTINGS: Dict[str, Tuple[float, float]] = {
	"current_draw_obc": (0.0, 7.5),
	"current_draw_motor_controller": (0.0, 2.0),
	"current_draw_neutron_spectrometer": (0.0, 9),
	"current_draw_apxs": (0.0, 9),
	"current_draw_camera": (0.0, 5.0),
	"current_draw_radio": (0.0, 5.0),
	"current_draw_eps": (0.0, 1.0),
}

# Battery voltage curve as (percentage, voltage) points
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

SOLAR_PANEL_NORMALS = {
	"deployed": np.array((0.0, -1.0, 0.0)),
	"stowed": np.array((0.0, 0.0, 1.0)),
}
BATTERY_CAPACITY_WH = 60.0  # Watt-hours
SOLAR_PANEL_MAX_POWER = 30.0  # Watts
MOTOR_COUNT = 6
MOTOR_POWER_W = 10.0
REGULATED_BUS_VOLTAGE = 5.0  # Volts
DC_DC_EFFICIENCY = 0.95  # 95% efficient battery -> 5V conversion
DEVICE_HEALTH_NOMINAL = "NOMINAL"
DEVICE_HEALTH_FAULT = "FAULT"
DEVICE_FAULT_EXTRA_POWER = 2.5  # Watts added when device is faulted

def _clamp(value: float, lower: float, upper: float) -> float:
	return max(lower, min(upper, value))


@dataclass
class PowerModel:
	"""Track battery charge based on solar input and device consumption.
        To integrate this model in a simulation, see the example in run_power_profile_test() below.
        inputs / computation / outputs are clearly separated for easy use.
    """

	battery_capacity_wh: float = BATTERY_CAPACITY_WH     
	battery_charge_wh: float = BATTERY_CAPACITY_WH      
	solar_panel_max_power: float = SOLAR_PANEL_MAX_POWER
	solar_panel_state: str = "stowed"
	motor_state: bool = False
	motor_count: int = MOTOR_COUNT
	motor_power_w: float = MOTOR_POWER_W
 
	device_power_settings: Mapping[str, Tuple[float, float]] = field(
		default_factory=lambda: dict(DEVICE_SETTINGS)
	)
	device_states: Dict[str, bool] = field(default_factory=dict)
	device_health: Dict[str, str] = field(default_factory=dict)

	device_current_noise_std: float = 0.02
	battery_percentage_noise_std: float = 0.5
	battery_voltage_noise_std: float = 0.05
	solar_input_noise_std: float = 0.1
 
	rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
	sun_position: Tuple[float, float, float] = (0.0, -10.0, 0.0)
	solar_input_power: float = 0.0
	battery_voltage_v: float = BATTERY_VOLTAGE_CURVE[-1][1]

	def __post_init__(self) -> None:
		for name in self.device_power_settings:
			self.device_states.setdefault(name, False)
			self.device_health.setdefault(name, DEVICE_HEALTH_NOMINAL)
		self.battery_charge_wh = _clamp(self.battery_charge_wh, 0.0, self.battery_capacity_wh)
		self._update_battery_voltage()

	def set_device_state(self, name: str, state: bool) -> None:
		if name not in self.device_power_settings:
			raise KeyError(f"Unknown device '{name}'")
		self.device_states[name] = state

	def set_device_states(self, states: Mapping[str, bool]) -> None:
		for name, state in states.items():
			self.set_device_state(name, state)

	def set_device_health(self, health_status: Mapping[str, str]) -> None:
		for name, status in health_status.items():
			if name not in self.device_power_settings:
				raise KeyError(f"Unknown device '{name}'")
			if status not in (DEVICE_HEALTH_NOMINAL, DEVICE_HEALTH_FAULT):
				raise ValueError(f"Invalid health '{status}' for device '{name}'")
			self.device_health[name] = status

	def set_rover_position(self, position: Tuple[float, float, float]) -> None:
		self.rover_position = position

	def set_sun_position(self, position: Tuple[float, float, float]) -> None:
		self.sun_position = position

	def set_all_devices(self, state: bool) -> None:
		for name in self.device_states:
			self.device_states[name] = state

	def set_solar_panel_state(self, state: str) -> None:
		if state not in SOLAR_PANEL_NORMALS:
			raise ValueError(f"Unknown solar panel state '{state}'")
		self.solar_panel_state = state

	def set_motor_state(self, state: bool) -> None:
		self.motor_state = state

	def _current_panel_normal(self) -> np.ndarray:
		return SOLAR_PANEL_NORMALS.get(self.solar_panel_state, SOLAR_PANEL_NORMALS["deployed"])

	def _device_power(self, name: str) -> float:
		lo, hi = self.device_power_settings[name]
		if self.device_states[name] == False:
			return lo
		elif self.device_health.get(name, DEVICE_HEALTH_NOMINAL) == DEVICE_HEALTH_FAULT:
			return hi + DEVICE_FAULT_EXTRA_POWER
		else:
			return hi

	def total_load_power(self) -> float:
		regulated_load = sum(self._device_power(name) for name in self.device_states)
		battery_power_for_regulated = regulated_load / DC_DC_EFFICIENCY
		motor_power = self.motor_power_w * self.motor_count if self.motor_state else 0.0
		return battery_power_for_regulated + motor_power

	def step(self, dt: float) -> None:
		"""Advance the battery state by *dt* seconds."""

		view_factor = self.compute_view_factor(self.rover_position, self.sun_position)
		self.solar_input_power = self.solar_panel_max_power * view_factor
		net_power = self.solar_input_power - self.total_load_power()
		self.battery_charge_wh += net_power * (dt / 3600.0)
		self.battery_charge_wh = _clamp(self.battery_charge_wh, 0.0, self.battery_capacity_wh)
		self._update_battery_voltage()

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
		panel_normal = self._current_panel_normal()
		return float(_clamp(float(np.dot(unit, panel_normal)), 0.0, 1.0))

	def battery_percentage(self) -> float:
		if self.battery_capacity_wh <= 0.0:
			return 0.0
		return 100.0 * self.battery_charge_wh / self.battery_capacity_wh

	def battery_voltage(self) -> float:
		return self.battery_voltage_v

	def _update_battery_voltage(self) -> None:
		if self.battery_capacity_wh <= 0.0:
			self.battery_voltage_v = BATTERY_VOLTAGE_CURVE[0][1]
			return
		fraction = _clamp(self.battery_charge_wh / self.battery_capacity_wh, 0.0, 1.0)
		curve = BATTERY_VOLTAGE_CURVE
		for idx in range(1, len(curve)):
			p_hi, v_hi = curve[idx]
			if fraction <= p_hi:
				p_lo, v_lo = curve[idx - 1]
				if p_hi == p_lo:
					self.battery_voltage_v = v_hi
					return
				slope = (v_hi - v_lo) / (p_hi - p_lo)
				self.battery_voltage_v = v_lo + slope * (fraction - p_lo)
				return
		self.battery_voltage_v = curve[-1][1]

	def measured_battery_percentage(self) -> float:
		value = self.battery_percentage()
		return _clamp(value + random.gauss(0.0, self.battery_percentage_noise_std), 0.0, 100.0)

	def measured_battery_voltage(self) -> float:
		value = self.battery_voltage()
		min_v = BATTERY_VOLTAGE_CURVE[0][1]
		max_v = BATTERY_VOLTAGE_CURVE[-1][1]
		return _clamp(value + random.gauss(0.0, self.battery_voltage_noise_std), min_v, max_v)

	def measured_device_currents(self) -> Dict[str, float]:
		"""Return measured device currents (A) derived from power draw."""
		currents = {}
		for name in self.device_states:
			device_power_w = self._device_power(name)
			currents[name] = device_power_w / REGULATED_BUS_VOLTAGE
		currents = {
			name: max(0.0, value + random.gauss(0.0, self.device_current_noise_std))
			for name, value in currents.items()
		}
		return currents

	def measured_motor_currents(self) -> Sequence[float]:
		voltage = max(self.battery_voltage_v, 1e-3)
		base_current = (self.motor_power_w / voltage) if self.motor_state else 0.0
		currents = [base_current for _ in range(self.motor_count)]
		return [
			max(0.0, value + random.gauss(0.0, self.device_current_noise_std))
			for value in currents
		]

	def measured_solar_input_current(self) -> float:
		power = _clamp(
			self.solar_input_power + random.gauss(0.0, self.solar_input_noise_std),
			0.0,
			self.solar_panel_max_power,
		)
		voltage = max(self.battery_voltage_v, 1e-3)
		return power / voltage
  
	def status(self) -> Dict[str, float | Dict[str, float] | Sequence[float]]:
		motor_currents = self.measured_motor_currents()
		device_currents = self.measured_device_currents()
		total_current_out = sum(device_currents.values()) + sum(motor_currents)
		status: Dict[str, float | Dict[str, float] | Sequence[float]] = {
			"net_power": self.solar_input_power - self.total_load_power(),
			"solar_input_current_measured": self.measured_solar_input_current(),
			"battery_percentage_measured": self.measured_battery_percentage(),
			"battery_voltage_measured": self.measured_battery_voltage(),
			"motor_currents_measured": motor_currents,
			"total_current_out_measured": total_current_out,
		}
		status["device_currents_measured"] = device_currents
		return status


def run_power_profile_test(
		on_duration: float = 800.0,
		off_duration: float = 800.0,
		dt: float = 1.0,
	) -> Tuple[
		Sequence[float],
		Sequence[float],
		Sequence[float],
		Sequence[float],
		Dict[str, Sequence[float]],
	]:
    
	model = PowerModel()
	times = [0.0]
	status = model.status()
	percentages = [status["battery_percentage_measured"]]
	voltages = [status["battery_voltage_measured"]]
	solar_currents = [status["solar_input_current_measured"]]
	current_series: Dict[str, list[float]] = {}
	for label, value in _extract_current_samples(status).items():
		current_series.setdefault(label, []).append(value)
	model.set_rover_position((0.0, 0.0, 0.0))
	model.set_solar_panel_state("stowed")
	devices_health = {name: DEVICE_HEALTH_NOMINAL for name in model.device_states}
	model.set_device_health(devices_health)

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
			model.set_rover_position(tuple(rover_pos))
			model.set_motor_state(devices_on)
			model.set_sun_position(sun_position)
			model.set_device_states(active_states)
			if _ > steps // 2:
				devices_health["current_draw_motor_controller"] = DEVICE_HEALTH_FAULT
				model.set_device_health(devices_health)
				model.set_solar_panel_state("deployed")

			# perform computation
			model.step(dt)

			# get results
			status = model.status()

			# for the plots
			times.append(times[-1] + dt)
			percentages.append(status["battery_percentage_measured"])
			voltages.append(status["battery_voltage_measured"])
			solar_currents.append(status["solar_input_current_measured"])
			for label, value in _extract_current_samples(status).items():
				series = current_series.setdefault(label, [0.0] * (len(times) - 1))
				while len(series) < len(times) - 1:
					series.append(0.0)
				series.append(value)
			for label, series in current_series.items():
				if len(series) < len(times):
					series.append(series[-1] if series else 0.0)
			rover_pos = rover_pos + np.asarray(rover_delta, dtype=float)

	# phase 1 all devices on, sun low horizon, rover moving forward
	simulate(on_duration, True, (0.0, -1000.0, 10.0), (0.5, 0.0, 0.0))
	# phase 2 all devices off, sun low horizon, rover moving forward
	simulate(off_duration, False, (0.0, -1000.0, 10.0), (0.5, 0.0, 0.0))
	return times, percentages, voltages, solar_currents, current_series

def _plot_battery_profile(
		times: Sequence[float],
		percentages: Sequence[float],
		voltages: Sequence[float],
		solar_currents: Sequence[float],
		current_series: Mapping[str, Sequence[float]],
		filename: str = "power_model_battery.png",
	) -> str:
	from matplotlib import pyplot as plt  # type: ignore[import]

	fig, (ax_top, ax_mid, ax_bottom) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

	line_soc, = ax_top.plot(times, percentages, label="Battery %", color="tab:blue")
	ax_top.set_ylabel("Battery [%]", color="tab:blue")
	ax_top.set_ylim(0, 100)
	ax_top.tick_params(axis="y", labelcolor="tab:blue")

	ax_voltage = ax_top.twinx()
	line_voltage, = ax_voltage.plot(times, voltages, label="Voltage", color="tab:red")
	ax_voltage.set_ylabel("Voltage [V]", color="tab:red")
	ax_voltage.tick_params(axis="y", labelcolor="tab:red")
	ax_top.legend([line_soc, line_voltage], ["Battery %", "Voltage"], loc="upper right")

	line_solar, = ax_mid.plot(times, solar_currents, label="Solar Current", color="tab:green")
	ax_mid.set_ylabel("Solar Current [A]")
	ax_mid.grid(True, alpha=0.3)
	ax_mid.legend(loc="upper right")

	for label, values in current_series.items():
		ax_bottom.plot(times, values, label=label)
	ax_bottom.set_xlabel("Time [s]")
	ax_bottom.set_ylabel("Currents [A]")
	ax_bottom.grid(True, alpha=0.3)
	ax_bottom.legend(loc="upper right", ncol=2)

	fig.suptitle("Battery, Solar Input, and Currents Over Time")
	fig.tight_layout()
	fig.savefig(filename)
	plt.close(fig)
	return filename

def _extract_current_samples(status: Mapping[str, Any]) -> Dict[str, float]:
	currents: Dict[str, float] = {}
	for key, value in status.items():
		if "current" not in key:
			continue
		if key == "total_current_out_measured":  # skipit
			continue
		if isinstance(value, MappingABC):
			for subkey, subvalue in value.items():
				currents[subkey] = float(subvalue)
		elif isinstance(value, SequenceABC) and not isinstance(value, (str, bytes, bytearray)):
			for idx, item in enumerate(value):
				currents[f"{key}_{idx}"] = float(item)
		else:
			currents[key] = float(value)
	return currents


def main() -> None:
	times, percentages, voltages, solar_currents, current_series = run_power_profile_test()
	output = _plot_battery_profile(times, percentages, voltages, solar_currents, current_series)
	print(f"Power model plot saved to {output}")


if __name__ == "__main__":
	main()
