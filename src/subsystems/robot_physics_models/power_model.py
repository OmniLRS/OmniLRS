__author__ = "Louis Burtz, Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

"""Simple power model with battery, solar charging, and device loads."""

import math
import random
from collections.abc import Mapping as MappingABC, Sequence as SequenceABC
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Mapping, Sequence, Tuple
from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel
from src.subsystems.device import CommonDevice, Device, HealthState, PowerState
from src.subsystems.robot_enums import SolarPanelState

import numpy as np

def _clamp(value: float, lower: float, upper: float) -> float:
	return max(lower, min(upper, value))

class PowerModelDefaults(float, Enum):
    DEVICE_CURRENT_NOISE = 0.02
    BATTERY_PERCENTAGE_NOISE = 0.5
    BATTERY_VOLTAGE_NOISE = 0.05
    SOLAR_INPUT_NOISE = 0.1
    REGULATED_BUS_VOLTAGE = 5.0        # Volts
    DC_DC_EFFICIENCY = 0.95            # 95% efficient battery -> 5V conversion
    DEVICE_FAULT_EXTRA_POWER = 2.5     # Watts added when device is faulted

@dataclass
class PowerModel(RobotPhysicsModel):
    """Track battery charge based on solar input and device consumption.
        To integrate this model in a simulation, see the example in run_power_profile_test() below.
        inputs / computation / outputs are clearly separated for easy use.
    """

    PM = PowerModelDefaults

    SOLAR_PANEL_NORMALS = {
        "DEPLOYED": np.array((0.0, 1.0, 0.0)),
        "STOWED": np.array((0.0, 0.0, 1.0)),
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
	
    def __init__(self):
        super().__init__()
        self._solar_input_power: float = 0.0
        self._rover_yaw_deg: float = 0.0 
        self._rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._sun_position: Tuple[float, float, float] = (0.0, -10.0, 0.0)
        self._battery_voltage_v: float = self.BATTERY_VOLTAGE_CURVE[-1][1]
    
    def initialize(self, *, battery_capacity_wh:float, battery_charge_wh:float,
				   solar_panel_max_power:float, solar_panel_state:SolarPanelState, 
				   motor_count:int, motor_power_w:float, is_in_motor_state:bool=False,
				   devices:dict[str,Device]):
        #NOTE should be called from within rover subsystems handler
        #   setup for specific rover use-case
        self._battery_capacity_wh = battery_capacity_wh
        self._battery_charge_wh = battery_charge_wh
        self._solar_panel_max_power = solar_panel_max_power
        self._solar_panel_state = solar_panel_state
        self._devices = devices
        self._is_in_motor_state:bool = is_in_motor_state #NOTE rover is in motor state if its obc_state is MOTOR
        self._motor_count = motor_count
        self._motor_power_w = motor_power_w
	
    def set_inputs(self, rover_position, sun_position, rover_yaw_deg, solar_panel_state, is_in_motor_state):  
        self._rover_position = rover_position
        self._sun_position  = sun_position
        self._rover_yaw_deg = rover_yaw_deg
        self._solar_panel_state = solar_panel_state
        self._is_in_motor_state = is_in_motor_state

    def compute(self, dt: float) -> None:
        """Advance the battery state by *dt* seconds."""

        view_factor = self._compute_view_factor(self._rover_position, self._sun_position)
        self._solar_input_power = self._solar_panel_max_power * view_factor
        net_power = self._solar_input_power - self._total_load_power()
        self._battery_charge_wh += net_power * (dt / 3600.0)
        self._battery_charge_wh = _clamp(self._battery_charge_wh, 0.0, self._battery_capacity_wh)
        self._update_battery_voltage()

    def get_outputs(self) -> Dict[str, float | Dict[str, float] | Sequence[float]]:
        motor_currents = self._measured_motor_currents()
        device_currents = self._measured_device_currents()
        regulated_power = self.PM.REGULATED_BUS_VOLTAGE * sum(device_currents.values())
        # sum of currents
        battery_voltage = self._battery_voltage()
        device_current_at_battery = (regulated_power / self.PM.DC_DC_EFFICIENCY) / battery_voltage
        total_current_out = device_current_at_battery + sum(motor_currents)
        status: Dict[str, float | Dict[str, float] | Sequence[float]] = {
            "net_power": self._solar_input_power - self._total_load_power(),
            "solar_input_current_measured": self._measured_solar_input_current(),
            "battery_percentage_measured": self._measured_battery_percentage(),
            "battery_voltage_measured": self._measured_battery_voltage(),
            "motor_currents_measured": motor_currents,
            "total_current_out_measured": total_current_out,
        }
        status["device_currents_measured"] = device_currents
        return status

    def __post_init__(self) -> None:
        self._battery_charge_wh = _clamp(self._battery_charge_wh, 0.0, self._battery_capacity_wh)
        self._update_battery_voltage()

    def set_rover_position(self, position: Tuple[float, float, float]) -> None:
        self._rover_position = position

    def set_rover_yaw(self, yaw_deg: float) -> None:
        self._rover_yaw_deg = yaw_deg

    def set_sun_position(self, position: Tuple[float, float, float]) -> None:
        self._sun_position = position

    def set_solar_panel_state(self, state: SolarPanelState) -> None:
        self._solar_panel_state = state

    def set_is_in_motor_state(self, is_in_motor_state:bool) -> None: #TODO fix this, no if the controller is ON, but if OBC state is MOTOR
        self._is_in_motor_state = is_in_motor_state

    def _current_panel_normal(self) -> np.ndarray:
        base_normal = self.SOLAR_PANEL_NORMALS.get(
            self._solar_panel_state.name, self.SOLAR_PANEL_NORMALS["DEPLOYED"]
        )
        yaw_rad = math.radians(self._rover_yaw_deg)
        rotation = np.array(
            [
                [math.cos(yaw_rad), -math.sin(yaw_rad), 0.0],
                [math.sin(yaw_rad), math.cos(yaw_rad), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return rotation @ base_normal

    def _device_power(self, name: str) -> float:
        device = self._devices[name]
        lo, hi = device.get_current_draw()
		
        if device.get_power_state() == PowerState.OFF:
            return lo
        elif device.get_health_state() == HealthState.FAULT:
            return hi + self.PM.DEVICE_FAULT_EXTRA_POWER
        else:
            return hi

    def _total_load_power(self) -> float:
        regulated_load = sum(self._device_power(name) for name in self._devices)
        battery_power_for_regulated = regulated_load / self.PM.DC_DC_EFFICIENCY
        motor_power = self._motor_power_w * self._motor_count if self._is_in_motor_state else 0.0
        return battery_power_for_regulated + motor_power

    def _compute_view_factor(
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
        clamp = float(_clamp(float(np.dot(unit, panel_normal)), 0.0, 1.0))
        return clamp

    def _battery_percentage(self) -> float:
        if self._battery_capacity_wh <= 0.0:
            return 0.0
        return 100.0 * self._battery_charge_wh / self._battery_capacity_wh

    def _battery_voltage(self) -> float:
        return max(self._battery_voltage_v, 1e-3)

    def _update_battery_voltage(self) -> None:
        if self._battery_capacity_wh <= 0.0:
            self._battery_voltage_v = self.BATTERY_VOLTAGE_CURVE[0][1]
            return
        fraction = _clamp(self._battery_charge_wh / self._battery_capacity_wh, 0.0, 1.0)
        curve = self.BATTERY_VOLTAGE_CURVE
        for idx in range(1, len(curve)):
            p_hi, v_hi = curve[idx]
            if fraction <= p_hi:
                p_lo, v_lo = curve[idx - 1]
                if p_hi == p_lo:
                    self._battery_voltage_v = v_hi
                    return
                slope = (v_hi - v_lo) / (p_hi - p_lo)
                self._battery_voltage_v = v_lo + slope * (fraction - p_lo)
                return
        self._battery_voltage_v = curve[-1][1]

	### Measured outputs (adds noise) ###
    def _measured_battery_percentage(self) -> float:
        value = self._battery_percentage()
        return _clamp(value + random.gauss(0.0, self.PM.BATTERY_PERCENTAGE_NOISE), 0.0, 100.0)

    def _measured_battery_voltage(self) -> float:
        value = self._battery_voltage()
        min_v = self.BATTERY_VOLTAGE_CURVE[0][1]
        max_v = self.BATTERY_VOLTAGE_CURVE[-1][1]
        return _clamp(value + random.gauss(0.0, self.PM.BATTERY_VOLTAGE_NOISE), min_v, max_v)

    def _measured_device_currents(self) -> Dict[str, float]:
        """Return measured device currents (A) derived from power draw."""
        currents = {}
        for name in self._devices:
            device_power_w = self._device_power(name)
            currents[name] = device_power_w / self.PM.REGULATED_BUS_VOLTAGE
        currents = {
            name: max(0.0, value + random.gauss(0.0, self.PM.DEVICE_CURRENT_NOISE))
            for name, value in currents.items()
        }
        return currents

    def _measured_motor_currents(self) -> Sequence[float]:
        voltage = self._battery_voltage()
        base_current = (self._motor_power_w / voltage) if self._is_in_motor_state else 0.0
        currents = [base_current for _ in range(self._motor_count)]
        return [
            max(0.0, value + random.gauss(0.0, self.PM.DEVICE_CURRENT_NOISE))
            for value in currents
        ]

    def _measured_solar_input_current(self) -> float:
        power = _clamp(
            self._solar_input_power + random.gauss(0.0, self.PM.SOLAR_INPUT_NOISE),
            0.0,
            self._solar_panel_max_power,
        )
        voltage = self._battery_voltage()
        return power / voltage


# ---------------------------------------------------------------------------
# Below: standalone functions for unit-testing of the power model.
# They enable the model to be tested in isolation, without needing to run the entire simulation
# ---------------------------------------------------------------------------



def _extract_current_samples(status: Mapping[str, Any]) -> Dict[str, float]:
    currents: Dict[str, float] = {}
    for key, value in status.items():
        if "current" not in key:
            continue
        if key == "total_current_out_measured":
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

    devices: dict[str, Device] = {}
    devices[CommonDevice.OBC] = Device(CommonDevice.OBC, current_draw=(0.0, 7.5))
    devices[CommonDevice.MOTOR_CONTROLLER] = Device(CommonDevice.MOTOR_CONTROLLER, current_draw=(0.0, 2.0))
    devices["neutron_spectrometer"] = Device("neutron_spectrometer", current_draw=(0.0, 9.0))
    devices["apxs"] = Device("apxs", current_draw=(0.0, 9.0))
    devices["camera"] = Device("camera", current_draw=(0.0, 5.0))
    devices[CommonDevice.RADIO] = Device(CommonDevice.RADIO, current_draw=(0.0, 5.0))
    devices["eps"] = Device("eps", current_draw=(0.0, 1.0))

    BATTERY_CAPACITY_WH = 60.0
    SOLAR_PANEL_MAX_POWER = 30.0
    MOTOR_COUNT = 6
    MOTOR_POWER_W = 10.0

    model.initialize(
        battery_capacity_wh=BATTERY_CAPACITY_WH,
        battery_charge_wh=BATTERY_CAPACITY_WH,
        solar_panel_max_power=SOLAR_PANEL_MAX_POWER,
        solar_panel_state=SolarPanelState.STOWED,
        motor_count=MOTOR_COUNT,
        motor_power_w=MOTOR_POWER_W,
        devices=devices,
    )

    times = [0.0]
    status = model.get_outputs()
    percentages = [status["battery_percentage_measured"]]
    voltages = [status["battery_voltage_measured"]]
    solar_currents = [status["solar_input_current_measured"]]
    current_series: Dict[str, list[float]] = {}
    for label, value in _extract_current_samples(status).items():
        current_series.setdefault(label, []).append(value)

    SUN_DISTANCE = 1000.0
    SUN_AZYMUTH_DEG = 65.0
    SUN_POSITION = (
        -SUN_DISTANCE * np.sin(np.pi * SUN_AZYMUTH_DEG / 180.0),
        SUN_DISTANCE * np.cos(np.pi * SUN_AZYMUTH_DEG / 180.0),
        10.0,
    )
    print(SUN_POSITION)

    def simulate(
        duration: float,
        is_in_motor_state: bool,
        sun_position: Tuple[float, float, float],
        rover_yaw_deg: float,
        solar_panel_state: SolarPanelState,
    ) -> None:
        steps = int(duration / dt)
        rover_pos = (0.0, 0.0, 0.0)

        for _ in range(steps):
            # set inputs
            model.set_inputs(rover_pos, sun_position, rover_yaw_deg, solar_panel_state, is_in_motor_state)
            # perform computation
            model.compute(dt)
            # get results
            status = model.get_outputs()

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

    # phase 1: all motors on, solar panel stowed
    simulate(on_duration, True, SUN_POSITION, 65.0, SolarPanelState.STOWED)
    # phase 2: motors off, solar panel deployed
    simulate(off_duration, False, SUN_POSITION, -42.0, SolarPanelState.DEPLOYED)
    return times, percentages, voltages, solar_currents, current_series


def _plot_battery_profile(
    times: Sequence[float],
    percentages: Sequence[float],
    voltages: Sequence[float],
    solar_currents: Sequence[float],
    current_series: Mapping[str, Sequence[float]],
    filename: str = "test/outputs/power_model_battery.png",
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


def main() -> None:
    import os
    os.makedirs("test/outputs", exist_ok=True)
    times, percentages, voltages, solar_currents, current_series = run_power_profile_test()
    output = _plot_battery_profile(times, percentages, voltages, solar_currents, current_series)
    print(f"Power model plot saved to {output}")


if __name__ == "__main__":
    main()