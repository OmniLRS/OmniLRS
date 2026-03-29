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
from typing import Any, Dict, Mapping, Sequence, Tuple
from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel
from src.subsystems.device import CommonDevice, Device, HealthState, PowerState
from src.subsystems.robot_enums import SolarPanelState

import numpy as np

def _clamp(value: float, lower: float, upper: float) -> float:
	return max(lower, min(upper, value))

DEVICE_CURRENT_NOISE:float = 0.02
BATTERY_PERCENTAGE_NOISE:float = 0.5
BATTERY_VOLTAGE_NOISE:float = 0.05
SOLAR_INPUT_NOISE:float = 0.1
REGULATED_BUS_VOLTAGE = 5.0  # Volts
DC_DC_EFFICIENCY = 0.95  # 95% efficient battery -> 5V conversion
DEVICE_FAULT_EXTRA_POWER = 2.5  # Watts added when device is faulted

SOLAR_PANEL_NORMALS = {
	SolarPanelState.DEPLOYED: np.array((0.0, 1.0, 0.0)),
	SolarPanelState.STOWED: np.array((0.0, 0.0, 1.0)),
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

@dataclass
class PowerModel(RobotPhysicsModel):
    """Track battery charge based on solar input and device consumption.
        To integrate this model in a simulation, see the example in run_power_profile_test() below.
        inputs / computation / outputs are clearly separated for easy use.
    """
	
    def __init__(self):
        super().__init__()
        self._solar_input_power: float = 0.0
        self._rover_yaw_deg: float = 0.0 
        self._rover_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._sun_position: Tuple[float, float, float] = (0.0, -10.0, 0.0)
        self._battery_voltage_v: float = BATTERY_VOLTAGE_CURVE[-1][1]
        #NOTE noise values were setup with default values, 
        # there are no implemented setters but the values may be altered from the subsystems manager if customization is desired
        self._device_current_noise = DEVICE_CURRENT_NOISE
        self._battery_percentage_noise = BATTERY_PERCENTAGE_NOISE
        self._battery_voltage_noise = BATTERY_VOLTAGE_NOISE
        self._solar_input_noise = SOLAR_INPUT_NOISE
        self._regulated_bus_voltage = REGULATED_BUS_VOLTAGE
        self._dc_dc_efficiency = DC_DC_EFFICIENCY
        self._device_fault_extra_power = DEVICE_FAULT_EXTRA_POWER
    
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

    def status(self) -> Dict[str, float | Dict[str, float] | Sequence[float]]:
        return self.get_outputs()

    def get_outputs(self) -> Dict[str, float | Dict[str, float] | Sequence[float]]:
        motor_currents = self._measured_motor_currents()
        device_currents = self._measured_device_currents()
        regulated_power = self._regulated_bus_voltage * sum(device_currents.values())
        # sum of currents
        battery_voltage = self._battery_voltage()
        device_current_at_battery = (regulated_power / self._dc_dc_efficiency) / battery_voltage
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
        base_normal = SOLAR_PANEL_NORMALS.get(
            self._solar_panel_state, SOLAR_PANEL_NORMALS[SolarPanelState.DEPLOYED]
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
            return hi + self._device_fault_extra_power
        else:
            return hi

    def _total_load_power(self) -> float:
        regulated_load = sum(self._device_power(name) for name in self._devices)
        battery_power_for_regulated = regulated_load / self._dc_dc_efficiency
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
            self._battery_voltage_v = BATTERY_VOLTAGE_CURVE[0][1]
            return
        fraction = _clamp(self._battery_charge_wh / self._battery_capacity_wh, 0.0, 1.0)
        curve = BATTERY_VOLTAGE_CURVE
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
        return _clamp(value + random.gauss(0.0, self._battery_percentage_noise), 0.0, 100.0)

    def _measured_battery_voltage(self) -> float:
        value = self._battery_voltage()
        min_v = BATTERY_VOLTAGE_CURVE[0][1]
        max_v = BATTERY_VOLTAGE_CURVE[-1][1]
        return _clamp(value + random.gauss(0.0, self._battery_voltage_noise), min_v, max_v)

    def _measured_device_currents(self) -> Dict[str, float]:
        """Return measured device currents (A) derived from power draw."""
        currents = {}
        for name in self._devices:
            device_power_w = self._device_power(name)
            currents[name] = device_power_w / self._regulated_bus_voltage
        currents = {
            name: max(0.0, value + random.gauss(0.0, self._device_current_noise))
            for name, value in currents.items()
        }
        return currents

    def _measured_motor_currents(self) -> Sequence[float]:
        voltage = self._battery_voltage()
        base_current = (self._motor_power_w / voltage) if self._is_in_motor_state else 0.0
        currents = [base_current for _ in range(self._motor_count)]
        return [
            max(0.0, value + random.gauss(0.0, self._device_current_noise))
            for value in currents
        ]

    def _measured_solar_input_current(self) -> float:
        power = _clamp(
            self._solar_input_power + random.gauss(0.0, self._solar_input_noise),
            0.0,
            self._solar_panel_max_power,
        )
        voltage = self._battery_voltage()
        return power / voltage