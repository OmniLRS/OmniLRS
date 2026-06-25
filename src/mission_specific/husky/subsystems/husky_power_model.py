__author__ = "Amaan Javed"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import Enum
from typing import Tuple

from src.subsystems.robot_physics_models.power_model import PowerModel, PowerModelDefaults


class HuskyPowerModelDefaults(float, Enum):
    DEVICE_CURRENT_NOISE = PowerModelDefaults.DEVICE_CURRENT_NOISE
    BATTERY_PERCENTAGE_NOISE = 0.05   # smaller than default 0.5 — Husky's large 389 Wh battery makes noise dominate otherwise
    BATTERY_VOLTAGE_NOISE = 0.02      # smaller than default 0.05
    REGULATED_BUS_VOLTAGE = 24.0      # Husky runs on a 24 V bus
    DC_DC_EFFICIENCY = PowerModelDefaults.DC_DC_EFFICIENCY
    DEVICE_FAULT_EXTRA_POWER = PowerModelDefaults.DEVICE_FAULT_EXTRA_POWER


class HuskyPowerModel(PowerModel):
    """Husky-specific power model.

    Overrides BATTERY_VOLTAGE_CURVE to reflect the Husky's 24 V / 16.2 Ah
    sealed lead-acid battery (nominal 24 V, ~389 Wh).
    """

    PM = HuskyPowerModelDefaults

    # Voltage curve for a 24 V SLA battery as (percentage, voltage) points
    BATTERY_VOLTAGE_CURVE: Tuple[Tuple[float, float], ...] = (
        (0.0, 21.6),
        (0.1, 22.5),
        (0.2, 23.0),
        (0.4, 24.0),
        (0.6, 24.8),
        (0.8, 25.4),
        (0.9, 25.8),
        (1.0, 26.4),
    )
