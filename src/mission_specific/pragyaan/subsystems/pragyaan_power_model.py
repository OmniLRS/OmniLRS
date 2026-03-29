__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import Enum
from typing import Tuple
import numpy as np

from src.subsystems.robot_physics_models.power_model import PowerModel


class PragyaanPowerModelDefaults(float, Enum):
    DEVICE_CURRENT_NOISE = 0.02
    BATTERY_PERCENTAGE_NOISE = 0.5
    BATTERY_VOLTAGE_NOISE = 0.05
    SOLAR_INPUT_NOISE = 0.1
    REGULATED_BUS_VOLTAGE = 5.0        # Volts
    DC_DC_EFFICIENCY = 0.95            # 95% efficient battery -> 5V conversion
    DEVICE_FAULT_EXTRA_POWER = 2.5     # Watts added when device is faulted


class PragyaanPowerModel(PowerModel):
    """Pragyaan-specific power model. Overrides the base power model defaults
    with values from PragyaanPowerModelDefaults to allow mission-specific tuning.
    initialize()/set_inputs()/compute()/get_outputs() can also be overridden to implement the logic for computing the power metrics based on mission specifics.    
    """

    PM = PragyaanPowerModelDefaults

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
