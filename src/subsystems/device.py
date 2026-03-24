__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC, abstractmethod
from enum import Enum, StrEnum
from typing import Tuple

class PowerState(StrEnum):
    OFF = "OFF"
    ON = "ON"

class HealthState(Enum):
    NOMINAL = 0
    FAULT = 1

class CommonDevice(StrEnum):
    MOTOR_CONTROLLER = "MOTOR_CONTROLLER"
    RADIO = "RADIO"
    OBC = "OBC"
    APXS = "APXS"
    CAMERA = "CAMERA"
    NEUTRON_SPECTROMETER = "NEUTRON_SPECTROMETER"
    EPS = "EPS"

class Device():
    def __init__(
        self,
        name:str,
        power_state:PowerState = PowerState.OFF,
        health_state:HealthState = HealthState.NOMINAL,
        current_draw:Tuple[float, float] = (0.0, 0.0), # (min, max)
    ) -> None:
        self._name = name
        self._power_state = power_state
        self._health_state = health_state
        self._current_draw = current_draw

    def get_health_state(self):
        return self._health_state 
    
    def set_health_state(self, health_state:HealthState):
        self._health_state = health_state
    
    def get_power_state(self):
        return self._power_state 
    
    def is_turned_on(self):
        return self._power_state == PowerState.ON
    
    def set_power_state(self, power_state:PowerState):
        self._power_state = power_state

    def get_current_draw(self):
        return self._current_draw