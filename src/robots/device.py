from abc import ABC, abstractmethod
from enum import Enum, StrEnum
from typing import Tuple

#TODO look into why is StrEnum, and change into normal Enum
class PowerState(StrEnum):
    OFF = "OFF"
    ON = "ON"

class HealthState(Enum):
    NOMINAL = 0
    FAULT = 1

class CommonDevice(Enum):
    NOMINAL = 0
    FAULT = 1

class CommonDevice(StrEnum):
    MOTOR_CONTROLLER = "motor_controller"
    RADIO = "radio"
    OBC = "obc"

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