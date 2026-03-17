from abc import ABC, abstractmethod
from enum import Enum, StrEnum

#TODO look into why is StrEnum, and change into normal Enum
class PowerState(StrEnum):
    OFF = "OFF"
    ON = "ON"


class HealthState(Enum):
    NOMINAL = 0
    FAULT = 1

class Device(ABC):
    def __init__(
        self,
        name:str,
        power_state:PowerState = PowerState.OFF,
        health_state:HealthState = HealthState.NOMINAL
    ) -> None:
        self._name = name
        self._power_state = power_state
        self._health_state = health_state

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