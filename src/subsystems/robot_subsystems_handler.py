__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC, abstractmethod
from enum import Enum

from src.subsystems.robot_physics_models.power_model import PowerModel
from src.subsystems.robot_physics_models.radio_model import RadioModel
from src.subsystems.robot_physics_models.thermal_model import ThermalModel
from src.subsystems.device import Device, PowerState
from src.subsystems.robot_physics_models.obc_metrics_model import ObcMetricsModel
from src.subsystems.robot_enums import GoNogoState, ObcState, SolarPanelState

class RobotSubsystemsHandler(ABC):
    def __init__(
        self,
        power_model:PowerModel = PowerModel(),
        thermal_model:ThermalModel = ThermalModel(),
        radio_model:RadioModel = RadioModel(),
        obc_metrics_model:ObcMetricsModel = ObcMetricsModel(),
        obc_state:ObcState = ObcState.IDLE,
        go_nogo_state:GoNogoState = GoNogoState.NOGO,
        devices:dict[str, Device] = {},
        solar_panel_state:SolarPanelState = SolarPanelState.STOWED,
    ) -> None:
        self._power_model:PowerModel = power_model
        self._thermal_model:ThermalModel = thermal_model
        self._radio_model:RadioModel = radio_model
        self._obc_metrics_model:ObcMetricsModel = obc_metrics_model
        self._obc_state = obc_state
        self._go_nogo_state = go_nogo_state
        self._devices:dict[str, Device] = devices
        self._solar_panel_state = solar_panel_state

    @abstractmethod
    def get_radio_status(self):
        pass
    
    @abstractmethod
    def get_thermal_status(self):
        pass
    
    @abstractmethod
    def get_power_status(self):
        pass

    def get_obc_status(self):
        self._obc_metrics_model.set_inputs(self._obc_state)
        self._obc_metrics_model.compute()
        return self._obc_metrics_model.get_outputs()

    @abstractmethod
    def _setup_devices(self):
        pass

    @abstractmethod
    def _setup_power_model(self):
        pass

    def get_go_nogo_state(self):
        return self._go_nogo_state

    def set_go_nogo_state(self, go_nogo_state):
        self._go_nogo_state = go_nogo_state

    def get_obc_state(self):
        return self._obc_state

    def set_obc_state(self, obc_state):
        self._obc_state = obc_state

    def get_solar_panel_state(self):
        return self._solar_panel_state

    def set_solar_panel_state(self, state:SolarPanelState):
        self._solar_panel_state = state

    def get_device_power_state(self, device_name):
        return self._devices[device_name].get_power_state()
    
    def set_device_power_state(self, device_name, power_state:PowerState):
        self._devices[device_name].set_power_state(power_state)
    
    def get_device_health_state(self, device_name):
        return self._devices[device_name].get_health_state()
