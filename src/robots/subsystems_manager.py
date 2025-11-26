from enum import StrEnum, Enum

from src.robots.RadioModel import RadioModel
from src.robots.ThermalModel import ThermalModel
from src.robots.PowerModel import PowerModel


class PowerState(StrEnum):
    OFF = "OFF",
    ON = "ON",

class SolarPanelState(StrEnum):
    STOWED = "STOWED",
    DEPLOYED = "DEPLOYED",

class GoNogoState(Enum):
    NOGO = 0
    GO = 1
    # UNDEF = "Other"

class ObcState(Enum):
    OFF = 0
    BOOT = 1
    IDLE = 2
    CAMERA = 3
    MOTOR = 4
    SAFE = 5
    ERROR = 6

class RobotSubsystemsManager:
    SUN_POSITION = (10.0, 5.0, 7.5)
    LANDER_POSITION = (0.0, 0.0, 0.0)

    def __init__(self):
        self._electronics_power_state = {
            "CAMERA": PowerState.OFF,
            "MOTOR_CONTROLLER": PowerState.OFF,
            "NEUTRON_SPECTROMETER": PowerState.OFF,
            "APXS": PowerState.OFF,
            "RADIO": PowerState.OFF,
        }
        self._solar_panel_state = SolarPanelState.STOWED
        self._go_nogo_state = GoNogoState.NOGO
        self._obc_state = ObcState.IDLE
        self._radio:RadioModel = RadioModel()
        self._thermal:ThermalModel = ThermalModel()
        self._power:PowerModel = PowerModel()

    def map_into_currents(self):
        mapped = {
            "current_draw_obc": True,
            "current_draw_motor_controller": self.is_turned_on("MOTOR_CONTROLLER"),
            "current_draw_neutron_spectrometer": self.is_turned_on("NEUTRON_SPECTROMETER"),
            "current_draw_apxs": self.is_turned_on("APXS"),
            "current_draw_camera": self.is_turned_on("CAMERA"),
            "current_draw_radio": self.is_turned_on("RADIO"),
            "current_draw_eps": True,
        }

        print(mapped)

        return mapped
    
    def is_turned_on(self, electronic):
        if self._electronics_power_state[electronic] == PowerState.ON:
            return True
        
        return False

    def calculate_rssi(self, robot_position):
        self._radio.rover_position = robot_position
        rssi = self._radio.rssi()

        return rssi
    
    def calculate_temperature(self, robot_position, interval_s):
        self._thermal.rover_position = robot_position
        self._thermal.sun_position = self.SUN_POSITION
        self._thermal.step(interval_s)
        t = self._thermal.temperatures()

        return t
    
    def calculate_power_status(self, robot_position, interval_s):
        self._power.set_device_states(self.map_into_currents())
        self._power.set_sun_position(self.SUN_POSITION)
        self._power.set_rover_position(robot_position)
        self._power.step(interval_s)
        status = self._power.status()
   
        return status

    def turn_on(self, electronics:str):
        if electronics not in self._electronics_power_state:
            print("Invalid electronics naming: ", electronics)
            return

        self._electronics_power_state[electronics] = PowerState.ON

    def turn_off(self, electronics:str):
        if electronics not in self._electronics_power_state:
            print("Invalid electronics naming: ", electronics)
            return

        self._electronics_power_state[electronics] = PowerState.OFF

    def get_electronics_state(self, electronics:str):
        if electronics not in self._electronics_power_state:
            print("Invalid electronics naming: ", electronics)
            return

        return self._electronics_power_state[electronics]
    
    def get_electronics_states(self):
        return self._electronics_power_state

    def deploy_solar(self):
        self._solar_panel_state = SolarPanelState.DEPLOYED

    def stow_solar(self):
        self._solar_panel_state = SolarPanelState.STOWED

    def get_solar_state(self):
        return self._solar_panel_state
    
    def set_go_nogo_state(self, new_state:GoNogoState):
        self._go_nogo_state = new_state

    def get_go_nogo_state(self):
        return self._go_nogo_state

    def get_obc_state(self):
        return self._obc_state
    
    def set_obc_to_camera(self):
        self._obc_state = ObcState.CAMERA

    def set_obc_to_motor(self):
        self._obc_state = ObcState.MOTOR

    def set_obc(self, state:ObcState):
        self._obc_state = state

    def set_obc_to_idle(self):
        self._obc_state = ObcState.IDLE
 
