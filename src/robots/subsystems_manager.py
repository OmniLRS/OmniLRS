from enum import StrEnum, Enum

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

class RobotSubsystemsManager:
    def __init__(self):
        self._electronics_power_state = {
            "CAMERA": PowerState.OFF,
            "MOTOR_CONTROLLER": PowerState.OFF,
            "NEUTRON_SPECTROMETER": PowerState.OFF,
            "APXS": PowerState.OFF,
            "RADIO": PowerState.OFF,
        }
        self._solar_panel_state = SolarPanelState.STOWED
        self.go_nogo_state = GoNogoState.NOGO

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

    def deploy_solar(self):
        self._solar_panel_state = SolarPanelState.DEPLOYED

    def stow_solar(self):
        self._solar_panel_state = SolarPanelState.STOWED

    def get_solar_state(self):
        return self._solar_panel_state
    
    def set_go_nogo_state(self, new_state:GoNogoState):
        self.go_nogo_state = new_state

    def get_go_nogo_state(self):
        return self.go_nogo_state
 
