from enum import StrEnum, Enum

from src.environments.utils import get_moon_env_name
from src.robots.RadioModel import RadioModel
from src.robots.ThermalModel import ThermalModel
from src.robots.PowerModel import PowerModel
import math
from omni.isaac.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.xforms import get_world_pose

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
    LANDER_PATH = "/StaticAssets/lander"
    USE_DYNAMIC_SUN = False # Lunalab has no sun prim

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
        self._neutron_spectrometer = NeutronSpectrometerSimulator()
        self._update_positions()

    def _update_positions(self):
        if self.USE_DYNAMIC_SUN:
            self._sun_pos, rot = get_world_pose("/" + get_moon_env_name() + "/Sun/sun") # self.SUN_POSITION
        else: 
            self._sun_pos = self.SUN_POSITION
        self._lander_pos, rot = get_world_pose(self.LANDER_PATH) #  self.LANDER_POSITION
        print("sun", self._sun_pos)
        print("lander", self._lander_pos)

    def update_positions_before(func):
        # definition of a decorator
        def wrapper(self, *args, **kwargs):
            self._update_positions()
            return func(self, *args, **kwargs)
        return wrapper

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

        # print(mapped)

        return mapped
    
    def is_turned_on(self, electronic):
        if self._electronics_power_state[electronic] == PowerState.ON:
            return True
        
        return False

    @update_positions_before
    def calculate_rssi(self, robot_position):
        self._radio.rover_position = robot_position
        self._radio.lander_position = self._lander_pos
        rssi = self._radio.rssi()

        return rssi
    
    @update_positions_before
    def calculate_temperature(self, robot_position, interval_s):
        self._thermal.rover_position = robot_position
        self._thermal.sun_position = self._sun_pos
        self._thermal.step(interval_s)
        t = self._thermal.temperatures()

        return t
    
    @update_positions_before
    def calculate_power_status(self, robot_position, interval_s, obc_state):
        self._power.set_device_states(self.map_into_currents())
        self._power.set_sun_position(self._sun_pos)
        self._power.set_rover_position(robot_position)
        self._power.set_motor_state(obc_state == ObcState.MOTOR) 
        self._power.step(interval_s)
        status = self._power.status()
   
        return status
    
    def set_battery_perc(self, battery_perc):
        capacity = self._power.battery_capacity_wh
        new_charge = battery_perc / 100 * capacity
        self._power.battery_charge_wh = new_charge

    def set_electronics_state(self, electronics:str, state:PowerState):
        if electronics not in self._electronics_power_state:
            print("Invalid electronics naming: ", electronics)
            return

        self._electronics_power_state[electronics] = state

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

    def set_obc_state(self, state:ObcState):
        self._obc_state = state

    def get_neutron_count(self, interval_s):
        return self._neutron_spectrometer.get_next_count(interval_s)

class NeutronSpectrometerSimulator():
    
    def __init__(self):
        self._t = 0
        self._neutron_gen_values = {
            "min":0,
            "max":5000,
            "period": 20,
            "offset": 0,
        }

    def get_next_count(self, interval_s):
        generated_value = self._generate_sine_value(self._t, self._neutron_gen_values["min"], self._neutron_gen_values["max"], self._neutron_gen_values["period"], self._neutron_gen_values["offset"])
        self._t += interval_s
        
        return int(generated_value)
    
    def _generate_sine_value(self, t, min_val, max_val, period, offset):
        """Generate a sine wave value at time t with given parameters."""
        omega = 2 * math.pi / period
        amplitude = (max_val - min_val) / 2
        center = (max_val + min_val) / 2
        return center + amplitude * math.sin(omega * (t + offset))