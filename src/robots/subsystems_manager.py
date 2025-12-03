from enum import StrEnum, Enum

from src.environments.utils import get_moon_env_name
from src.robots.RadioModel import RadioModel
from src.robots.ThermalModel import ThermalModel
from src.robots.PowerModel import PowerModel
import math
from isaacsim.core.utils.xforms import get_world_pose
import random
import time

class PowerState(StrEnum):
    OFF = "OFF"
    ON = "ON"

class SolarPanelState(Enum):
    STOWED = 0
    DEPLOYED = 1

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

class Electronics(Enum):
    CAMERA = "CAMERA"
    MOTOR_CONTROLLER = "MOTOR_CONTROLLER"
    NEUTRON_SPECTROMETER = "NEUTRON_SPECTROMETER"
    APXS = "APXS"
    RADIO = "RADIO"

class HealthStatus(Enum):
    NOMINAL = 0
    FAULT = 1

class RobotSubsystemsManager:
    # SUN_POSITION = (-100.0, -100.0, 10.0)
    SUN_DISTANCE = 1000. # m
    SUN_AZYMUTH_DEG = 65.0
    SUN_POSITION = (
        - SUN_DISTANCE * math.sin(math.pi * SUN_AZYMUTH_DEG / 180.0), 
        SUN_DISTANCE * math.cos(math.pi * SUN_AZYMUTH_DEG / 180.0), 
        10.0
    )

    LANDER_POSITION = (0.0, 0.0, 0.0)

    def __init__(self, pos_relative_to_prim):
        self.LANDER_PATH = pos_relative_to_prim
        self._electronics_power_state = {
            Electronics.CAMERA.value: PowerState.OFF,
            Electronics.MOTOR_CONTROLLER.value: PowerState.OFF,
            Electronics.NEUTRON_SPECTROMETER.value: PowerState.OFF,
            Electronics.APXS.value: PowerState.OFF,
            Electronics.RADIO.value: PowerState.ON,
        }
        self._electronics_health = {
            key: HealthStatus.NOMINAL for key in self._electronics_power_state
        }
        self._solar_panel_state = SolarPanelState.STOWED
        self._go_nogo_state = GoNogoState.NOGO
        self._obc_state = ObcState.IDLE
        self._radio:RadioModel = RadioModel()
        self._thermal:ThermalModel = ThermalModel()
        self._power:PowerModel = PowerModel()
        self._neutron_spectrometer = NeutronSpectrometerSimulator()
        self._obc_metrics_simulator = ObcMetricsSimulator()

        if (pos_relative_to_prim != ""):
            self._lander_pos, rot = get_world_pose(self.LANDER_PATH) #  self.LANDER_POSITION
        else:
            self._lander_pos = (0.0, 0.0, 0.0)

        self._update_positions()

    def _update_positions(self):
        # if get_moon_env_name() == "Lunaryard": # Lunalab has no sun prim
        #     self._sun_pos, rot = get_world_pose("/" + get_moon_env_name() + "/Sun/sun") # self.SUN_POSITION
        # else: 
        #     self._sun_pos = self.SUN_POSITION
        self._sun_pos = self.SUN_POSITION
        print(self._sun_pos)
        # print(get_moon_env_name())
        # print(get_moon_env_name() == "Lunaryard")

    def get_lander_position(self):
        return self._lander_pos

    def _update_positions_before(func):
        # definition of a decorator
        def wrapper(self, *args, **kwargs):
            self._update_positions()
            return func(self, *args, **kwargs)
        return wrapper

    def _map_into_currents(self):
        mapped = {
            "current_draw_obc": True,
            "current_draw_motor_controller": self.is_turned_on(Electronics.MOTOR_CONTROLLER.value),
            "current_draw_neutron_spectrometer": self.is_turned_on(Electronics.NEUTRON_SPECTROMETER.value),
            "current_draw_apxs": self.is_turned_on(Electronics.APXS.value),
            "current_draw_camera": self.is_turned_on(Electronics.CAMERA.value),
            "current_draw_radio": self.is_turned_on(Electronics.RADIO.value),
            "current_draw_eps": True,
        }

        return mapped
    
    def _map_into_healths(self):
        mapped = {
            "current_draw_obc": HealthStatus.NOMINAL.name,
            "current_draw_motor_controller":  self.get_electronics_health(Electronics.MOTOR_CONTROLLER.value).name,
            "current_draw_neutron_spectrometer":  self.get_electronics_health(Electronics.NEUTRON_SPECTROMETER.value).name,
            "current_draw_apxs":  self.get_electronics_health(Electronics.APXS.value).name,
            "current_draw_camera":  self.get_electronics_health(Electronics.CAMERA.value).name,
            "current_draw_radio":  self.get_electronics_health(Electronics.RADIO.value).name,
            "current_draw_eps":  HealthStatus.NOMINAL.name,
        }
        
        return mapped
    
    def is_turned_on(self, electronic):
        if self._electronics_power_state[electronic] == PowerState.ON:
            return True
        
        return False

    @_update_positions_before
    def calculate_rssi(self, robot_position):
        self._radio.rover_position = robot_position
        self._radio.lander_position = self._lander_pos
        rssi = self._radio.rssi()

        return rssi
    
    @_update_positions_before
    def calculate_temperature(self, robot_position, robot_yaw_deg, interval_s):
        self._thermal.set_rover_yaw(robot_yaw_deg)
        self._thermal.set_rover_position(robot_position)
        self._thermal.set_sun_position(self._sun_pos)
        self._thermal.step(interval_s)
        t = self._thermal.temperatures()

        return t
    
    @_update_positions_before
    def calculate_power_status(self, robot_position, robot_yaw_deg, interval_s, obc_state):
        self._power.set_device_states(self._map_into_currents())
        self._power.set_sun_position(self._sun_pos)
        self._power.set_rover_position(robot_position)
        self._power.set_rover_yaw(robot_yaw_deg)
        sp_state = "deployed" if self._solar_panel_state == SolarPanelState.DEPLOYED else "stowed"
        self._power.set_solar_panel_state(sp_state)
        self._power.set_motor_state(obc_state == ObcState.MOTOR) 
        self._power.set_device_health(self._map_into_healths())
        print("SUNCE")
        print(self._power.sun_position)
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

    def set_solar_panel_state(self, state:SolarPanelState):
        self._solar_panel_state = state

    def get_solar_panel_state(self):
        return self._solar_panel_state
    
    def set_go_nogo_state(self, new_state:GoNogoState):
        self._go_nogo_state = new_state

    def get_go_nogo_state(self):
        return self._go_nogo_state

    def get_obc_state(self):
        return self._obc_state

    def set_obc_state(self, state:ObcState):
        self._obc_state = state
        self._obc_metrics_simulator.input_obc_state(state)

    def input_obc_state(self, state:ObcState):
        self._obc_metrics_simulator.input_obc_state(state)

    def get_obc_cpu_usage(self):
        return self._obc_metrics_simulator.get_obc_cpu_usage()

    def get_obc_ram_usage(self):
        return self._obc_metrics_simulator.get_obc_ram_usage()

    def get_obc_disk_usage(self):
        return self._obc_metrics_simulator.get_obc_disk_usage()

    def get_obc_uptime(self):
        return self._obc_metrics_simulator.get_obc_uptime()

    def get_neutron_count(self, interval_s):
        # return self._neutron_spectrometer.get_next_count(interval_s)
        return self._neutron_spectrometer.get_next_count()
    
    def set_is_near_water(self, is_near):
        self._neutron_spectrometer.is_near_water = is_near

    def set_electronics_health(self, electronics:str, status:HealthStatus):
        if electronics not in self._electronics_health:
            print("Invalid electronics naming: ", electronics)
            return

        self._electronics_health[electronics] = status
    
    def is_electronics_healthy(self, electronics:str):
        if electronics not in self._electronics_health:
            print("Invalid electronics naming: ", electronics)
            return

        return self._electronics_health[electronics] == HealthStatus.NOMINAL
    
    def get_electronics_health(self, electronics:str):
        if electronics not in self._electronics_health:
            print("Invalid electronics naming: ", electronics)
            return

        return self._electronics_health[electronics]


class NeutronSpectrometerSimulator():
    
    def __init__(self):
        self._t = 0
        self._neutron_gen_values = {
            "min":0,
            "max":5000,
            "period": 20,
            "offset": 0,
        }
        self.is_near_water:bool = False
        self._std_neutron = 10

    #NOTE left this old implementation until confirmed the new implementation is correct
    # def get_next_count(self, interval_s):
    #     generated_value = self._generate_sine_value(self._t, self._neutron_gen_values["min"], self._neutron_gen_values["max"], self._neutron_gen_values["period"], self._neutron_gen_values["offset"])
    #     self._t += interval_s
        
    #     return int(generated_value)
    
    def get_next_count(self):
        if self.is_near_water:
            count =  max(0, 50 + random.gauss(0.0, self._std_neutron))
        else:
            count = max(0, 200 + random.gauss(0.0, self._std_neutron))

        return int(count)
    
    def _generate_sine_value(self, t, min_val, max_val, period, offset):
        """Generate a sine wave value at time t with given parameters."""
        omega = 2 * math.pi / period
        amplitude = (max_val - min_val) / 2
        center = (max_val + min_val) / 2
        return center + amplitude * math.sin(omega * (t + offset))
    
class ObcMetricsSimulator():
    """ Simulates OBC metrics like CPU, RAM, Disk usage and Uptime.
    Usage: 
        Instantiate the class, then call input_obc_state() to share the state of the OBC with this simulator.
        Then, call the get_*() methods to get the respective metrics.
    """
    def __init__(self):
        self._state = ObcState.IDLE
        self._boot_ts = time.monotonic()

    def input_obc_state(self, state:ObcState):
        # First, check if need to reset uptime (whenever the controller transitions through OFF)
        if state == ObcState.OFF or (self._state == ObcState.OFF and state != ObcState.OFF):
            self._boot_ts = time.monotonic()
        # keep track of the obc state internally:
        self._state = state

    def get_obc_cpu_usage(self):
        base = self._select_by_state(25.0, 50.0, 75.0)
        return self._usage_with_noise(base)

    def get_obc_ram_usage(self):
        base = self._select_by_state(40.0, 50.0, 60.0)
        return self._usage_with_noise(base)

    def get_obc_disk_usage(self):
        base = self._select_by_state(10.0, 25.0, 75.0)
        return self._usage_with_noise(base)

    def get_obc_uptime(self):
        elapsed = int(time.monotonic() - self._boot_ts)
        return elapsed % (2 ** 32)

    def _select_by_state(self, low, medium, high):
        if self._state == ObcState.CAMERA:
            return high
        if self._state == ObcState.MOTOR:
            return medium
        return low

    def _usage_with_noise(self, base, noise=2.0):
        noisy_value = base + random.uniform(-noise, noise)
        return max(0.0, min(100.0, noisy_value))


