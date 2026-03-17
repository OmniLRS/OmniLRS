__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import StrEnum, Enum

from src.environments.utils import get_moon_env_name
from src.physics.robot_physics_models.RadioModel import RadioModel
from src.physics.robot_physics_models.ThermalModel import ThermalModel
from src.physics.robot_physics_models.PowerModel import PowerModel
import math
from isaacsim.core.utils.xforms import get_world_pose
import random
import time

from src.robots.device import Device, HealthState
from src.robots.neutron_spectrometer_model import NeutronSpectrometerModel
from src.robots.robot_enums import ObcState, SolarPanelState
from src.robots.robot_subsystems_handler import RobotSubsystemsHandler

class PragyaanSubsystemsHandler(RobotSubsystemsHandler):

    #TODO Take a look into sun calculations... was dynamic fetching of the sun from the sim fixed/implemented?
    SUN_DISTANCE = 1000. # m
    SUN_AZYMUTH_DEG = 65.0
    SUN_POSITION = (
        - SUN_DISTANCE * math.sin(math.pi * SUN_AZYMUTH_DEG / 180.0), 
        SUN_DISTANCE * math.cos(math.pi * SUN_AZYMUTH_DEG / 180.0), 
        10.0
    )
    LANDER_POSITION = (0.0, 0.0, 0.0)

    def __init__(self, pos_relative_to_prim):
        super().__init__()
        self._init_devices(["camera", "motor_controller", "neutron_spectrometer", "apxs", "radio", "obc", "eps"])
        self.LANDER_PATH = pos_relative_to_prim
        self._neutron_spectrometer = NeutronSpectrometerModel()

        if (pos_relative_to_prim != ""):
            self._lander_pos, rot = get_world_pose(self.LANDER_PATH) 
        else:
            self._lander_pos = self.LANDER_POSITION

        self._update_positions()

    def _init_devices(self, devices:list[str]):
        for device_name in devices:
            self._devices[device_name] = Device(device_name)

    def _update_positions(self):
        self._sun_pos = self.SUN_POSITION
        print(self._sun_pos)

    def _update_positions_before(func):
        # definition of a decorator
        def wrapper(self, *args, **kwargs):
            self._update_positions()
            return func(self, *args, **kwargs)
        return wrapper
    
    @_update_positions_before
    def get_rssi(self, robot_position):
        self._radio_model.update_inputs(robot_position, self._lander_pos)
        rssi = self._radio_model.get_rssi()

        return rssi
    
    @_update_positions_before
    def get_thermal_status(self, robot_position, robot_yaw_deg, interval_s):
        self._thermal_model.set_rover_yaw(robot_yaw_deg)
        self._thermal_model.set_rover_position(robot_position)
        self._thermal_model.set_sun_position(self._sun_pos)
        self._thermal_model.step(interval_s)
        t = self._thermal.temperatures()

        return t
    
    @_update_positions_before
    def get_power_status(self, robot_position, robot_yaw_deg, interval_s, obc_state):
        self._power_model.set_device_states(self._map_into_currents())
        self._power_model.set_sun_position(self._sun_pos)
        self._power_model.set_rover_position(robot_position)
        self._power_model.set_rover_yaw(robot_yaw_deg)
        sp_state = "deployed" if self._solar_panel_state == SolarPanelState.DEPLOYED else "stowed"
        self._power_model.set_solar_panel_state(sp_state)
        self._power_model.set_motor_state(obc_state == ObcState.MOTOR) 
        self._power_model.set_device_health(self._map_into_healths())
        self._power_model.step(interval_s)
        status = self._power_model.status()
   
        return status
    

    def get_lander_position(self):
        return self._lander_pos
    
    def set_battery_perc(self, battery_perc):
        #NOTE specific to workshop use case
        capacity = self._power.battery_capacity_wh
        new_charge = battery_perc / 100 * capacity
        self._power.battery_charge_wh = new_charge

    def get_neutron_count(self):
        #NOTE specific to workshop use case
        return self._neutron_spectrometer.get_next_count()
    
    def set_is_near_water(self, is_near):
        #NOTE specific to workshop use case
        self._neutron_spectrometer.is_near_water = is_near

    def set_device_health_state(self, electronics:str, status:HealthState):
        #NOTE specific to workshop use case
        if electronics not in self._electronics_health:
            print("Invalid electronics naming: ", electronics)
            return

        self._electronics_health[electronics] = status
    