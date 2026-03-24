__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import StrEnum, Enum

from src.environments.utils import get_moon_env_name
from src.physics.robot_physics_models.pragyaan_thermal_model import PragyaanThermalModel
from src.physics.robot_physics_models.radio_model import RadioModel
import math
from isaacsim.core.utils.xforms import get_world_pose
import random
import time

from src.robots.device import CommonDevice, Device, HealthState, PowerState
from src.robots.neutron_spectrometer_model import NeutronSpectrometerModel
from src.robots.robot_enums import ObcState, SolarPanelState
from src.robots.robot_subsystems_handler import RobotSubsystemsHandler

class PragyaanSubsystemsHandler(RobotSubsystemsHandler):

    #TODO @Louis Take a look into sun calculations... was dynamic fetching of the sun from the sim fixed/implemented?
    SUN_DISTANCE = 1000. # m
    SUN_AZYMUTH_DEG = 65.0
    SUN_POSITION = (
        - SUN_DISTANCE * math.sin(math.pi * SUN_AZYMUTH_DEG / 180.0), 
        SUN_DISTANCE * math.cos(math.pi * SUN_AZYMUTH_DEG / 180.0), 
        10.0
    )
    LANDER_POSITION = (0.0, 0.0, 0.0)

    # for setting up the Pragyaan-specific PowerModel:
    BATTERY_CAPACITY_WH = 60.0  # Watt-hours
    SOLAR_PANEL_MAX_POWER = 30.0  # Watts
    MOTOR_COUNT = 6
    MOTOR_POWER_W = 10.0

    def __init__(self, pos_relative_to_prim):
        thermal_model = PragyaanThermalModel()
        super().__init__(thermal_model=thermal_model)
        self._setup_devices()
        self._setup_power_model()
        self.LANDER_PATH = pos_relative_to_prim
        self._neutron_spectrometer = NeutronSpectrometerModel()

        if (pos_relative_to_prim != ""):
            self._lander_pos, rot = get_world_pose(self.LANDER_PATH) 
        else:
            self._lander_pos = self.LANDER_POSITION

        self._update_positions()

    def _setup_devices(self):
        # Pragyaan and workshop-specific values for the subsystem devices
        self._devices[CommonDevice.OBC] = Device(CommonDevice.OBC, current_draw=(0.0, 7.5), power_state=PowerState.ON)
        self._devices[CommonDevice.MOTOR_CONTROLLER] = Device(CommonDevice.MOTOR_CONTROLLER, current_draw=(0.0, 2.0))
        self._devices[CommonDevice.NEUTRON_SPECTROMETER] = Device(CommonDevice.NEUTRON_SPECTROMETER, current_draw=(0.0, 9.0))
        self._devices[CommonDevice.APXS] = Device(CommonDevice.APXS, current_draw=(0.0, 9.0))
        self._devices[CommonDevice.CAMERA] = Device(CommonDevice.CAMERA, current_draw=(0.0, 5.0))
        self._devices[CommonDevice.RADIO] = Device(CommonDevice.RADIO, current_draw=(0.0, 5.0), power_state=PowerState.ON)
        self._devices[CommonDevice.EPS] = Device(CommonDevice.EPS, current_draw=(0.0, 1.0), power_state=PowerState.ON)

    def _setup_power_model(self):
        self._power_model.setup(battery_capacity_wh=self.BATTERY_CAPACITY_WH, battery_charge_wh=self.BATTERY_CAPACITY_WH, 
                                solar_panel_max_power=self.SOLAR_PANEL_MAX_POWER, solar_panel_state=self._solar_panel_state, 
                                motor_count=self.MOTOR_COUNT, motor_power_w=self.MOTOR_POWER_W,
                                devices=self._devices)

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
        self._thermal_model.update_inputs(robot_position, self._sun_pos, robot_yaw_deg)
        self._thermal_model.step(interval_s)
        t = self._thermal_model.temperatures()

        return t
    
    @_update_positions_before
    def get_power_status(self, robot_position, robot_yaw_deg, interval_s, obc_state):
        # device states are reflected between the handler and power model, as they use the same dict
        self._power_model.update_inputs(rover_position=robot_position, 
                                        sun_position=self._sun_pos, 
                                        rover_yaw_deg=robot_yaw_deg, 
                                        solar_panel_state=self._solar_panel_state, 
                                        is_in_motor_state=(obc_state == ObcState.MOTOR))
        self._power_model.step(interval_s)
        status = self._power_model.status()
   
        return status

    def get_lander_position(self):
        return self._lander_pos
    
    def set_battery_perc(self, battery_perc):
        #NOTE specific to workshop use case
        capacity = self._power_model._battery_capacity_wh
        new_charge = battery_perc / 100 * capacity
        self._power_model._battery_charge_wh = new_charge

    def get_neutron_count(self):
        #NOTE specific to workshop use case
        return self._neutron_spectrometer.get_next_count()
    
    def set_is_near_water(self, is_near):
        #NOTE specific to workshop use case
        self._neutron_spectrometer.is_near_water = is_near

    def set_device_health_state(self, device_name:str, state:HealthState):
        #NOTE specific to workshop use case
        if device_name not in self._devices:
            print("Invalid electronics naming: ", device_name)
            return

        self._devices[device_name].set_health_state(state)
    