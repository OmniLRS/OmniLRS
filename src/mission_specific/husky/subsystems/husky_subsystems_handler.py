__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"

import math

from isaacsim.core.utils.xforms import get_world_pose

from src.subsystems.device import CommonDevice, Device, HealthState, PowerState
from src.subsystems.robot_physics_models.obc_metrics_model import ObcMetricsModel
from src.subsystems.robot_physics_models.power_model import PowerModel
from src.subsystems.robot_physics_models.radio_model import RadioModel
from src.subsystems.robot_physics_models.thermal_model import ThermalModel
from src.subsystems.robot_subsystems_handler import RobotSubsystemsHandler
from src.subsystems.robot_enums import ObcState
from src.mission_specific.pragyaan.subsystems.neutron_spectrometer_model import NeutronSpectrometerModel


class HuskySubsystemsHandler(RobotSubsystemsHandler):
    SUN_DISTANCE = 1000.0  # m
    SUN_AZIMUTH_DEG = 65.0
    SUN_POSITION = (
        -SUN_DISTANCE * math.sin(math.pi * SUN_AZIMUTH_DEG / 180.0),
        SUN_DISTANCE * math.cos(math.pi * SUN_AZIMUTH_DEG / 180.0),
        10.0,
    )

    BATTERY_CAPACITY_WH = 389.0   # Wh  (Husky uses 24 V / 16.2 Ah ≈ 389 Wh)
    SOLAR_PANEL_MAX_POWER = 0.0   # W   (Husky has no solar panel)
    MOTOR_COUNT = 4
    MOTOR_POWER_W = 50.0          # W per motor (rough estimate)

    def __init__(self, pos_relative_to_prim: str = ""):
        thermal_model = ThermalModel()
        obc_metrics_model = ObcMetricsModel()
        power_model = PowerModel()
        super().__init__(
            thermal_model=thermal_model,
            obc_metrics_model=obc_metrics_model,
            power_model=power_model,
        )
        self._neutron_spectrometer = NeutronSpectrometerModel()
        self._setup_devices()
        self._setup_power_model()

        self._base_station_path = pos_relative_to_prim
        if pos_relative_to_prim:
            self._base_station_pos, _ = get_world_pose(pos_relative_to_prim)
        else:
            self._base_station_pos = (0.0, 0.0, 0.0)

        self._sun_pos = self.SUN_POSITION

    def _setup_devices(self):
        self._devices[CommonDevice.OBC] = Device(
            CommonDevice.OBC, current_draw=(0.0, 5.0), power_state=PowerState.ON
        )
        self._devices[CommonDevice.MOTOR_CONTROLLER] = Device(
            CommonDevice.MOTOR_CONTROLLER, current_draw=(0.0, 10.0), power_state=PowerState.ON
        )
        self._devices[CommonDevice.CAMERA] = Device(
            CommonDevice.CAMERA, current_draw=(0.0, 4.0), power_state=PowerState.ON
        )
        self._devices[CommonDevice.RADIO] = Device(
            CommonDevice.RADIO, current_draw=(0.0, 3.0), power_state=PowerState.ON
        )
        self._devices[CommonDevice.EPS] = Device(
            CommonDevice.EPS, current_draw=(0.0, 1.0), power_state=PowerState.ON
        )
        self._devices[CommonDevice.APXS] = Device(
            CommonDevice.APXS, current_draw=(0.0, 9.0), power_state=PowerState.ON
        )
        self._devices[CommonDevice.NEUTRON_SPECTROMETER] = Device(
            CommonDevice.NEUTRON_SPECTROMETER, current_draw=(0.0, 9.0), power_state=PowerState.ON
        )

    def _setup_power_model(self):
        self._power_model.initialize(
            battery_capacity_wh=self.BATTERY_CAPACITY_WH,
            battery_charge_wh=self.BATTERY_CAPACITY_WH,
            solar_panel_max_power=self.SOLAR_PANEL_MAX_POWER,
            solar_panel_state=self._solar_panel_state,
            motor_count=self.MOTOR_COUNT,
            motor_power_w=self.MOTOR_POWER_W,
            devices=self._devices,
        )

    def get_radio_status(self, robot_position):
        self._radio_model.set_inputs(robot_position, self._base_station_pos)
        return self._radio_model.get_rssi()

    def get_thermal_status(self, robot_position, robot_yaw_deg, interval_s):
        self._thermal_model.set_inputs(robot_position, self._sun_pos, robot_yaw_deg)
        self._thermal_model.compute(interval_s)
        return self._thermal_model.temperatures()

    def get_power_status(self, robot_position, robot_yaw_deg, interval_s, obc_state):
        self._power_model.set_inputs(
            rover_position=robot_position,
            sun_position=self._sun_pos,
            rover_yaw_deg=robot_yaw_deg,
            solar_panel_state=self._solar_panel_state,
            is_in_motor_state=(obc_state == ObcState.MOTOR),
        )
        self._power_model.compute(interval_s)
        return self._power_model.get_outputs()

    def get_neutron_count(self):
        return self._neutron_spectrometer.get_next_count()

    def set_is_near_water(self, is_near):
        self._neutron_spectrometer.is_near_water = is_near

    def set_battery_perc(self, battery_perc):
        capacity = self._power_model._battery_capacity_wh
        new_charge = battery_perc / 100 * capacity
        self._power_model._battery_charge_wh = new_charge

    def get_base_station_position(self):
        return self._base_station_pos