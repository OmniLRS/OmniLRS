__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.xforms import get_world_pose
from scipy.spatial.transform import Rotation

from src.mission_specific.husky.subsystems.husky_power_model import (
    HuskyPowerModel,
)
from src.subsystems.device import CommonDevice, Device, PowerState
from src.subsystems.robot_enums import ObcState
from src.subsystems.robot_physics_models.obc_metrics_model import (
    ObcMetricsModel,
)
from src.subsystems.robot_physics_models.radio_model import RadioModel
from src.subsystems.robot_physics_models.thermal_model import ThermalModel
from src.subsystems.robot_subsystems_handler import RobotSubsystemsHandler


class HuskySubsystemsHandler(RobotSubsystemsHandler):
    BASE_STATION_POSITION = (0.0, 0.0, 0.0)

    BATTERY_CAPACITY_WH = 389.0  # Wh  (Husky uses 24 V / 16.2 Ah ≈ 389 Wh)
    MOTOR_COUNT = 4
    MOTOR_POWER_W = 50.0  # W per motor (rough estimate)

    def __init__(self, pos_relative_to_prim: str = ""):
        thermal_model = ThermalModel()
        radio_model = RadioModel()
        obc_metrics_model = ObcMetricsModel()
        power_model = HuskyPowerModel()
        super().__init__(
            thermal_model=thermal_model,
            radio_model=radio_model,
            obc_metrics_model=obc_metrics_model,
            power_model=power_model,
        )
        self._setup_devices()
        self._setup_power_model()

        self._base_station_path = pos_relative_to_prim
        self._sun_prim_path = None
        self._sun_direction = np.array((0.0, 1.0, 0.0))

        if is_prim_path_valid(pos_relative_to_prim):
            self._base_station_pos, _ = get_world_pose(pos_relative_to_prim)
        else:
            if pos_relative_to_prim == "":
                print(
                    "[HuskySubsystemsHandler] WARNING: no base station prim "
                    "path provided. Defaulting to global position reporting."
                )
            else:
                print(
                    "[HuskySubsystemsHandler] WARNING: base station prim path "
                    f"'{self._base_station_path}' not found in the world. "
                    "Defaulting to global position reporting."
                )
            self._base_station_pos = self.BASE_STATION_POSITION

    def set_sun_prim_path(self, sun_prim_path: str):
        self._sun_prim_path = sun_prim_path

    def _setup_devices(self):
        self._devices[CommonDevice.OBC] = Device(
            CommonDevice.OBC,
            current_draw=(0.0, 5.0),
            power_state=PowerState.ON,
        )
        self._devices[CommonDevice.MOTOR_CONTROLLER] = Device(
            CommonDevice.MOTOR_CONTROLLER,
            current_draw=(0.0, 10.0),
            power_state=PowerState.ON,
        )
        self._devices[CommonDevice.CAMERA] = Device(
            CommonDevice.CAMERA,
            current_draw=(0.0, 4.0),
            power_state=PowerState.ON,
        )
        self._devices[CommonDevice.RADIO] = Device(
            CommonDevice.RADIO,
            current_draw=(0.0, 3.0),
            power_state=PowerState.ON,
        )
        self._devices[CommonDevice.EPS] = Device(
            CommonDevice.EPS,
            current_draw=(0.0, 1.0),
            power_state=PowerState.ON,
        )

    def _setup_power_model(self):
        self._power_model.initialize(
            battery_capacity_wh=self.BATTERY_CAPACITY_WH,
            battery_charge_wh=self.BATTERY_CAPACITY_WH,
            motor_count=self.MOTOR_COUNT,
            motor_power_w=self.MOTOR_POWER_W,
            devices=self._devices,
        )

    def _update_sun_direction(self):
        if self._sun_prim_path is not None:
            _, quat_wxyz = get_world_pose(self._sun_prim_path)
            w, x, y, z = quat_wxyz
            self._sun_direction = Rotation.from_quat([x, y, z, w]).apply([-1.0, 0.0, 0.0])

    @staticmethod
    def _update_sun_direction_before(func):
        def wrapper(self, *args, **kwargs):
            self._update_sun_direction()
            return func(self, *args, **kwargs)

        return wrapper

    def get_obc_model_outputs(self):
        self._obc_metrics_model.set_inputs(self._obc_state)
        self._obc_metrics_model.compute(dt=0.0)
        return self._obc_metrics_model.get_outputs()

    def get_radio_model_outputs(self, robot_position):
        self._radio_model.set_inputs(robot_position, self._base_station_pos)
        self._radio_model.compute(dt=0.0)
        return self._radio_model.get_outputs()

    @_update_sun_direction_before
    def get_thermal_model_outputs(self, robot_yaw_deg, interval_s):
        self._thermal_model.set_inputs(self._sun_direction, robot_yaw_deg)
        self._thermal_model.compute(interval_s)
        return self._thermal_model.get_outputs()

    def get_power_model_outputs(self, robot_yaw_deg, interval_s, obc_state):
        self._power_model.set_inputs(
            rover_yaw_deg=robot_yaw_deg,
            is_in_motor_state=(obc_state == ObcState.MOTOR),
        )
        self._power_model.compute(interval_s)
        return self._power_model.get_outputs()

    def set_battery_perc(self, battery_perc):
        capacity = self._power_model._battery_capacity_wh
        new_charge = battery_perc / 100 * capacity
        self._power_model._battery_charge_wh = new_charge

    def get_base_station_position(self):
        return self._base_station_pos
