__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time

from src.environments_wrappers.zenoh.transport.factory import make_transports
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)


class IMUBridge:
    def __init__(self, robot_cfg: dict, zenoh_cfg: dict, RM: RobotManager):
        self.robot_cfg = robot_cfg
        self.zenoh_cfg = zenoh_cfg

        self.robot_name = self.robot_cfg["robot_name"]

        self.publish_period_s = self.robot_cfg["zenoh"]["imu"]["publish_period_s"]

        self.wire_format = self.robot_cfg["zenoh"]["imu"].get(
            "wire_format",
            self.zenoh_cfg["default_wire_format"],
        )

        self.is_logging = self.robot_cfg["zenoh"]["imu"]["is_logging"]

        self.log_every_n = self.zenoh_cfg["pub_log_every_n"]

        self.RM = RM

        self.transports = []

        self.log = logger.info

        self._inited = False
        self._transports_started = False
        self._t_last_publish = 0.0

    def make_transports(self):
        spec = {
            "type": "zenoh",
            "keyexpr": self.zenoh_cfg["keyexprs"]["imu"].format(robot_name=self.robot_name),
            "wire_format": self.wire_format,
            "is_logging": self.is_logging,
            "log_every_n": self.log_every_n,
        }
        self.transports = make_transports([spec])

    def maybe_initialize(self):
        if self._inited:
            return True

        try:
            self.make_transports()

            if not self._transports_started:
                for t in self.transports:
                    t.start()
                self._transports_started = True

            self._inited = True
            return True

        except Exception as e:
            self.log(f"[imu_bridge] init failed: {e}")
            self._inited = False
            return False

    def update(self):
        """
        Publish current imu readings
        """
        if not self._inited or self.RM.robot_parameters.imu_sensor_path == "":
            return False

        now = time.time()
        if (now - self._t_last_publish) < self.publish_period_s:
            return False
        self._t_last_publish = now

        lin_acc, ang_vel, ori_rpy = self.RM.robot.get_imu_readings()

        frame = {
            "stamp_s": now,
            "imu_prim_path": self.RM.robot_parameters.imu_sensor_path,
            "imu": {
                "lin_acc_m_s2": lin_acc,
                "ang_vel_rad_s": ang_vel,
                "orientation_rpy_deg": ori_rpy,
            },
        }

        for t in self.transports:
            try:
                t.publish(frame)
            except Exception as e:
                self.log(f"[imu_bridge] transport error: {e}")

        return True

    def close(self):
        self._inited = False

        for t in self.transports:
            try:
                t.close()
            except Exception:
                pass

        self._transports_started = False
        self.log("[imu_bridge] closed.")
