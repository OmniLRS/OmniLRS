__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time

from src.environments_wrappers.zenoh.transport.factory import make_transports
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)


class CameraBridge:
    def __init__(self, robot_cfg: dict, zenoh_cfg: dict, RM: RobotManager):
        self.robot_cfg = robot_cfg
        self.zenoh_cfg = zenoh_cfg

        self.robot_name = self.robot_cfg["robot_name"]

        self.camera_cfg = self.robot_cfg["camera"]
        self.publish_period_s = self.robot_cfg["zenoh"]["camera"]["publish_period_s"]

        self.wire_format = self.robot_cfg["zenoh"]["camera"].get(
            "wire_format",
            self.zenoh_cfg["default_wire_format"],
        )

        self.is_logging = self.robot_cfg["zenoh"]["camera"]["is_logging"]

        self.log_every_n = self.zenoh_cfg["pub_log_every_n"]

        self.keyexpr_template = self.zenoh_cfg["keyexprs"]["camera"]

        self.RM = RM

        self.transports = []

        self.log = logger.info

        self._inited = False
        self._transports_started = False
        self._t_last_publish = 0.0

    def build_camera_keyexpr(self, camera_name: str, resolution: str) -> str:
        return self.keyexpr_template.format(robot_name=self.robot_name) + f"/{camera_name}/{resolution}"

    def make_transports(self):
        if self.camera_cfg:
            resolutions = self.camera_cfg["resolutions"]
            specs = []
            for res in resolutions:
                specs.append(
                    {
                        "type": "zenoh",
                        "keyexpr": self.build_camera_keyexpr(self.camera_cfg["name"], res),
                        "wire_format": self.wire_format,
                        "is_logging": self.is_logging,
                        "log_every_n": self.log_every_n,
                    }
                )
            self.transports = make_transports(specs)

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
            self.log(f"[camera_bridge] init failed: {e}")
            self._inited = False
            return False

    def update(self):
        """
        Publish current frame from each camera
        """
        if not self._inited or not self.camera_cfg:
            return False

        now = time.time()
        if (now - self._t_last_publish) < self.publish_period_s:
            return False
        self._t_last_publish = now

        for i, transport in enumerate(self.transports):
            resolution = transport.keyexpr.split("/")[-1]
            frame = self.RM.robot.get_rgba_camera_view(resolution)

            if frame.size != 0:
                transport.publish_array(frame)

        return True

    def close(self):
        self._inited = False

        for t in self.transports:
            try:
                t.close()
            except Exception:
                pass

        self._transports_started = False
        self.log("[camera_bridge] closed.")
