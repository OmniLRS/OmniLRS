__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time

from src.environments_wrappers.zenoh.transport.factory import make_transports
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)


class CameraBridge:
    def __init__(self, robot_cfg: dict, mode_cfg: dict, streams_cfg: dict, RM: RobotManager):
        self.robot_cfg = robot_cfg
        self.mode_cfg = mode_cfg

        self.robot_name = self.robot_cfg["robot_name"]

        self.publish_period_s = streams_cfg["camera"]["publish_period_s"]

        self.wire_format = streams_cfg["camera"].get(
            "wire_format",
            self.mode_cfg["default_wire_format"],
        )

        self.is_logging = streams_cfg["camera"]["is_logging"]

        self.log_every_n = self.mode_cfg["pub_log_every_n"]

        self.keyexpr_template = self.mode_cfg["keyexprs"]["camera"]

        self.RM = RM

        self.transports = []
        self._camera_views = []  # (camera name, resolution) published by each transport

        self.log = logger.info

        self._inited = False
        self._transports_started = False
        self._t_last_publish = 0.0

    def build_camera_keyexpr(self, camera_name: str, resolution: str) -> str:
        return self.keyexpr_template.format(robot_name=self.robot_name) + f"/{camera_name}/{resolution}"

    def make_transports(self):
        # One transport per rgba camera and resolution of the robot yaml
        self._camera_views = [
            (camera.name, res)
            for camera in self.RM.robot.cameras
            if camera.has_type("rgba")
            for res in camera.resolutions
        ]
        specs = [
            {
                "type": "zenoh",
                "keyexpr": self.build_camera_keyexpr(camera_name, res),
                "wire_format": self.wire_format,
                "is_logging": self.is_logging,
                "log_every_n": self.log_every_n,
            }
            for camera_name, res in self._camera_views
        ]
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
        if not self._inited or not self.transports:
            return False

        now = time.time()
        if (now - self._t_last_publish) < self.publish_period_s:
            return False
        self._t_last_publish = now

        for transport, (camera_name, resolution) in zip(self.transports, self._camera_views):
            frame = self.RM.robot.get_rgba_camera_view(camera_name, resolution)

            if frame is not None:
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
