__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time
from typing import Optional

from src.environments_wrappers.zenoh.transport.factory import make_transports
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)


class CameraBridge:
    def __init__(self, camera_cfg: Optional[dict], zenoh_cfg: dict, RM: RobotManager, publish_period_s: float = 0.0333):
        self.camera_cfg = camera_cfg
        self.zenoh_cfg = zenoh_cfg

        self.resolution = self.zenoh_cfg.get("resolution", None)
        self.publish_period_s = self.zenoh_cfg.get("publish_period_s", float(publish_period_s))

        self.base_expr_template = self.zenoh_cfg.get("base_keyexpr", "OmniLRS/{robot_name}/camera")

        self.RM = RM

        self.transports = []

        self.log = logger.info

        self._inited = False
        self._transports_started = False
        self._t_last_publish = 0.0

    def build_camera_keyexpr(self, camera_name: str) -> str:
        return self.base_expr_template.format(robot_name=self.RM.robot_parameters.robot_name) + f"/{camera_name}"

    def make_transports(self):
        if self.camera_cfg:
            if isinstance(self.camera_cfg, list):
                specs = []
                for camera in self.camera_cfg:
                    specs.append({"type": "zenoh", "keyexpr": self.build_camera_keyexpr(camera["name"])})

                self.transports = make_transports(specs)

            else:
                spec = {"type": "zenoh", "keyexpr": self.build_camera_keyexpr(self.camera_cfg["name"])}
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
            self.log(f"[camera_bridge] init failed: {e}")
            self._inited = False
            return False

    def update(self):
        """
        Publish current frame from each camera
        """
        if not self._inited or not self.camera_cfg or self.resolution is None:
            return False

        now = time.time()
        if (now - self._t_last_publish) < self.publish_period_s:
            return False
        self._t_last_publish = now

        for i, transport in enumerate(self.transports):
            if len(self.transports) > 1:
                frame = self.RM.robot.get_rgba_camera_view_by_idx(i, self.resolution)
            else:
                frame = self.RM.robot.get_rgba_camera_view(self.resolution)

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
