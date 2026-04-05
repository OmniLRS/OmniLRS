__author__ = "Shamistan Karimov, Elian NEPPEL, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import os
import sys
import time
from typing import Callable, Dict, List, Tuple

import msgspec
import numpy as np

from src.configurations.simulator_mode_enum import SimulatorMode
from src.robots.robot import RobotManager

try:
    import omnilrs_artefacts
except ImportError:
    # just in case
    module_path = os.path.abspath(f"{os.path.dirname(__file__)}/../../../external/omnilrs_artefacts/src")
    sys.path.append(module_path)

from omnilrs_artefacts.control.articulation_controller import ArticulationController
from omnilrs_artefacts.telemetry.joint_force_bridge import JointForceBridge
from omnilrs_artefacts.transport.zenoh_cmd import ZenohCommandReceiver
from omnilrs_artefacts.transport.zenoh_pub import ZenohPubTransport


class Zenoh_RobotManager:
    """
    Zenoh wrapper that manages the robots.
    """

    def __init__(self, RM_conf: dict, zenoh_conf: dict) -> None:
        self.RM = RobotManager(RM_conf, mode=SimulatorMode.ZENOH)

        self.modifications: List[Tuple[Callable, dict]] = []

        self.transports: List[ZenohPubTransport] = []
        self.cams: List[ZenohPubTransport] = []

        robot = RM_conf["parameters"]

        robot_name = f'{robot["robot_name"]}'
        robot_path = self.RM.robots_root + "/" + robot_name

        if isinstance(robot["camera"], list):
            for i, camera in enumerate(robot["camera"]):
                cam_pub = ZenohPubTransport(
                    keyexpr=f'{zenoh_conf["sensors"]["camera"]["base_keyexpr"]}/{robot["camera"][i]["name"]}',
                    json_compact=zenoh_conf["sensors"]["camera"]["json_compact"],
                )
                self.cams.append(cam_pub)
                self.transports.append(cam_pub)
        else:
            cam_pub = ZenohPubTransport(
                keyexpr=f'{zenoh_conf["sensors"]["camera"]["base_keyexpr"]}/{robot["camera"]["name"]}',
                json_compact=zenoh_conf["sensors"]["camera"]["json_compact"],
            )
            self.cams.append(cam_pub)
            self.transports.append(cam_pub)

        gt_pub = ZenohPubTransport(
            keyexpr=f"{robot_name}/gt_pose",
            json_compact=False,
        )
        self.gt = gt_pub
        self.transports.append(gt_pub)

        self.joint_bridge = JointForceBridge(
            transports=[
                {"type": "zenoh", "keyexpr": f"{robot_name}/joint_telemetry"},
            ],
            robot_root_prim=robot_path,
        )

        self.controller = ArticulationController(
            prim_path=robot_path,
        )

        self.cmd_receiver = ZenohCommandReceiver(
            controller=self.controller,
            keyexpr=f"{robot_name}/joint_cmd",
        )

        self.resolution = zenoh_conf["sensors"]["camera"]["resolution"]

        self.transports_inited = False

    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """
        self.clear_modifications()
        self.reset_robot()

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """
        self.modifications: List[Tuple[Callable, dict]] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """
        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def reset_robot(self) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robot, {}])

    def publish_cameras(self) -> None:
        """
        Publish current frame from each camera
        """
        if self.transports_inited:

            for i, cam in enumerate(self.cams):
                if len(self.cams) > 1:
                    frame = self.RM.robot.get_rgba_camera_view_by_idx(i, self.resolution)
                else:
                    frame = self.RM.robot.get_rgba_camera_view(self.resolution)

                if frame.size != 0:
                    cam.publish_array(frame)

    def publish_telemetry(self) -> None:
        self.joint_bridge.maybe_initialize()
        self.joint_bridge.update()

    def update_controller(self) -> None:
        self.controller.maybe_initialize()
        self.controller.update()

    def update_cmd(self) -> None:
        self.cmd_receiver.start()

    def publish_gt(self) -> None:
        if self.transports_inited:
            pos, quat = self.RM.robot.get_pose()

            gt = {
                "stamp_s": time.time(),
                "robot_name": self.RM.robot.robot_name,
                "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                "orientation_xyzw": [
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                ],
            }

            self.gt.publish(gt)
