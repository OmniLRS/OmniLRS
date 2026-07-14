__author__ = "Shamistan Karimov, Elian NEPPEL, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import time
from typing import Callable, List, Tuple

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments_wrappers.zenoh.telemetry.camera_bridge import CameraBridge
from src.environments_wrappers.zenoh.telemetry.imu_bridge import IMUBridge
from src.environments_wrappers.zenoh.telemetry.joint_force_bridge import JointForceBridge
from src.environments_wrappers.zenoh.transport.zenoh_cmd import ZenohCommandReceiver
from src.environments_wrappers.zenoh.transport.zenoh_pub import ZenohPubTransport
from src.robots.robot import RobotManager


class Zenoh_RobotManager:
    """
    Zenoh wrapper that manages the robots.
    """

    def __init__(self, RM_conf: dict, zenoh_conf: dict) -> None:
        self.RM_ = RobotManager(RM_conf, mode=SimulatorMode.ZENOH)

        self.modifications: List[Tuple[Callable, dict]] = []

        self.transports: List[ZenohPubTransport] = []
        self.cams: List[ZenohPubTransport] = []

        robot = RM_conf["parameters"]

        robot_name = f"{robot['robot_name']}"
        robot_path = self.RM_.robots_root + "/" + robot_name

        ### BEGIN TELEMETRY ###

        ## Camera Telemetry
        camera_cfg = robot.get("camera", None)  # camera_cfg optional
        camera_zenoh_cfg = zenoh_conf.get("sensors", {}).get("camera", {})
        self.camera_bridge = CameraBridge(camera_cfg, camera_zenoh_cfg, self.RM_)

        ## IMU Telemetry
        imu_zenoh_cfg = zenoh_conf.get("sensors", {}).get("imu", {})
        self.imu_bridge = IMUBridge(imu_zenoh_cfg, self.RM_)

        ## Joint Force Telemetry
        joint_zenoh_cfg = zenoh_conf.get("sensors", {}).get("joint_force", {})
        self.joint_bridge = JointForceBridge(
            joint_zenoh_cfg,
            RM=self.RM_,
            robot_name=robot_name,
            robot_root_prim=robot_path,
        )
        ### END TELEMETRY ###

        gt_pub = ZenohPubTransport(
            keyexpr=zenoh_conf.get("misc", {})
            .get("sim", {})
            .get("gt_pose_keyexpr", "OmniLRS/{robot_name}/gt_pose")
            .format(robot_name=robot_name)
        )
        self.gt = gt_pub
        self.transports.append(gt_pub)

        self.cmd_receiver = ZenohCommandReceiver(
            RM=self.RM_,
            keyexpr=zenoh_conf.get("controller", {})
            .get("cmd_keyexpr", "OmniLRS/{robot_name}/joint_cmd")
            .format(robot_name=robot_name),
            wire_format=zenoh_conf.get("controller", {}).get("wire_format", "json"),
        )

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

    def get_RM(self) -> RobotManager:
        return self.RM_

    def reset_robot(self) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM_.reset_robot, {}])

    def publish_telemetry(self) -> None:
        self.camera_bridge.maybe_initialize()
        self.imu_bridge.maybe_initialize()
        self.joint_bridge.maybe_initialize()

        self.camera_bridge.update()
        self.imu_bridge.update()
        self.joint_bridge.update()

    def invalidate_articulation_api(self) -> bool:
        robot = getattr(self.RM_, "robot", None)
        if robot is None:
            return False

        robot.invalidate_articulation_api()
        return True

    def update_articulation_api(self) -> bool:
        robot = getattr(self.RM_, "robot", None)
        if robot is None:
            return False

        robot.update_articulation_api()
        return True

    def update_cmd(self) -> None:
        self.cmd_receiver.start()

    def publish_gt(self) -> None:
        if self.transports_inited:
            pos, quat = self.RM_.robot.get_pose()

            gt = {
                "stamp_s": time.time(),
                "robot_name": self.RM_.robot.robot_name,
                "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                "orientation_xyzw": [
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                ],
            }

            self.gt.publish(gt)

    def close(self) -> None:
        for t in self.transports:
            t.close()

        self.camera_bridge.close()
        self.imu_bridge.close()
        self.joint_bridge.close()
        self.cmd_receiver.close()
