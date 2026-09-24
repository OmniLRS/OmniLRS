__author__ = "Shamistan Karimov, Elian NEPPEL, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

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

        self.publishers: List[ZenohPubTransport] = []
        self.cams: List[ZenohPubTransport] = []

        robot = RM_conf["parameters"]

        robot_name = robot["robot_name"]
        robot_path = self.RM_.robots_root + "/" + robot_name

        ### BEGIN TELEMETRY ###

        ## Camera Telemetry
        self.camera_bridge = CameraBridge(robot, zenoh_conf, self.RM_)

        ## IMU Telemetry
        self.imu_bridge = IMUBridge(robot, zenoh_conf, self.RM_)

        ## Joint Force Telemetry
        self.joint_bridge = JointForceBridge(
            robot,
            zenoh_conf,
            RM=self.RM_,
            robot_root_prim=robot_path,
        )
        ### END TELEMETRY ###

        ## Joint Commands
        self.cmd_receiver = ZenohCommandReceiver(
            RM=self.RM_,
            keyexpr=zenoh_conf["keyexprs"]["joint_commands"].format(robot_name=robot_name),
            wire_format=robot["zenoh"]["joint_commands"].get(
                "wire_format",
                zenoh_conf["default_wire_format"],
            ),
            is_logging=robot["zenoh"]["joint_commands"]["is_logging"],
            log_every_n=zenoh_conf["sub_log_every_n"],
        )

        ## Ground Truth
        gt_pub = ZenohPubTransport(
            keyexpr=zenoh_conf["keyexprs"]["ground_truth_pose"].format(robot_name=robot_name),
            wire_format=robot["zenoh"]["ground_truth_pose"].get(
                "wire_format",
                zenoh_conf["default_wire_format"],
            ),
            is_logging=robot["zenoh"]["ground_truth_pose"]["is_logging"],
            log_every_n=zenoh_conf["pub_log_every_n"],
        )
        self.gt = gt_pub
        self.publishers.append(gt_pub)

        self.pubs_inited = False

    def start_publishing(self) -> None:
        """
        Start publishers.
        """
        for pub in self.publishers:
            pub.start()

        self.pubs_inited = True

    def start_listening(self) -> None:
        """
        Start subscribers.
        """
        self.cmd_receiver.start()

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

    def publish_gt(self) -> None:
        if self.pubs_inited:
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
        for pub in self.publishers:
            pub.close()

        self.camera_bridge.close()
        self.imu_bridge.close()
        self.joint_bridge.close()
        self.cmd_receiver.close()
