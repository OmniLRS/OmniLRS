__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple
import os
import sys

from src.configurations.simulator_mode_enum import SimulatorMode
from src.robots.robot import RobotManager

module_path = os.path.abspath(f"{os.path.dirname(__file__)}/../../../external/omnilrs_artefacts/src")
sys.path.append(module_path)
from omnilrs_artefacts.transport.zenoh_pub import ZenohPubTransport


class Zenoh_RobotManager():
    """
    Zenoh wrapper that manages the robots.
    """

    def __init__(self, RM_conf: dict, zenoh_conf: dict) -> None:
        self.RM = RobotManager(RM_conf, mode=SimulatorMode.ZENOH)

        self.modifications: List[Tuple[callable, dict]] = []

        self.robots = []
        self.cam_pubs = []        
        for robot in RM_conf["parameters"]:
            ### TODO: each robot should have multiple cameras
            cam_pub = ZenohPubTransport(
                keyexpr = f"{zenoh_conf["sensors"]["camera"]["base_keyexpr"]}/{robot["camera"]["name"]}",
                json_compact = zenoh_conf["sensors"]["camera"]["json_compact"],
            )
            self.cam_pubs.append(cam_pub)
            self.robots.append(self.RM.robots[robot["robot_name"]])
        
        self.resolution = zenoh_conf["sensors"]["camera"]["resolution"]

        self.transports = self.cam_pubs # + others

        self.inited = False
        
    
    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """
        self.clear_modifications()
    
    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """
        self.modifications: List[Tuple[callable, dict]] = []
    
    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """
        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def publish_cameras(self) -> None:
        """
        Publish current frame from each camera
        """
        for i, robot in enumerate(self.robots):
            frame = robot.get_rgba_camera_view(self.resolution)
            self.cam_pubs[i].publish(f"heyy {i}")