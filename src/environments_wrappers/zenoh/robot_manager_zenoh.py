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
import msgspec
import numpy as np
import json

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

        self.transports = []
        self.cams = {}
        for robot in RM_conf["parameters"]:
            ### TODO: each robot should have multiple cameras
            cam_pub = ZenohPubTransport(
                keyexpr = f'{zenoh_conf["sensors"]["camera"]["base_keyexpr"]}/{robot["camera"]["name"]}',
                json_compact = zenoh_conf["sensors"]["camera"]["json_compact"],
            )
            self.cams[f'/{robot["robot_name"]}'] = cam_pub
            self.transports.append(cam_pub)
        
        self.resolution = zenoh_conf["sensors"]["camera"]["resolution"]

        self.transports_inited = False
        
    
    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """
        self.clear_modifications()
        self.reset_robots()
    
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

    def reset_robots(self) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def publish_cameras(self) -> None:
        """
        Publish current frame from each camera
        """
        if self.transports_inited:
            for i, robot_name in enumerate(self.RM.robots.keys()):
                frame = self.RM.robots[robot_name].get_rgba_camera_view(self.resolution)
                if frame.size!=0:
                    encoded = self.encode_image(frame)
                    self.cams[robot_name].publish(encoded)

    def encode_image(self, im):
        encoded = ImageStruct.pack(im)
        encoded = json.loads(msgspec.json.encode(encoded))
        return encoded

## TODO: move this ImageStruct to new publish_image() in omnilrs-artefacts 
class ImageStruct(msgspec.Struct, array_like=True, kw_only=True):
    # ref: https://github.com/jcrist/msgspec/issues/732
    dtype: str
    shape: tuple[int, ...]
    data: memoryview

    @classmethod
    def pack(cls, arr: np.ndarray):
        return cls(data=arr.data, dtype=str(arr.dtype), shape=arr.shape)
    
    def unpack(self) -> np.ndarray:
        return np.frombuffer(self.data, dtype=self.dtype).reshape(self.shape)