__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple

from src.environments.simulator_mode_enum import SimulatorMode
from src.robots.robot import RobotManager

#TODO 3rd March 2026 (for after Version 3)
# implemented to resemble the same structure as used for ROS
# might be unnecessary if we do not want to control the environment during runtim same way as ROS wrappers do
class Yamcs_RobotManager():
    """
    Yamcs wrapper that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        self.RM = RobotManager(RM_conf, mode=SimulatorMode.YAMCS)

        #NOTE Yamcs command subscriptions would go here
        # if ROS2 architecture is to be followed

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []

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
