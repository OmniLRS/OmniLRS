__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple

from src.configurations.simulator_mode_enum import SimulatorMode
from src.robots.robot import RobotManager

#TODO 3rd March 2026 (for after Version 3)
# implemented to resemble the same structure as used for ROS
# might be unnecessary if we do not want to control the environment during runtim same way as ROS wrappers do
class Yamcs_RobotManager():
    """
    Yamcs wrapper that manages the robots.
    """

    def __init__(self, RM_conf: dict, yamcs_instance_conf:dict) -> None:
        self.RM = RobotManager(RM_conf, mode=SimulatorMode.YAMCS, yamcs_instance_conf=yamcs_instance_conf)

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
