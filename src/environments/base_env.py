__author__ = "Antoine Richard, Junnosuke Kamohara, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import omni
from pxr import Usd

from src.configurations.simulator_mode_enum import SimulatorMode
from src.robots.robot import RobotManager


class BaseEnv:
    """
    This class is used to control the environment's interactive elements.
    """

    def __init__(
        self,
        mode: SimulatorMode = SimulatorMode.ROS2,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.

        Args:
            **kwargs: Arbitrary keyword arguments.
        """
        self._mode: SimulatorMode = mode
        self.stage: Usd.Stage = omni.usd.get_context().get_stage()
        self.SAM = None  # Static Assets Manager
        self.MCM = None  # Monitoring Cameras Manager
        self.robotManager = None  # Set later via add_robot_manager()

    def build_scene(self) -> None:
        """
        Builds the scene. It either loads the scene from a file or creates it from scratch.
        """

        raise NotImplementedError()

    def instantiate_scene(self) -> None:
        """
        Instantiates the scene. Applies any operations that need to be done after the scene is built and
        the renderer has been stepped.
        """

        raise NotImplementedError()

    def reset(self) -> None:
        """
        Resets the environment. Implement the logic to reset the environment.
        """

        raise NotImplementedError()

    def update(self) -> None:
        """
        Updates the environment.
        """

        raise NotImplementedError()

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        raise NotImplementedError()

    def add_robot_manager(self, robotManager: RobotManager) -> None:
        """
        Adds the robot manager to the environment.

        Args:
            robotManager (RobotManager): The robot manager.
        """
        self.robotManager = robotManager
