__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# Custom libs
from src.environments.lunalab import LunalabController

# Loads ROS2 dependent libraries
from src.environments_wrappers.yamcs.base_wrapper_yamcs import Yamcs_BaseManager

#NOTE removed ROS-dependent imports and function implementations

#TODO 3rd March 2026 (for after Version 3)
# implemented to resemble the same structure as used for ROS
# might be unnecessary if we do not want to control the environment during runtim same way as ROS wrappers do
class Yamcs_LunalabManager(Yamcs_BaseManager):
    """
    Wrapper for managing environment in Yamcs mode"""

    def __init__(
        self,
        environment_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab manager.

        Args:
            environment_cfg (dict): Environment configuration.
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        #TODO Should self.LC be renamed into EC (as 'Environment' Controller)
        # because LC was probably named when Lunalab was the only environment
        # this naming still 'works' because all environments start with an L
        # but that might not be the case forever
        self.LC = LunalabController(**environment_cfg)
        self.LC.load()
        self.trigger_reset = False

        #NOTE Yamcs command subscriptions would go here
        # if ROS2 architecture is to be followed

    def periodic_update(self, dt: float) -> None:
        pass

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass