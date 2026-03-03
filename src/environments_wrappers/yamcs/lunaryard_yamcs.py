# Custom libs
from src.environments.lunaryard import LunaryardController

from src.environments_wrappers.yamcs.base_wrapper_yamcs import Yamcs_BaseManager

#NOTE removed ROS-dependent imports and function implementations

#TODO 3rd March 2026 (for after Version 3)
# implemented to resemble the same structure as used for ROS
# might be unnecessary if we do not want to control the environment during runtim same way as ROS wrappers do
class Yamcs_LunaryardManager(Yamcs_BaseManager):
    """
    Wrapper for managing environment in Yamcs mode"""

    def __init__(
        self,
        environment_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lunaryard manager.

        Args:
            environment_cfg (dict): Environment configuration.
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        #TODO Should self.LC be renamed into EC (as 'Environment' Controller)
        # because LC was probably named when Lunalab was the only environment
        # this naming still 'works' because all environments start with an L
        # but that might not be the case forever
        self.LC = LunaryardController(**environment_cfg)
        self.LC.load()

        #NOTE Yamcs command subscriptions would go here
        # if ROS2 architecture is to be followed

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        self.modifications.append([self.LC.update_stellar_engine, {"dt": dt}])

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass
