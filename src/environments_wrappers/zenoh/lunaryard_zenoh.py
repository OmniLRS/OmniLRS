__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.lunaryard import LunaryardController

from src.environments_wrappers.zenoh.base_wrapper_zenoh import Zenoh_BaseManager


class Zenoh_LunaryardManager(Zenoh_BaseManager):
    """
    Wrapper for managing environment in Zenoh mode
    """

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
        self.LC = LunaryardController(mode=SimulatorMode.ZENOH, **environment_cfg)
        self.LC.load()
    
    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        self.modifications.append([self.LC.update_stellar_engine, {"dt": dt}])

    def reset(self) -> None:
        """
        Resets the lab to its initial state
        """
        pass