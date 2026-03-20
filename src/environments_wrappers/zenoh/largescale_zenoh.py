__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.large_scale_lunar import LargeScaleController
from src.environments_wrappers.zenoh.base_wrapper_zenoh import Zenoh_BaseManager


class Zenoh_LargeScaleManager(Zenoh_BaseManager):
    """
    Wrapper for managing the environment in Zenoh mode
    """

    def __init__(
        self,
        environment_cfg: dict = None,
        is_simulation_alive: callable = lambda: True,
        close_simulation: callable = lambda: None,
        **kwargs,
    ) -> None:
        """
        Initializes the environment manager.

        Args:
            environment_cfg (dict): Environment configuration.
            is_simulation_alive (callable): function to check if the simulation is alive.
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        
        self.LC = LargeScaleController(
            mode=SimulatorMode.ZENOH, **environment_cfg,
            is_simulation_alive=is_simulation_alive,
            close_simulation=close_simulation
        )
        self.LC.load()
    
    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        self.modifications.append([self.LC.update_stellar_engine, {"dt": dt}])
        self.LC.update()
    
    def reset(self) -> None:
        """
        Resets the lab to its initial state
        """
        pass