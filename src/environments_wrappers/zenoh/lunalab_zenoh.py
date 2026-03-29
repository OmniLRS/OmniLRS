__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import zenoh

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.lunalab import LunalabController
from src.environments_wrappers.zenoh.base_wrapper_zenoh import Zenoh_BaseManager


class Zenoh_LunalabManager(Zenoh_BaseManager):
    def __init__(
        self,
        environment_cfg: dict = None,
        zenoh_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab manager.

        Args:
            environment_cfg (dict): Environment configuration.
            **kwargs: Additional arguments
        """

        super().__init__(environment_cfg=environment_cfg, zenoh_cfg=zenoh_cfg, **kwargs)

        self.LC = LunalabController(mode=SimulatorMode.ZENOH, **environment_cfg)
        self.LC.load()

        self.rocks_randomize_keyexpr = zenoh_cfg["misc"]["rocks"]["randomize"]["keyexpr"]
        self.trigger_reset = False
    
    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """
        pass

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """
        pass

    def randomize_rocks(self, sample: zenoh.Sample):
        data = int(sample.payload.to_string())
        assert data > 0, "The number of rocks must be greater than 0."
        self.modifications.append([self.LC.randomize_rocks, {"num": data}])
        self.trigger_reset = True