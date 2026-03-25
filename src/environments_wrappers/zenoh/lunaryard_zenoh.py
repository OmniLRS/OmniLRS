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

#from asyncio_for_robotics.zenoh.session import auto_session
from asyncio_for_robotics.zenoh.sub import Sub
import zenoh


class Zenoh_LunaryardManager(Zenoh_BaseManager):
    """
    Wrapper for managing environment in Zenoh mode
    """

    def __init__(
        self,
        environment_cfg: dict = None,
        zenoh_cfg: dict = None,
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

        self.sub = Sub(zenoh_cfg["misc"]["rocks"]["randomize"]["keyexpr"])

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

    def randomize_rocks(self, sample: zenoh.Sample):
        data = int(sample.payload.to_string())
        assert data > 0, "The number of rocks must be greater than 0."
        self.modifications.append([self.LC.randomize_rocks, {"num": data}])
        self.trigger_reset = True