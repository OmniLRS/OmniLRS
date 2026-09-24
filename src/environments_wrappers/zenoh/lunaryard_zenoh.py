__author__ = "Shamistan Karimov, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import asyncio
import logging
from asyncio import Task
from typing import List

import asyncio_for_robotics.zenoh as afor

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.lunaryard import LunaryardController
from src.environments_wrappers.zenoh.base_wrapper_zenoh import Zenoh_BaseManager

logger = logging.getLogger(__name__)


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

        super().__init__(environment_cfg=environment_cfg, zenoh_cfg=zenoh_cfg, **kwargs)
        self.LC = LunaryardController(mode=SimulatorMode.ZENOH, **environment_cfg)
        self.LC.load()

        self.zenoh_cfg = zenoh_cfg

        self.rocks_randomize_keyexpr = self.zenoh_cfg["keyexprs"]["randomize_rocks"]

        self.log = logger.info

        self.subscribers: List[Task] = []

        self.trigger_reset = False

    def start_listening(self) -> None:
        """
        Re-implement Zenoh_BaseManager's start_listening template.
        """
        if self.subs_inited:
            return

        afor.auto_session()

        randomize_rocks_task = asyncio.ensure_future(self._randomize_rocks_sub())

        self.subscribers.append(randomize_rocks_task)

        # more tasks can be appended here:
        #   self.subscribers.append(task..)
        #   ...

        self.subs_inited = True

        self.log(
            f"[ZenohLunaryardManager] listening: {self.rocks_randomize_keyexpr} wire_format={self.zenoh_cfg['default_wire_format']}"
        )

    async def _randomize_rocks_sub(self) -> None:
        sub = afor.Sub(self.rocks_randomize_keyexpr)

        try:
            async for sample in sub.listen_reliable():
                self.log("[ZenohLunaryardManager] received cmd: randomize_rocks")

                data = int(sample.payload.to_string())
                assert data > 0, "The number of rocks must be greater than 0."
                self.modifications.append([self.LC.randomize_rocks, {"num": data}])
                self.trigger_reset = True

        finally:
            sub.close()

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

    def close(self) -> None:
        for sub in self.subscribers:
            if sub is not None:
                sub.cancel()

        self.subscribers.clear()
        self.subs_inited = False
        super().close()
