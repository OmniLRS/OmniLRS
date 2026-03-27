__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple

### temporary until omnilrs_artefacts is public through pip install
import os
import sys
module_path = os.path.abspath(f"{os.path.dirname(__file__)}/../../../external/omnilrs_artefacts/src")
sys.path.append(module_path)
from omnilrs_artefacts.transport.zenoh_pub import ZenohPubTransport


class Zenoh_BaseManager():
    def __init__(
        self,
        environment_cfg: dict = None,
        zenoh_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the Zenoh environment manager.

        Args:
            environment_cfg (dict): Environment configuration.
            flares_cfg (dict): Flares configuration.
            **kwargs: Additional arguments.
        """

        self.trigger_reset = False

        self.modifications: List[Tuple[callable, dict]] = []

        self.transports = []

        self.sim_running_pub = ZenohPubTransport(
            keyexpr = zenoh_cfg["misc"]["sim"]["is_running_keyexpr"],
            json_compact = zenoh_cfg["misc"]["sim"]["json_compact"]
        )
        self.transports.append(self.sim_running_pub)

        self.transports_inited = False

    
    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step
        """

        raise NotImplementedError
    
    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """

        raise NotImplementedError

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab
        """

        self.modifications: List[Tuple[callable, dict]] = []
    
    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()
    
    def pub_sim_is_running(self, is_running: bool) -> None:
        """
        Publish to Zenoh keyexpr to let subscribers know that the simulation is running
        """
        if self.transports_inited:
            self.sim_running_pub.publish({"is_running": is_running})