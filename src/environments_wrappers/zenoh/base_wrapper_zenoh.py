__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple


class Zenoh_BaseManager():
    def __init__(
        self,
        environment_cfg: dict = None,
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