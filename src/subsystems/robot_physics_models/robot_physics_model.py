__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC, abstractmethod
from typing import Any, Dict


class RobotPhysicsModel(ABC):
    """Base contract for robot subsystem physics models.

    Every model implements the same four lifecycle methods.
    Arguments for initialize() and set_inputs() are intentionally model-specific,
    so they accept arbitrary positional/keyword arguments;
    subclasses define the concrete signature that suits their physics.
    """

    @abstractmethod
    def initialize(self, *args, **kwargs) -> None:
        """Set up internal state. Called once before the first `set_inputs()` and `compute()` calls.

        Subclasses may declare model-specific configuration arguments here.
        """
        pass

    @abstractmethod
    def set_inputs(self, *args, **kwargs) -> None:
        """Set the inputs needed by the next `compute` step."""
        pass

    @abstractmethod
    def compute(self, dt: float) -> None:
        """Advance the model state by `dt` seconds using the stored inputs."""
        pass

    @abstractmethod
    def get_outputs(self) -> Dict[str, Any]:
        """Return the current model outputs (e.g. sensor readings, telemetry)."""
        pass
