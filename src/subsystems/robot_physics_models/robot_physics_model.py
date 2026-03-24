__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC, abstractmethod

class RobotPhysicsModel(ABC):

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def set_inputs(self):
        pass

    @abstractmethod
    def compute(self):
        pass

    @abstractmethod
    def get_outputs(self):
        pass