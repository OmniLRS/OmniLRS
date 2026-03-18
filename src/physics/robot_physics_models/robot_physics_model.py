__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC, abstractmethod

class RobotPhysicsModel(ABC):

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update_inputs(self):
        pass

    @abstractmethod
    def step(self):
        pass
    
    @abstractmethod
    def get_output(self):
        pass