__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import Enum

class SimulatorMode(Enum):
    ROS2 = 1
    YAMCS = 2
    SDG = 3
    ZENOH = 4
