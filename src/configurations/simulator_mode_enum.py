__author__ = "Aleksa Stanivuk, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import Enum


class SimulatorMode(Enum):
    ROS2 = 1
    YAMCS = 2
    SDG = 3
    ZENOH = 4
