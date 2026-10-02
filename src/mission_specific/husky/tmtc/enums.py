__author__ = "Amaan Javed"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import StrEnum


class HuskyYamcsArguments(StrEnum):
    START = "START"
    STOP = "STOP"


class HuskyCameraResolution(StrEnum):
    # must match the resolution keys in husky.yaml
    LOW = "low"
    HIGH = "high"
