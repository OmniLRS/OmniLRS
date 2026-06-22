__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"
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