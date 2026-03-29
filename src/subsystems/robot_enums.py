__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import IntEnum

# IntEnum is used instead of Enum so that mission-specific submodules (e.g. pragyaan_robot_enums)
# can define their own enum classes with the same members and still compare equal across boundaries.
# Plain Enum instances from different classes are never equal, even with identical names/values.


class GoNogoState(IntEnum):
    NOGO = 0
    GO = 1
    # UNDEF = -1

class ObcState(IntEnum):
    OFF = 0
    BOOT = 1
    IDLE = 2
    CAMERA = 3
    MOTOR = 4
    SAFE = 5
    ERROR = 6

class SolarPanelState(IntEnum):
    STOWED = 0
    DEPLOYED = 1