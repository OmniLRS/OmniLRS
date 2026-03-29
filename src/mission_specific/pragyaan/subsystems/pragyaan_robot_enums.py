__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import IntEnum

# Pragyaan-specific overrides of robot enums (see src/subsystems/robot_enums.py for defaults).
# Modify values here to tune Pragyaan's subsystem behavior.


class GoNogoState(IntEnum):
    NOGO = 0
    GO = 1

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
