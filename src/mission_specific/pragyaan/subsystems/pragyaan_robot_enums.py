__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

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
