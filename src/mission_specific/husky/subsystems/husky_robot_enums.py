__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"

from enum import IntEnum

# Husky-specific overrides of robot enums (see src/subsystems/robot_enums.py for defaults).
# Modify values here to tune Husky's subsystem behavior.


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