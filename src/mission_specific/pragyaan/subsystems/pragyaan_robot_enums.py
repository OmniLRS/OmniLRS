__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import Enum

# Pragyaan-specific overrides of robot enums (see src/subsystems/robot_enums.py for defaults).
# Modify values here to tune Pragyaan's subsystem behavior.


class GoNogoState(Enum):
    NOGO = 0
    GO = 1

class ObcState(Enum):
    OFF = 0
    BOOT = 1
    IDLE = 2
    CAMERA = 3
    MOTOR = 4
    SAFE = 5
    ERROR = 6

class SolarPanelState(Enum):
    STOWED = 0
    DEPLOYED = 1

class CpuUsageLevel(Enum):
    LOW = 25.0
    MEDIUM = 50.0
    HIGH = 75.0

class RamUsageLevel(Enum):
    LOW = 40.0
    MEDIUM = 50.0
    HIGH = 60.0

class DiskUsageLevel(Enum):
    LOW = 10.0
    MEDIUM = 25.0
    HIGH = 75.0
