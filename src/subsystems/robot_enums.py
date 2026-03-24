from enum import Enum


class GoNogoState(Enum):
    NOGO = 0
    GO = 1
    # UNDEF = "Other"

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