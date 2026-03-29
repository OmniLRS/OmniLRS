from enum import Enum

class PragyaanYamcsArguments(Enum):
    DEPLOY = "DEPLOY"
    STOW = "STOW"
    START = "START"
    STOP = "STOP"

class PragyaanCameraResolution(Enum):
    # depend on the yaml conf file
    LOW = "low"
    HIGH = "high"