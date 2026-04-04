from enum import StrEnum

class PragyaanYamcsArguments(StrEnum):
    DEPLOY = "DEPLOY"
    STOW = "STOW"
    START = "START"
    STOP = "STOP"

class PragyaanCameraResolution(StrEnum):
    # depend on the yaml conf file
    LOW = "low"
    HIGH = "high"