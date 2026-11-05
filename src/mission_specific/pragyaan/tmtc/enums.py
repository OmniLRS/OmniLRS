from enum import StrEnum


class PragyaanYamcsArguments(StrEnum):
    DEPLOY = "DEPLOY"
    STOW = "STOW"
    START = "START"
    STOP = "STOP"


class PragyaanCameraResolution(StrEnum):
    # must match the resolution keys in pragyaan.yaml
    LOW = "low"
    HIGH = "high"


# Resolution of each image kind: every pragyaan.yaml camera must have the resolutions of its kinds (checked by
# PragyaanCameraHandler at startup and by test/test_yamcs_naming.py). The Yamcs image mocks load it by file path.
KIND_RESOLUTION = {
    "streaming": PragyaanCameraResolution.LOW,
    "oncommand": PragyaanCameraResolution.HIGH,
    "depth": PragyaanCameraResolution.HIGH,
}

# Streaming telemetry PragyaanTransmitter sends per rgba camera, under /Rover/camera/<camera>/streaming/<field>
# (yamcs_naming.streaming_param). The Yamcs MDB generator loads this file by path to define them.
STREAMING_FIELDS = ("state", "bandwidth", "packets_sent", "packets_lost", "decode_errors")
