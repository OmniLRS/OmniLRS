__author__ = "Amaan Javed"
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


# Resolution of each image kind: every husky.yaml camera must have the resolutions of its kinds (checked by
# HuskyCameraHandler at startup and by test/test_yamcs_naming.py). The Yamcs image mocks load it by file path.
KIND_RESOLUTION = {
    "streaming": HuskyCameraResolution.LOW,
    "oncommand": HuskyCameraResolution.HIGH,
    "depth": HuskyCameraResolution.HIGH,
}

# Streaming telemetry HuskyTransmitter sends per rgba camera, under /Rover/camera/<camera>/streaming/<field>
# (yamcs_naming.streaming_param). The Yamcs MDB generator loads this file by path to define them.
STREAMING_FIELDS = ("state",)
