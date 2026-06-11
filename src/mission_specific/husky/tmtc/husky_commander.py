__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import omni.kit.app

from src.mission_specific.husky.tmtc.enums import HuskyYamcsArguments, HuskyCameraResolution
from src.mission_specific.husky.tmtc.camera_handler import CameraViewType, HuskyCameraHandler
from src.mission_specific.husky.tmtc.transmitter import HuskyTransmitter
from src.mission_specific.husky.subsystems.husky_robot_enums import ObcState
from src.tmtc.intervals_handler import IntervalName


class HuskyCommander:
    """
    Commander for the Husky UGV.

    Contains the rover-specific logic behind each Yamcs command.  Methods here
    are registered as callbacks inside HuskyController.setup_command_callbacks().

    Husky-specific capabilities vs. Pragyaan:
      * Differential-drive only (drive_straight / drive_turn).
      * Onboard camera (RGBA + depth capture, streaming on/off).
      * No solar panel, no GO/NOGO gate.
    """

    def __init__(
        self,
        robot,
        intervals,
        camera_handler: HuskyCameraHandler,
        transmitter: HuskyTransmitter,
        drive_handler,
        obc_handler,
        intervals_handler,
    ):
        self._robot = robot
        self._intervals = intervals
        self._camera_handler = camera_handler
        self._transmitter = transmitter
        self._drive_handler = drive_handler
        self._obc_handler = obc_handler
        self._intervals_handler = intervals_handler

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def drive_straight(self, linear_velocity, distance):
        self._drive_handler.drive_robot_straight(linear_velocity, distance)

    def drive_turn(self, angular_velocity, angle):
        self._drive_handler.drive_robot_turn(angular_velocity, angle)

    # ------------------------------------------------------------------
    # Camera commands
    # ------------------------------------------------------------------

    def handle_high_res_capture(self):
        self._camera_handler.transmit_camera_view(
            HuskyCameraHandler.BUCKET_ONCOMMAND,
            HuskyCameraResolution.HIGH.value,
            CameraViewType.RGBA,
        )
        self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self):
        self._camera_handler.transmit_camera_view(
            HuskyCameraHandler.BUCKET_DEPTH,
            HuskyCameraResolution.HIGH.value,
            CameraViewType.DEPTH,
        )
        self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def set_activity_of_camera_streaming(self, action: str):
        if action == HuskyYamcsArguments.STOP.value:
            self._intervals_handler.remove_interval(IntervalName.CAMERA_STREAMING.value)
        elif action == HuskyYamcsArguments.START.value:
            if not self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value):
                self._intervals_handler.add_new_interval(
                    name=IntervalName.CAMERA_STREAMING.value,
                    seconds=self._intervals["camera_streaming"],
                    is_repeating=True,
                    execute_immediately=False,
                    function=self._camera_handler.transmit_camera_view,
                    f_args=(
                        HuskyCameraHandler.BUCKET_STREAMING,
                        HuskyCameraResolution.LOW.value,
                    ),
                )
        else:
            print("HuskyCommander.set_activity_of_camera_streaming: unknown action:", action)

