__author__ = "Amaan Javed"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


from functools import partial

from src.mission_specific.husky.tmtc.camera_handler import HuskyCameraHandler
from src.mission_specific.husky.tmtc.enums import HuskyYamcsArguments
from src.mission_specific.husky.tmtc.transmitter import HuskyTransmitter
from src.subsystems.robot_enums import ObcState
from src.tmtc.intervals_handler import camera_streaming_interval


class HuskyCommander:
    """
    Commander for the Husky UGV.

    Contains the rover-specific logic behind each Yamcs command. Methods here
    are mapped to their commands in HuskyController.command_handlers().

    Husky-specific capabilities vs. Pragyaan:
      * Differential-drive only (drive_straight / drive_turn).
      * Onboard cameras (robot yaml `camera` list): on-command and streaming RGB images per rgba camera, depth
        images per depth camera. Camera commands take a camera_name argument.
      * No solar panel, no GO/NOGO gate, no camera power command, no streaming compression model.
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

    # YamcsTMTC only calls them with a camera_name of the command's camera type (commands_handler.check_camera_argument)

    def handle_high_res_capture(self, camera_name: str):
        self._camera_handler.transmit_camera_view(camera_name, kind="oncommand")
        self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self, camera_name: str):
        self._camera_handler.transmit_camera_view(camera_name, kind="depth")
        self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def set_activity_of_camera_streaming(self, camera_name: str, action: str):
        interval_name = camera_streaming_interval(camera_name)
        if action == HuskyYamcsArguments.STOP.value:
            self._intervals_handler.remove_interval(interval_name)
        elif action == HuskyYamcsArguments.START.value:
            if not self._intervals_handler.does_exist(interval_name):
                self._intervals_handler.add_new_interval(
                    name=interval_name,
                    seconds=self._intervals["camera_streaming"],
                    is_repeating=True,
                    execute_immediately=False,
                    function=partial(self._camera_handler.transmit_camera_view, camera_name, kind="streaming"),
                )
        else:
            print("HuskyCommander.set_activity_of_camera_streaming: unknown action:", action)
