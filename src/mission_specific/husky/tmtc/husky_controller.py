__author__ = "Amaan Javed"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


from src.mission_specific.husky.tmtc.camera_handler import HuskyCameraHandler
from src.mission_specific.husky.tmtc.husky_commander import HuskyCommander
from src.mission_specific.husky.tmtc.husky_drive_handler import HuskyDriveHandler
from src.mission_specific.husky.tmtc.transmitter import HuskyTransmitter
from src.tmtc.yamcs_TMTC import YamcsTMTC


class HuskyController(YamcsTMTC):
    """
    YAMCS controller for the Husky UGV.

    Inherits YamcsTMTC and wires up Husky-specific handlers.

    Implements the two mandatory abstract methods:
        command_handlers()      — maps each command of the controller yaml `commands` (by key) to its HuskyCommander
                                  method and the argument names it takes; YamcsTMTC registers them and checks them
                                  against the Yamcs server MDB at startup.
        start_streaming_data()  — schedules periodic telemetry transmissions.
    """

    def __init__(
        self,
        yamcs_instance_conf,
        yamcs_conf,
        robot_name,
        robot_RG,
        robot,
    ):
        super().__init__(yamcs_instance_conf, yamcs_conf, robot_name, robot_RG, robot)
        self._drive_handler = HuskyDriveHandler(robot, self._intervals_handler, self._obc_handler)

        self._intervals = yamcs_conf["intervals"]

        self._camera_handler = HuskyCameraHandler(self._images_handler, robot)

        self._transmitter = HuskyTransmitter(
            self.transmit_to_yamcs,
            self._intervals_handler,
            robot,
            robot_RG,
            robot_name,
            yamcs_conf["parameters"],
            self._camera_handler,
        )

        self._commander = HuskyCommander(
            robot,
            self._intervals,
            self._camera_handler,
            self._transmitter,
            self._drive_handler,
            self._obc_handler,
            self._intervals_handler,
        )

    def command_handlers(self):
        # controller yaml `commands` key -> (handler, MDB argument names passed to it, in call order)
        return {
            "drive_straight": (self._commander.drive_straight, ["linear_velocity", "distance"]),
            "drive_turn": (self._commander.drive_turn, ["angular_velocity", "angle"]),
            "camera_capture_high": (self._commander.handle_high_res_capture, ["camera_name"]),
            "camera_streaming_on_off": (self._commander.set_activity_of_camera_streaming, ["camera_name", "action"]),
            "camera_capture_depth": (self._commander.handle_depth_capture, ["camera_name"]),
        }

    def start_streaming_data(self):
        """Schedule periodic telemetry intervals."""
        self._images_handler.snap_no_data_images()

        interval = self._intervals["robot_stats"]

        self._intervals_handler.add_new_interval(
            name="Pose of base link",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_pose_of_base_link,
        )
        self._intervals_handler.add_new_interval(
            name="camera streaming state",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_camera_streaming_state,
        )
        self._intervals_handler.add_new_interval(
            name="IMU readings",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_imu_readings,
        )
        self._intervals_handler.add_new_interval(
            name="OBC state",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_obc_state,
        )
        self._intervals_handler.add_new_interval(
            name="Radio rssi",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_radio_signal_info,
        )
        self._intervals_handler.add_new_interval(
            name="Thermal info",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_thermal_info,
            f_args=[interval],
        )
        self._intervals_handler.add_new_interval(
            name="Power status",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_power_info,
            f_args=[interval],
        )
        self._intervals_handler.add_new_interval(
            name="OBC metrics",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_obc_metrics,
        )
        self._intervals_handler.add_new_interval(
            name="Motor encoder",
            seconds=interval,
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_wheels_joint_angles,
        )
