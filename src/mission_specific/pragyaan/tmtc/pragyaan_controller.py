__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


from src.mission_specific.pragyaan.tmtc.camera_handler import PragyaanCameraHandler
from src.mission_specific.pragyaan.tmtc.commander import PragyaanCommander
from src.mission_specific.pragyaan.tmtc.enums import STREAMING_FIELDS
from src.mission_specific.pragyaan.tmtc.payload_handler import PayloadHandler
from src.mission_specific.pragyaan.tmtc.transmitter import PragyaanTransmitter
from src.tmtc.yamcs_TMTC import YamcsTMTC


class PragyaanController(YamcsTMTC):
    """
    Implementation of controller for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover controllers.

    Rover controller inherits YamcsTMTC to benefit from the pre-implemented helping handlers that come with the YamcsTMTC framework.
    Implements two mandatory funtions inherited from YamcsTMTC:
        command_handlers - maps each command of the controller yaml `commands` (by key) to its handler and the
                           argument names the handler takes; YamcsTMTC registers them and checks them against the
                           Yamcs server MDB at startup
        start_streaming_data - utilizes IntervalsHandler to setup and trigger update of rover's parameters in Yamcs

    The rest of the functions are implementations of specific desired behaviour of the Pragyaan rover.
    Implementations of controllers for other rovers should follow the same architecture, and separation of high level YamcsTMTC concepts, and
    functionalities of the specific rover in question.

    Besides utilizing handlers provided by YamcsTMTC, the controller implements its own specific handlers that adhere to rover's use case.
    """

    CAMERA_STREAMING_FIELDS = STREAMING_FIELDS

    def __init__(self, yamcs_instance_conf, yamcs_conf, robot_name, robot_RG, robot):
        super().__init__(yamcs_instance_conf, yamcs_conf, robot_name, robot_RG, robot)
        self._intervals = yamcs_conf["intervals"]
        self._payload_handler: PayloadHandler = PayloadHandler(self._images_handler, self._yamcs_conf["payload"])
        self._camera_handler = PragyaanCameraHandler(
            self._images_handler,
            robot,
            yamcs_conf["lander_camera"],
            streaming_interval=self._intervals["camera_streaming"],
            compression_conf=yamcs_conf["camera_compression"],
        )
        self._transmitter: PragyaanTransmitter = PragyaanTransmitter(
            self.transmit_to_yamcs,
            self._intervals_handler,
            self._robot,
            self._robot_RG,
            robot_name,
            self._yamcs_conf["parameters"],
            self._camera_handler,
        )
        self._commander: PragyaanCommander = PragyaanCommander(
            self._robot,
            self._intervals,
            self._payload_handler,
            self._camera_handler,
            self._transmitter,
            self._drive_handler,
            self._obc_handler,
            self._intervals_handler,
        )

    "Performs mapping between yamcs commands and functions that affect the simulation and rover's state"

    def command_handlers(self):
        # controller yaml `commands` key -> (handler, MDB argument names passed to it, in call order)
        return {
            "go_nogo": (self._commander.handle_go_nogo, ["decision"]),
            "power_electronics": (self._commander.handle_electronics_on_off, ["subsystem_id", "power_state"]),
            "solar_panel": (self._commander.handle_solar_panel, ["deployment"]),
            "drive_straight": (self._commander.drive_straight, ["linear_velocity", "distance"]),
            "drive_turn": (self._commander.drive_turn, ["angular_velocity", "angle"]),
            "camera_capture_high": (self._commander.handle_high_res_capture, ["camera_name"]),
            "camera_streaming_on_off": (self._commander.set_activity_of_camera_streaming, ["camera_name", "action"]),
            "camera_capture_depth": (self._commander.handle_depth_capture, ["camera_name"]),
            "set_camera_streaming_params": (
                self._commander.set_camera_streaming_params,
                ["camera_name", "bitrate", "crf", "keyframe_interval", "max_payload_size", "bandwidth_window_seconds"],
            ),
            "capture_apxs": (self._commander.snap_apxs, []),
            "admin_battery_percentage": (self._commander.handle_battery_perc_change, ["battery_percentage"]),
            "admin_water_detection": (self._commander.set_is_near_water, ["trigger_water_detection"]),
            "admin_inject_fault": (self._commander.inject_fault, []),
            "lander_camera_capture_high": (self._commander.handle_lander_camera_capture, []),
        }

    "Creates intervals for automatic transmission of rover's parameters to yamcs"

    def start_streaming_data(self):
        self._payload_handler.snap_apxs()  # snaps initial blank apxs reading
        self._camera_handler.transmit_no_data_images()
        self._intervals_handler.add_new_interval(
            name="Pose of base link",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_pose_of_base_link,
        )
        self._intervals_handler.add_new_interval(
            name="camera streaming state",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_camera_streaming_state,
        )
        self._intervals_handler.add_new_interval(
            name="Camera compression",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_camera_compression_info,
        )
        self._intervals_handler.add_new_interval(
            name="GO_NOGO",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_go_nogo,
        )
        self._intervals_handler.add_new_interval(
            name="IMU readings",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_imu_readings,
        )
        self._intervals_handler.add_new_interval(
            name="OBC state",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_obc_state,
        )
        self._intervals_handler.add_new_interval(
            name="Radio rssi",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_radio_signal_info,
        )
        self._intervals_handler.add_new_interval(
            name="Thermal info",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_thermal_info,
            f_args=[self._intervals["robot_stats"]],
        )
        self._intervals_handler.add_new_interval(
            name="Power status",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_power_info,
            f_args=[self._intervals["robot_stats"]],
        )
        self._intervals_handler.add_new_interval(
            name="OBC metrics",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_obc_metrics,
        )
        self._intervals_handler.add_new_interval(
            name="Solar panel state",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._update_and_transmit_solar_panel_state,
        )
        self._intervals_handler.add_new_interval(
            name="Monitoring camera stream",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._camera_handler.transmit_monitoring_camera_view,
        )
        self._intervals_handler.add_new_interval(
            name="Motor encoder",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmitter.transmit_wheels_joint_angles,
        )
        # here add further intervals and their functionalities

    def _update_and_transmit_solar_panel_state(self):
        self._commander.update_solar_panel_state()
        self._transmitter.transmit_solar_panel_state()
