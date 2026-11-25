__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


from src.mission_specific.pragyaan.tmtc.camera_handler import PragyaanCameraHandler
from src.mission_specific.pragyaan.tmtc.commander import PragyaanCommander
from src.mission_specific.pragyaan.tmtc.payload_handler import PayloadHandler
from src.mission_specific.pragyaan.tmtc.pragyaan_tm_collector import PragyaanTMCollector
from src.tmtc.yamcs_TMTC import YamcsTMTC


class PragyaanController(YamcsTMTC):
    """
    Implementation of controller for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover controllers.

    Rover controller inherits YamcsTMTC to benefit from the pre-implemented helping handlers that come with the YamcsTMTC framework.
    Implements two mandatory funtions inherited from YamcsTMTC:
        _setup_command_callbacks(commands_conf) - utilizes CommandsHandler to construct the mapping between high level Yamcs commands,
                                                        and desired functionalities
                                                - commands_conf are previously defined inside .yaml configuration file
        start_transmitting_TM - utilizes IntervalsHandler to periodically transmit the parameters collected inside
                                PragyaanTMCollector to Yamcs

    The rest of the functions are implementations of specific desired behaviour of the Pragyaan rover.
    Implementations of controllers for other rovers should follow the same architecture, and separation of high level YamcsTMTC concepts, and
    functionalities of the specific rover in question.

    Besides utilizing handlers provided by YamcsTMTC, the controller implements its own specific handlers that adhere to rover's use case.
    """

    def __init__(self, yamcs_instance_conf, yamcs_conf, robot_name, robot_RG, robot):
        super().__init__(yamcs_instance_conf, yamcs_conf, robot_name, robot_RG, robot)
        self._intervals = yamcs_conf["intervals"]
        self._payload_handler: PayloadHandler = PayloadHandler(self._images_handler, self._yamcs_conf["payload"])
        self._camera_handler = PragyaanCameraHandler(self._images_handler, robot, yamcs_conf["lander_camera"])
        self._tm_collector: PragyaanTMCollector = PragyaanTMCollector(
            self._intervals_handler,
            self._robot,
            self._robot_RG,
            robot_name,
            self._yamcs_conf["parameters"],
            self._intervals,
        )
        self._tm_collector.start_collecting_TM()
        self._commander: PragyaanCommander = PragyaanCommander(
            self._robot,
            self._intervals,
            self._payload_handler,
            self._camera_handler,
            self._tm_collector,
            self._drive_handler,
            self._obc_handler,
            self._intervals_handler,
        )

    "Performs mapping between yamcs commands and functions that affect the simulation and rover's state"

    def setup_command_callbacks(self, commands_conf):
        self._commands_handler.add_command(
            commands_conf["drive_straight"], self._commander.drive_straight, args=["linear_velocity", "distance"]
        )
        self._commands_handler.add_command(
            commands_conf["drive_turn"], self._commander.drive_turn, args=["angular_velocity", "angle"]
        )
        self._commands_handler.add_command(
            commands_conf["camera_capture_high"], self._commander.handle_high_res_capture
        )
        self._commands_handler.add_command(
            commands_conf["camera_streaming_on_off"], self._commander.set_activity_of_camera_streaming, args=["action"]
        )
        self._commands_handler.add_command(commands_conf["camera_capture_depth"], self._commander.handle_depth_capture)
        self._commands_handler.add_command(
            commands_conf["power_electronics"],
            self._commander.handle_electronics_on_off,
            args=["subsystem_id", "power_state"],
        )
        self._commands_handler.add_command(
            commands_conf["solar_panel"], self._commander.handle_solar_panel, args=["deployment"]
        )
        self._commands_handler.add_command(commands_conf["go_nogo"], self._commander.handle_go_nogo, args=["decision"])
        self._commands_handler.add_command(commands_conf["capture_apxs"], self._commander.snap_apxs)
        self._commands_handler.add_command(
            commands_conf["admin_water_detection"], self._commander.set_is_near_water, args=["trigger_water_detection"]
        )
        self._commands_handler.add_command(commands_conf["admin_inject_fault"], self._commander.inject_fault)
        self._commands_handler.add_command(
            commands_conf["lander_camera_capture_high"], self._commander.handle_lander_camera_capture
        )
        self._commands_handler.add_command(
            commands_conf["admin_battery_percentage"],
            self._commander.handle_battery_perc_change,
            args=["battery_percentage"],
        )

    "Creates intervals for automatic transmission of rover's collected parameters to yamcs"

    def start_transmitting_TM(self):
        self._payload_handler.snap_apxs()  # snaps initial blank apxs reading
        self._images_handler.snap_no_data_images()
        self._intervals_handler.add_new_interval(
            name="Monitoring camera stream",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._camera_handler.transmit_monitoring_camera_view,
        )
        self._intervals_handler.add_new_interval(
            name="TM transmission",
            seconds=self._intervals["tm_transmission"],
            is_repeating=True,
            execute_immediately=True,
            function=self._transmit_collected_TM,
        )
        # here add further intervals and their functionalities

    def _transmit_collected_TM(self):
        # TM is already being collected by self._tm_collector (see start_collecting_TM, called in __init__)
        param_values = {
            param_name: entry.value
            for param_name, entry in self._tm_collector.take_snapshot().items()
            if entry.value is not None  # never collected, and cannot be encoded by Yamcs
        }
        if param_values:
            self.transmit_to_yamcs_in_batch(param_values)
