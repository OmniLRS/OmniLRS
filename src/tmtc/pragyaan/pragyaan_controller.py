__author__ = "Aleksa Stanivuk"
__status__ = "development"

from enum import Enum
from src.robots.subsystems_manager import Electronics, GoNogoState, HealthStatus, ObcState, PowerState, SolarPanelState
from src.tmtc.intervals_handler import IntervalName
from src.tmtc.pragyaan.camera_handler import CameraViewType, PragyaanCameraHandler
from src.tmtc.pragyaan.payload_handler import PayloadHandler
from src.tmtc.pragyaan.transmitter import PragyaanTransmitter
from src.tmtc.yamcs_TMTC import  YamcsTMTC
import omni.kit.app

class YamcsArguments(Enum):
    DEPLOY = "DEPLOY"
    STOW = "STOW"
    START = "START"
    STOP = "STOP"

class CameraResolution(Enum):
    # depend on the yaml conf file
    LOW = "low"
    HIGH = "high"

class PragyaanController(YamcsTMTC):
    """
    Implementation of controller for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover controllers.

    Rover controller inherits YamcsTMTC to benefit from the pre-implemented helping handlers that come with the YamcsTMTC framework.
    Implements two mandatory funtions inherited from YamcsTMTC:
        _setup_command_callbacks(commands_conf) - utilizes CommandsHandler to construct the mapping between high level Yamcs commands, 
                                                        and desired functionalities 
                                                - commands_conf are previously defined inside .yaml configuration file
        start_streaming_data - utilizes IntervalsHandler to setup and trigger update of rover's parameters in Yamcs 

    The rest of the functions are implementations of specific desired behaviour of the Pragyaan rover. 
    Implementations of controllers for other rovers should follow the same architecture, and separation of high level YamcsTMTC concepts, and 
    functionalities of the specific rover in question.

    Besides utilizing handlers provided by YamcsTMTC, the controller implements its own specific handlers that adhere to rover's use case.
    """
    def __init__(self,
        yamcs_conf,
        robot_name,
        robot_RG,
        robot):
        super().__init__(yamcs_conf, robot_name, robot_RG, robot)
        self._intervals = yamcs_conf["intervals"]
        self._payload_handler:PayloadHandler = PayloadHandler(self._images_handler, self._yamcs_conf["payload"])
        self._camera_handler = PragyaanCameraHandler(self._images_handler, robot, yamcs_conf["lander_camera"])
        self._transmitter:PragyaanTransmitter = PragyaanTransmitter(self._yamcs_processor, self._intervals_handler, self._robot, self._robots_RG, robot_name, self._yamcs_conf["parameters"])
        self._setup_command_callbacks(yamcs_conf["commands"])

    "Performs mapping between yamcs commands and functions that affect the simulation and rover's state"
    def _setup_command_callbacks(self, commands_conf):
        self._commands_handler.add_command(commands_conf["drive_straight"], self.drive_straight, args=["linear_velocity", "distance"] )
        self._commands_handler.add_command(commands_conf["drive_turn"], self.drive_turn, args=["angular_velocity", "angle"] )
        self._commands_handler.add_command(commands_conf["camera_capture_high"], self.handle_high_res_capture )
        self._commands_handler.add_command(commands_conf["camera_streaming_on_off"], self.set_activity_of_camera_streaming, args=["action"] )
        self._commands_handler.add_command(commands_conf["camera_capture_depth"], self.handle_depth_capture )
        self._commands_handler.add_command(commands_conf["power_electronics"], self.handle_electronics_on_off,  args=["subsystem_id", "power_state"] )
        self._commands_handler.add_command(commands_conf["solar_panel"], self.handle_solar_panel, args=["deployment"] )
        self._commands_handler.add_command(commands_conf["go_nogo"], self.handle_go_nogo, args=["decision"] )
        self._commands_handler.add_command(commands_conf["capture_apxs"], self.snap_apxs )
        self._commands_handler.add_command(commands_conf["admin_water_detection"], self.set_is_near_water, args=["trigger_water_detection"] )
        self._commands_handler.add_command(commands_conf["admin_inject_fault"], self.inject_fault)
        self._commands_handler.add_command(commands_conf["lander_camera_capture_high"], self.handle_lander_camera_capture )

    "Creates intervals for automatic transmission of rover's parameters to yamcs"
    def start_streaming_data(self):
        self._payload_handler.snap_apxs() # snaps initial blank apxs reading
        self._images_handler.snap_no_data_images()
        self._intervals_handler.add_new_interval(name="Pose of base link", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_pose_of_base_link)
        self._intervals_handler.add_new_interval(name="camera streaming state", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_camera_streaming_state)
        self._intervals_handler.add_new_interval(name="GO_NOGO", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_go_nogo)
        self._intervals_handler.add_new_interval(name="IMU readings", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_imu_readings)
        self._intervals_handler.add_new_interval(name="OBC state", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_obc_state)
        self._intervals_handler.add_new_interval(name="Radio rssi", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_radio_signal_info)
        self._intervals_handler.add_new_interval(name="Thermal info", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_thermal_info, f_args=[self._intervals["robot_stats"]])
        self._intervals_handler.add_new_interval(name="Power status", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_power_info, f_args=[self._intervals["robot_stats"]])
        self._intervals_handler.add_new_interval(name="OBC metrics", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_obc_metrics)
        self._intervals_handler.add_new_interval(name="Solar panel state", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_solar_panel_state)
        self._intervals_handler.add_new_interval(name="Monitoring camera stream", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._camera_handler.transmit_monitoring_camera_view)
        self._intervals_handler.add_new_interval(name="Motor encoder", seconds=self._intervals["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmitter.transmit_wheels_joint_angles)
        # here add further intervals and their functionalities

    def snap_apxs(self):
        power_state = self._robot.subsystems.get_power_state(Electronics.APXS.value)
        self._payload_handler.snap_apxs(power_state)

    def inject_fault(self):
        self._robot.subsystems.set_electronics_health(Electronics.MOTOR_CONTROLLER.value, HealthStatus.FAULT)
        self._drive_handler.stop_robot()

    def handle_battery_perc_change(self, battery_percentage:int):
        self._robot.subsystems.set_battery_perc(battery_percentage)

    def handle_lander_camera_capture(self):
        self._camera_handler.transmit_lander_camera_view()

    def handle_high_res_capture(self):
        if (self._robot.subsystems.get_electronics_state(Electronics.CAMERA.value) == PowerState.ON):
            self._camera_handler.transmit_camera_view(PragyaanCameraHandler.BUCKET_ONCOMMAND, CameraResolution.HIGH.value, CameraViewType.RGBA)
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self):
        if (self._robot.subsystems.get_electronics_state(Electronics.CAMERA.value) == PowerState.ON):
            self._camera_handler.transmit_camera_view(PragyaanCameraHandler.BUCKET_DEPTH, CameraResolution.HIGH.value, CameraViewType.DEPTH)
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_solar_panel(self, command:str):
        if self._robot.subsystems.get_go_nogo_state() == GoNogoState.NOGO:
            return

        if command == YamcsArguments.DEPLOY.value:
            self._robot.subsystems.set_solar_panel_state(SolarPanelState.DEPLOYED)
            self._robot.deploy_solar_panel()
        elif command == YamcsArguments.STOW.value:
            self._robot.subsystems.set_solar_panel_state(SolarPanelState.STOWED)
            self._robot.stow_solar_panel()
        else:
            print("Command for solar panel is unknown:", command)
            return

    def handle_electronics_on_off(self, electronics:str, new_state:PowerState):
        if new_state not in [PowerState.ON.value, PowerState.OFF.value]:
            print("New decision for PowerState of electronics is unknown:", new_state)
            return
        
        new_state = PowerState[new_state]
        self._robot.subsystems.set_electronics_state(electronics, new_state)

        if electronics == Electronics.CAMERA.value:
            self.set_activity_of_camera_streaming("START") if new_state == PowerState.ON else self._set_activity_of_camera_streaming("STOP")
        elif electronics == Electronics.NEUTRON_SPECTROMETER.value:
            self.set_activity_of_neutron_streaming(new_state)
            pass
        elif electronics == Electronics.MOTOR_CONTROLLER.value:
            if (new_state == PowerState.OFF):
                self._drive_handler.stop_robot()
                self._robot.subsystems.set_electronics_health(Electronics.MOTOR_CONTROLLER.value, HealthStatus.NOMINAL)
        elif electronics == Electronics.RADIO.value:
            #NOTE The rover would lose all communication capabilities if the radio is turned off.
            pass

    def handle_go_nogo(self, decision:str):
        if decision not in [GoNogoState.GO.name, GoNogoState.NOGO.name]:
            print("New decision for GO / NOGO is unknown:", decision)
            return
        
        decision = GoNogoState[decision]
        self._robot.subsystems.set_go_nogo_state(decision)

        if (decision == GoNogoState.NOGO):
                self._drive_handler.stop_robot()
    
    def set_activity_of_camera_streaming(self, action:str):
        if action == YamcsArguments.STOP.value:
            self._intervals_handler.remove_interval(IntervalName.CAMERA_STREAMING.value)
        elif action == YamcsArguments.START.value:
            if not self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value):
                self._intervals_handler.add_new_interval(name=IntervalName.CAMERA_STREAMING.value, seconds=self._intervals["camera_streaming"], is_repeating=True, execute_immediately=False,
                                                 function=self._camera_handler.transmit_camera_view, f_args=(PragyaanCameraHandler.BUCKET_STREAMING, CameraResolution.LOW.value))
        else:
            print("Unknown action:", action)

    def set_activity_of_neutron_streaming(self, power_state:PowerState):
        if power_state == PowerState.OFF:
            self._intervals_handler.remove_interval(IntervalName.NEUTRON_COUNT.value)
        elif power_state == PowerState.ON:
            if not self._intervals_handler.does_exist(IntervalName.NEUTRON_COUNT.value):
                self._intervals_handler.add_new_interval(name=IntervalName.NEUTRON_COUNT.value, seconds=self._intervals["camera_streaming"], is_repeating=True, execute_immediately=False,
                                                 function=self._transmitter.transmit_neutroun_count, f_args=[self._intervals["robot_stats"]])

    def drive_straight(self, linear_velocity, distance):
        self._drive_handler.drive_robot_straight(linear_velocity, distance)

    def drive_turn(self, angular_velocity, angle):
        self._drive_handler.drive_robot_turn(angular_velocity, angle)

    def set_is_near_water(self, trigger_water_detection):
         self._robot.subsystems.set_is_near_water(trigger_water_detection)