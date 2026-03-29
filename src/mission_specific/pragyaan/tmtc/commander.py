__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.mission_specific.pragyaan.tmtc.enums import PragyaanCameraResolution, PragyaanYamcsArguments
from src.subsystems.device import CommonDevice, HealthState, PowerState
from src.subsystems.robot_enums import GoNogoState, ObcState, SolarPanelState
from src.tmtc.intervals_handler import IntervalName
from src.mission_specific.pragyaan.tmtc.camera_handler import CameraViewType, PragyaanCameraHandler
from src.mission_specific.pragyaan.tmtc.payload_handler import PayloadHandler
from src.mission_specific.pragyaan.tmtc.transmitter import PragyaanTransmitter
import omni.kit.app

class PragyaanCommander:
    """
    Implementation of commander for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover commanders.

    Commander contains methods specific to the Pragyaan rover. 
    Functions implemented here are called by PragyaanController, and are mapped inside setup_command_callbacks function.
    
    This class encodes the logic behind the execution of the received commands specific to the Pragyaan rover.
        For example, in case of receiving command for high res camera capture, the commander checks if the camera is powered on, and only then executes the command by calling the appropriate function in the camera handler.
    """
    def __init__(self,
        robot,
        intervals,
        payload_handler,
        camera_handler,
        transmitter,
        drive_handler,
        obc_handler,
        intervals_handler
        ):
        self._robot = robot
        self._intervals = intervals
        self._payload_handler:PayloadHandler = payload_handler
        self._camera_handler:PragyaanCameraHandler = camera_handler
        self._transmitter:PragyaanTransmitter = transmitter
        self._drive_handler = drive_handler
        self._obc_handler = obc_handler
        self._intervals_handler = intervals_handler

    def snap_apxs(self):
        power_state = self._robot.subsystems.get_device_power_state(CommonDevice.APXS)
        self._payload_handler.snap_apxs(power_state)

    def inject_fault(self):
        self._robot.subsystems.set_device_health_state(CommonDevice.MOTOR_CONTROLLER, HealthState.FAULT)
        self._drive_handler.stop_robot()

    def handle_battery_perc_change(self, battery_percentage:int):
        self._robot.subsystems.set_battery_perc(battery_percentage)

    def handle_lander_camera_capture(self):
        self._camera_handler.transmit_lander_camera_view()

    def handle_high_res_capture(self):
        if (self._robot.subsystems.get_device_power_state(CommonDevice.CAMERA) == PowerState.ON):
            self._camera_handler.transmit_camera_view(PragyaanCameraHandler.BUCKET_ONCOMMAND, PragyaanCameraResolution.HIGH.value, CameraViewType.RGBA)
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self):
        if (self._robot.subsystems.get_device_power_state(CommonDevice.CAMERA) == PowerState.ON):
            self._camera_handler.transmit_camera_view(PragyaanCameraHandler.BUCKET_DEPTH, PragyaanCameraResolution.HIGH.value, CameraViewType.DEPTH)
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_solar_panel(self, command:str):
        if self._robot.subsystems.get_go_nogo_state() == GoNogoState.NOGO:
            return

        if command == PragyaanYamcsArguments.DEPLOY.value:
            self._robot.subsystems.set_solar_panel_state(SolarPanelState.DEPLOYED)
            self._robot.deploy_solar_panel()
        elif command == PragyaanYamcsArguments.STOW.value:
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
        self._robot.subsystems.set_device_power_state(electronics, new_state)

        if electronics == CommonDevice.CAMERA:
            self._set_activity_of_camera_streaming("START") if new_state == PowerState.ON else self._set_activity_of_camera_streaming("STOP")
        elif electronics == CommonDevice.NEUTRON_SPECTROMETER:
            self._set_activity_of_neutron_streaming(new_state)
            pass
        elif electronics == CommonDevice.MOTOR_CONTROLLER:
            if (new_state == PowerState.OFF):
                self._drive_handler.stop_robot()
                # workshop use-case:
                # in case of a fault on Motor controller, the motor controller should be powered off and on
                # after that, the health state will again be nominal
                self._robot.subsystems.set_device_health_state(CommonDevice.MOTOR_CONTROLLER, HealthState.NOMINAL)
        elif electronics == CommonDevice.RADIO:
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
        if action == PragyaanYamcsArguments.STOP.value:
            self._intervals_handler.remove_interval(IntervalName.CAMERA_STREAMING.value)
        elif action == PragyaanYamcsArguments.START.value:
            if not self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value):
                self._intervals_handler.add_new_interval(name=IntervalName.CAMERA_STREAMING.value, seconds=self._intervals["camera_streaming"], is_repeating=True, execute_immediately=False,
                                                 function=self._camera_handler.transmit_camera_view, f_args=(PragyaanCameraHandler.BUCKET_STREAMING, PragyaanCameraResolution.LOW.value))
        else:
            print("Unknown action:", action)

    def set_activity_of_neutron_streaming(self, power_state:PowerState):
        if power_state == PowerState.OFF:
            self._intervals_handler.remove_interval(IntervalName.NEUTRON_COUNT.value)
        elif power_state == PowerState.ON:
            if not self._intervals_handler.does_exist(IntervalName.NEUTRON_COUNT.value):
                self._intervals_handler.add_new_interval(name=IntervalName.NEUTRON_COUNT.value, seconds=self._intervals["camera_streaming"], is_repeating=True, execute_immediately=False,
                                                 function=self._transmitter.transmit_neutroun_count)

    def drive_straight(self, linear_velocity, distance):
        self._drive_handler.drive_robot_straight(linear_velocity, distance)

    def drive_turn(self, angular_velocity, angle):
        self._drive_handler.drive_robot_turn(angular_velocity, angle)

    def set_is_near_water(self, trigger_water_detection):
         self._robot.subsystems.set_is_near_water(trigger_water_detection)