__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from src.mission_specific.pragyaan.tmtc.camera_handler import CameraViewType, PragyaanCameraHandler
from src.mission_specific.pragyaan.tmtc.enums import PragyaanCameraResolution, PragyaanYamcsArguments
from src.mission_specific.pragyaan.tmtc.payload_handler import PayloadHandler
from src.mission_specific.pragyaan.tmtc.transmitter import PragyaanTransmitter
from src.subsystems.device import CommonDevice, HealthState, PowerState
from src.subsystems.robot_enums import GoNogoState, ObcState, SolarPanelState
from src.tmtc.intervals_handler import IntervalName


class PragyaanCommander:
    """
    Implementation of commander for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover commanders.

    Commander contains methods specific to the Pragyaan rover.
    Functions implemented here are called by PragyaanController, and are mapped inside setup_command_callbacks function.

    This class encodes the logic behind the execution of the received commands specific to the Pragyaan rover.
        For example, in case of receiving command for high res camera capture, the commander checks if the camera is powered on, and only then executes the command by calling the appropriate function in the camera handler.
    """

    # TODO for v4: find the optimal place to store the angle values (potentially will refactor robot)
    SOLAR_PANEL_ANGLE_DEPLOYED = 0
    SOLAR_PANEL_ANGLE_STOWED = -80

    def __init__(
        self,
        robot,
        intervals,
        payload_handler,
        camera_handler,
        transmitter,
        drive_handler,
        obc_handler,
        intervals_handler,
    ):
        self._robot = robot
        self._intervals = intervals
        self._payload_handler: PayloadHandler = payload_handler
        self._camera_handler: PragyaanCameraHandler = camera_handler
        self._transmitter: PragyaanTransmitter = transmitter
        self._drive_handler = drive_handler
        self._obc_handler = obc_handler
        self._intervals_handler = intervals_handler

    def snap_apxs(self):
        power_state = self._robot.subsystems.get_device_power_state(CommonDevice.APXS)
        self._payload_handler.snap_apxs(power_state)

    def inject_fault(self):
        self._robot.subsystems.set_device_health_state(CommonDevice.MOTOR_CONTROLLER, HealthState.FAULT)
        self._drive_handler.stop_robot()

    def handle_battery_perc_change(self, battery_percentage: int):
        self._robot.subsystems.set_battery_perc(battery_percentage)

    def handle_lander_camera_capture(self):
        self._camera_handler.transmit_lander_camera_view()

    def handle_high_res_capture(self, camera_name: str):
        if self._robot.subsystems.get_device_power_state(CommonDevice.CAMERA) == PowerState.ON:
            camera_idx, _ = self._robot.get_camera_by_name(camera_name)
            self._camera_handler.transmit_camera_view(
                f"{PragyaanCameraHandler.BUCKET_ONCOMMAND_PREFIX}_{camera_name}", PragyaanCameraResolution.HIGH.value, CameraViewType.RGBA, camera_idx
            )
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self, camera_name: str):
        if self._robot.subsystems.get_device_power_state(CommonDevice.CAMERA) == PowerState.ON:
            camera_idx, _ = self._robot.get_camera_by_name(camera_name)
            self._camera_handler.transmit_camera_view(
                f"{PragyaanCameraHandler.BUCKET_DEPTH_PREFIX}_{camera_name}", PragyaanCameraResolution.HIGH.value, CameraViewType.DEPTH, camera_idx
            )
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_solar_panel(self, command: str):
        if self._robot.subsystems.get_go_nogo_state() == GoNogoState.NOGO:
            return

        if command == PragyaanYamcsArguments.DEPLOY.value:
            self._robot.subsystems.set_solar_panel_state(SolarPanelState.DEPLOYED)
            self._robot.set_solar_panel_angle(self.SOLAR_PANEL_ANGLE_DEPLOYED)
        elif command == PragyaanYamcsArguments.STOW.value:
            self._robot.subsystems.set_solar_panel_state(SolarPanelState.STOWED)
            self._robot.set_solar_panel_angle(self.SOLAR_PANEL_ANGLE_STOWED)
        else:
            print("Command for solar panel is unknown:", command)
            return

    def handle_electronics_on_off(self, electronics: str, new_state: PowerState):
        if new_state not in [PowerState.ON.value, PowerState.OFF.value]:
            print("New decision for PowerState of electronics is unknown:", new_state)
            return

        new_state = PowerState[new_state]
        self._robot.subsystems.set_device_power_state(electronics, new_state)

        if electronics == CommonDevice.CAMERA:
            for camera in self._robot._cameras:
                self.set_activity_of_camera_streaming(
                    "START", camera["name"]
                ) if new_state == PowerState.ON else self.set_activity_of_camera_streaming("STOP", camera["name"])
        elif electronics == CommonDevice.NEUTRON_SPECTROMETER:
            self.set_activity_of_neutron_streaming(new_state)
            pass
        elif electronics == CommonDevice.MOTOR_CONTROLLER:
            if new_state == PowerState.OFF:
                self._drive_handler.stop_robot()
                # workshop use-case:
                # in case of a fault on Motor controller, the motor controller should be powered off and on
                # after that, the health state will again be nominal
                self._robot.subsystems.set_device_health_state(CommonDevice.MOTOR_CONTROLLER, HealthState.NOMINAL)
        elif electronics == CommonDevice.RADIO:
            # NOTE The rover would lose all communication capabilities if the radio is turned off.
            pass

    def handle_go_nogo(self, decision: str):
        if decision not in [GoNogoState.GO.name, GoNogoState.NOGO.name]:
            print("New decision for GO / NOGO is unknown:", decision)
            return

        decision = GoNogoState[decision]
        self._robot.subsystems.set_go_nogo_state(decision)

        if decision == GoNogoState.NOGO:
            self._drive_handler.stop_robot()

    def set_activity_of_camera_streaming(self, action: str, camera_name: str):
        if action == PragyaanYamcsArguments.STOP.value:
            self._intervals_handler.remove_interval(f"{IntervalName.CAMERA_STREAMING.value}_{camera_name}")
        elif action == PragyaanYamcsArguments.START.value:
            if not self._intervals_handler.does_exist(f"{IntervalName.CAMERA_STREAMING.value}_{camera_name}"):
                camera_idx, _ = self._robot.get_camera_by_name(camera_name)

                self._intervals_handler.add_new_interval(
                    name=f"{IntervalName.CAMERA_STREAMING.value}_{camera_name}",
                    seconds=self._intervals["camera_streaming"],
                    is_repeating=True,
                    execute_immediately=False,
                    function=self._camera_handler.transmit_camera_view,
                    f_args=(f"{PragyaanCameraHandler.BUCKET_STREAMING_PREFIX}_{camera_name}", PragyaanCameraResolution.LOW.value, CameraViewType.RGBA, camera_idx),
                )
        else:
            print("Unknown action:", action)

    def set_camera_streaming_params(
        self,
        camera_name: str,
        bitrate: int,
        crf: int,
        keyframe_interval: int,
        max_payload_size: int,
        bandwidth_window_seconds: float,
    ):
        # MDB arguments can not be null, so crf = -1 stands for None (bitrate mode)
        try:
            self._camera_handler.set_streaming_compression_params(
                camera_name=camera_name,
                bitrate=bitrate,
                keyframe_interval=keyframe_interval,
                crf=None if crf < 0 else crf,
                max_payload_size=max_payload_size,
                bandwidth_window_seconds=bandwidth_window_seconds,
            )
        except ValueError as error:
            print("New camera streaming parameters are invalid:", error)

    def set_activity_of_neutron_streaming(self, power_state: PowerState):
        if power_state == PowerState.OFF:
            self._intervals_handler.remove_interval(IntervalName.NEUTRON_COUNT.value)
        elif power_state == PowerState.ON:
            if not self._intervals_handler.does_exist(IntervalName.NEUTRON_COUNT.value):
                self._intervals_handler.add_new_interval(
                    name=IntervalName.NEUTRON_COUNT.value,
                    seconds=self._intervals["neutron_count"],
                    is_repeating=True,
                    execute_immediately=False,
                    function=self._transmitter.transmit_neutroun_count,
                )

    def drive_straight(self, linear_velocity, distance):
        self._drive_handler.drive_robot_straight(linear_velocity, distance)

    def drive_turn(self, angular_velocity, angle):
        self._drive_handler.drive_robot_turn(angular_velocity, angle)

    def set_is_near_water(self, trigger_water_detection):
        self._robot.subsystems.set_is_near_water(trigger_water_detection)
