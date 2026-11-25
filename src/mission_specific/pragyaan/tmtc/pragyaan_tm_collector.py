__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import math

from src.configurations.robot_camera_confs import cameras_of_type
from src.mission_specific.pragyaan.tmtc.enums import STREAMING_FIELDS
from src.robots.robot import Robot
from src.subsystems.device import CommonDevice, PowerState
from src.subsystems.robot_enums import SolarPanelState
from src.tmtc import yamcs_naming
from src.tmtc.intervals_handler import IntervalName, camera_streaming_interval
from src.tmtc.tm_collector import TMCollector


class PragyaanTMCollector(TMCollector):
    """
    Implementation of TM collector for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover TM collectors.

    TM collector contains methods specific to the Pragyaan rover.
    The _collect_... functions arrange the data from the simulation to the data structure expected by Yamcs,
    and store it as the latest value of the corresponding TM parameter.
    They are called periodically by the intervals set up inside start_collecting_TM.

    Sending the collected values to Yamcs is not done here, but by PragyaanController (see start_transmitting_TM).
    """

    # packets_sent/packets_lost are 16-bit unsigned in the Yamcs MDB, so they wrap from 65535 back to 0.
    PACKET_COUNTER_MODULO = 1 << 16

    def __init__(self, intervals_handler, robot, robot_RG, robot_name, parameters_conf, intervals, camera_handler):
        super().__init__()
        self._robot: Robot = robot
        self._camera_handler = camera_handler
        self._intervals_handler = intervals_handler
        self._parameters_conf = parameters_conf
        self._intervals = intervals
        self._robot_RG = robot_RG
        self._robot_name = robot_name
        self.define_parameters(self._parameters_conf.values())
        # per-camera streaming telemetry is not listed in parameters_conf, but computed from the robot's rgba cameras
        self.define_parameters(
            yamcs_naming.streaming_param(camera_name, field)
            for camera_name in cameras_of_type(self._robot.cameras, "rgba")
            for field in STREAMING_FIELDS
        )

    "Creates intervals for automatic collection of rover's parameters"

    def start_collecting_TM(self):
        self._intervals_handler.add_new_interval(
            name="Pose of base link",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_pose_of_base_link,
        )
        self._intervals_handler.add_new_interval(
            name="camera streaming state",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_camera_streaming_state,
        )
        self._intervals_handler.add_new_interval(
            name="Camera compression",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_camera_compression_info,
        )
        self._intervals_handler.add_new_interval(
            name="GO_NOGO",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_go_nogo,
        )
        self._intervals_handler.add_new_interval(
            name="IMU readings",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_imu_readings,
        )
        self._intervals_handler.add_new_interval(
            name="OBC state",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_obc_state,
        )
        self._intervals_handler.add_new_interval(
            name="Radio rssi",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_radio_signal_info,
        )
        self._intervals_handler.add_new_interval(
            name="Thermal info",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_thermal_info,
            f_args=[self._intervals["robot_stats"]],
        )
        self._intervals_handler.add_new_interval(
            name="Power status",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_power_info,
            f_args=[self._intervals["robot_stats"]],
        )
        self._intervals_handler.add_new_interval(
            name="OBC metrics",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_obc_metrics,
        )
        self._intervals_handler.add_new_interval(
            name="Solar panel state",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_solar_panel_state,
        )
        self._intervals_handler.add_new_interval(
            name="Motor encoder",
            seconds=self._intervals["robot_stats"],
            is_repeating=True,
            execute_immediately=True,
            function=self._collect_wheels_joint_angles,
        )
        # here add further intervals and their functionalities

    def set_activity_of_neutron_collecting(self, power_state: PowerState):
        if power_state == PowerState.OFF:
            self._intervals_handler.remove_interval(IntervalName.NEUTRON_COUNT.value)
        elif power_state == PowerState.ON:
            if not self._intervals_handler.does_exist(IntervalName.NEUTRON_COUNT.value):
                self._intervals_handler.add_new_interval(
                    name=IntervalName.NEUTRON_COUNT.value,
                    seconds=self._intervals["robot_stats"],
                    is_repeating=True,
                    execute_immediately=False,
                    function=self._collect_neutron_count,
                )

    def _collect_wheels_joint_angles(self):
        angles = self._robot.get_wheels_joint_angles()
        if not angles:  # articulation telemetry not ready yet (startup, restart): nothing measured to collect
            return
        self._transform_joint_angles(angles)
        self.update(self._parameters_conf["motor_encoder"], angles)

    def _transform_joint_angles(self, angles):
        for i in range(len(angles)):
            modulo = angles[i] % (2 * math.pi)
            angles[i] = int(modulo * (1024.0 / (2 * math.pi)))

    def _collect_solar_panel_state(self):
        state: SolarPanelState = self._robot.subsystems.get_solar_panel_state()
        self.update(self._parameters_conf["solar_panel_state"], state.value)

    def _collect_obc_metrics(self):
        obc_metrics = self._robot.subsystems.get_obc_model_outputs()

        self.update(self._parameters_conf["obc_cpu_usage"], int(obc_metrics["cpu_usage"]))
        self.update(self._parameters_conf["obc_ram_usage"], int(obc_metrics["ram_usage"]))
        self.update(self._parameters_conf["obc_disk_usage"], int(obc_metrics["disk_usage"]))
        self.update(self._parameters_conf["obc_uptime"], obc_metrics["uptime"])

    def _collect_radio_signal_info(self):
        robot_position, orientation = self._robot_RG.get_pose_of_base_link()
        radio_status = self._robot.subsystems.get_radio_model_outputs(robot_position)
        self.update(self._parameters_conf["rssi"], int(radio_status["rssi"]))

    def _collect_thermal_info(self, interval_s):
        _, _, imu_orientation = self._robot.get_imu_readings()
        robot_yaw_deg = math.degrees(imu_orientation["yaw"])
        temperatures = self._robot.subsystems.get_thermal_model_outputs(robot_yaw_deg, interval_s)
        self.update(self._parameters_conf["temperature_front"], temperatures["+X"])
        self.update(self._parameters_conf["temperature_back"], temperatures["-X"])
        self.update(self._parameters_conf["temperature_left"], temperatures["+Y"])
        self.update(self._parameters_conf["temperature_right"], temperatures["-Y"])
        self.update(self._parameters_conf["temperature_top"], temperatures["+Z"])
        self.update(self._parameters_conf["temperature_bottom"], temperatures["-Z"])
        self.update(self._parameters_conf["temperature_elec_box"], temperatures["interior"])

    def _collect_power_info(self, interval_s):
        _, _, imu_orientation = self._robot.get_imu_readings()
        robot_yaw_deg = math.degrees(imu_orientation["yaw"])
        obc_state = self._robot.subsystems.get_obc_state()
        power_status = self._robot.subsystems.get_power_model_outputs(robot_yaw_deg, interval_s, obc_state)
        self.update(self._parameters_conf["battery_charge"], int(power_status["battery_percentage_measured"]))
        self.update(self._parameters_conf["battery_voltage"], power_status["battery_voltage_measured"])
        self.update(self._parameters_conf["total_current_in"], power_status["solar_input_current_measured"])
        self.update(self._parameters_conf["total_current_out"], power_status["total_current_out_measured"])
        self.update(
            self._parameters_conf["current_draw_obc"], power_status["device_currents_measured"][CommonDevice.OBC]
        )
        self.update(
            self._parameters_conf["current_draw_motor_controller"],
            power_status["device_currents_measured"][CommonDevice.MOTOR_CONTROLLER],
        )
        self.update(
            self._parameters_conf["current_draw_neutron_spectrometer"],
            power_status["device_currents_measured"][CommonDevice.NEUTRON_SPECTROMETER],
        )
        self.update(
            self._parameters_conf["current_draw_apxs"], power_status["device_currents_measured"][CommonDevice.APXS]
        )
        self.update(
            self._parameters_conf["current_draw_camera"], power_status["device_currents_measured"][CommonDevice.CAMERA]
        )
        self.update(
            self._parameters_conf["current_draw_radio"], power_status["device_currents_measured"][CommonDevice.RADIO]
        )
        self.update(
            self._parameters_conf["current_draw_eps"], power_status["device_currents_measured"][CommonDevice.EPS]
        )
        self.update(self._parameters_conf["motor_current"], power_status["motor_currents_measured"])

    def _collect_neutron_count(self):
        neutron_counts = self._robot.subsystems.get_neutron_count()
        self.update(self._parameters_conf["neutron_counts"], neutron_counts)

    def _collect_obc_state(self):
        obc_state = self._robot.subsystems.get_obc_state()
        self.update(self._parameters_conf["obc_state"], obc_state.value)

    def _collect_imu_readings(self):
        imu_accelerometer, imu_gyroscope, orientation = self._robot.get_imu_readings()
        self.update(self._parameters_conf["imu_accelerometer"], imu_accelerometer)
        self.update(self._parameters_conf["imu_gyroscope"], imu_gyroscope)
        self.update(self._parameters_conf["imu_orientation"], orientation)

    # Per-camera streaming telemetry, under yamcs_naming.streaming_param(camera, field) with the fields of
    # enums.STREAMING_FIELDS (state, bandwidth, packets_sent, packets_lost, decode_errors): keep them in step.

    def _collect_camera_streaming_state(self):
        for camera_name in cameras_of_type(self._robot.cameras, "rgba"):
            is_camera_streaming = self._intervals_handler.does_exist(camera_streaming_interval(camera_name))
            state = PowerState.ON if is_camera_streaming else PowerState.OFF
            self.update(yamcs_naming.streaming_param(camera_name, field="state"), state)

    def _collect_camera_compression_info(self):
        for camera_name in cameras_of_type(self._robot.cameras, "rgba"):
            compression_status = self._camera_handler.get_compression_model_outputs(camera_name)
            # 0 until the first frame is encoded
            self.update(
                yamcs_naming.streaming_param(camera_name, field="bandwidth"), int(compression_status["bandwidth"])
            )
            # Packet counters roll over like hardware counters; the model keeps the full counts.
            packets_sent = compression_status["packets_sent"] % self.PACKET_COUNTER_MODULO
            packets_lost = compression_status["packets_lost"] % self.PACKET_COUNTER_MODULO
            self.update(yamcs_naming.streaming_param(camera_name, field="packets_sent"), packets_sent)
            self.update(yamcs_naming.streaming_param(camera_name, field="packets_lost"), packets_lost)
            self.update(
                yamcs_naming.streaming_param(camera_name, field="decode_errors"), compression_status["decode_errors"]
            )

    def _collect_go_nogo(self):
        go_nogo_state = self._robot.subsystems.get_go_nogo_state().value
        self.update(self._parameters_conf["go_nogo"], go_nogo_state)

    def _collect_pose_of_base_link(self):
        position, orientation = self._robot_RG.get_pose_of_base_link()
        lander_pos = self._robot.subsystems.get_lander_position()
        position = position - lander_pos
        # euler_orient = transform_orientation_into_xyz(orientation)
        position = position.tolist()
        orientation = orientation.tolist()
        pose_of_base_link = {
            "position": {"x": position[0], "y": position[1], "z": position[2]},
            "orientation": {"w": orientation[0], "x": orientation[1], "y": orientation[2], "z": orientation[3]},
        }
        self.update(self._parameters_conf["pose_of_base_link"], pose_of_base_link)
