__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.environments.utils import transform_orientation_into_xyz
import math
import omni.kit.app
from src.subsystems.device import CommonDevice, PowerState
from src.robots.robot import Robot
from src.subsystems.robot_enums import SolarPanelState
from src.tmtc.intervals_handler import IntervalName

class PragyaanTransmitter:
    """
    Implementation of transmitter for the Pragyaan rover.
    Functions as a reference implementation for the implementation of other rover transmitters`.

    Transmitter contains methods specific to the Pragyaan rover. 
    Functions implemented here are called by PragyaanController, and are mapped inside the start_streaming_data function.
    
    This class arranges the data from the simulation to the data structure expected by the transmit_func. 
    (in this case, the transmit_func is provided by the YamcsTMTC class, and is used to transmit the data to Yamcs)
    
    """

    def __init__(
            self,
            transmit_func,
            intervals_handler,
            robot, 
            robot_RG,
            robot_name,
            parameters_conf
    ):
        self._transmit = transmit_func
        self._robot:Robot = robot
        self._intervals_handler = intervals_handler
        self._parameters_conf = parameters_conf
        self._robot_RG = robot_RG
        self._robot_name = robot_name

    def transmit_wheels_joint_angles(self):
        angles = self._robot.get_wheels_joint_angles()
        self._transform_joint_angles(angles)
        self._transmit(self._parameters_conf["motor_encoder"], angles)

    def _transform_joint_angles(self, angles):
        for i in range(len(angles)):
            modulo = angles[i] % (2 * math.pi)
            angles[i] = int(modulo * (1024.0 / (2 * math.pi)))

    def transmit_solar_panel_state(self):
        state:SolarPanelState = self._robot.subsystems.get_solar_panel_state()
        self._transmit(self._parameters_conf["solar_panel_state"], state.value)

    def transmit_obc_metrics(self):
        obc_metrics = self._robot.subsystems.get_obc_status()

        self._transmit(self._parameters_conf["obc_cpu_usage"], int(obc_metrics["cpu_usage"]))
        self._transmit(self._parameters_conf["obc_ram_usage"], int(obc_metrics["ram_usage"]))
        self._transmit(self._parameters_conf["obc_disk_usage"], int(obc_metrics["disk_usage"]))
        self._transmit(self._parameters_conf["obc_uptime"], obc_metrics["uptime"])
        
    def transmit_radio_signal_info(self):
        robot_position, orientation = self._robot_RG.get_pose_of_base_link()
        rssi = self._robot.subsystems.get_radio_status(robot_position)
        self._transmit(self._parameters_conf["rssi"], int(rssi))

    def transmit_thermal_info(self, interval_s):
        robot_position, _ = self._robot_RG.get_pose_of_base_link()
        _, _, imu_orientation = self._robot.get_imu_readings()
        robot_yaw_deg = imu_orientation["yaw"]
        temperatures = self._robot.subsystems.get_thermal_status(
            robot_position, 
            robot_yaw_deg, 
            interval_s
        )
        self._transmit(self._parameters_conf["temperature_front"], temperatures['+X'])
        self._transmit(self._parameters_conf["temperature_back"], temperatures['-X'])
        self._transmit(self._parameters_conf["temperature_left"], temperatures['+Y'])
        self._transmit(self._parameters_conf["temperature_right"], temperatures['-Y'])
        self._transmit(self._parameters_conf["temperature_top"], temperatures['+Z'])
        self._transmit(self._parameters_conf["temperature_bottom"], temperatures['-Z'])
        self._transmit(self._parameters_conf["temperature_elec_box"], temperatures['interior'])

    def transmit_power_info(self, interval_s):
        robot_position, _ = self._robot_RG.get_pose_of_base_link()
        _, _, imu_orientation = self._robot.get_imu_readings()
        robot_yaw_deg = imu_orientation['yaw']
        obc_state = self._robot.subsystems.get_obc_state()
        power_status = self._robot.subsystems.get_power_status(
            robot_position,
            robot_yaw_deg, 
            interval_s, 
            obc_state
        )
        self._transmit(self._parameters_conf["battery_charge"], int(power_status['battery_percentage_measured']))
        self._transmit(self._parameters_conf["battery_voltage"], power_status['battery_voltage_measured'])
        self._transmit(self._parameters_conf["total_current_in"], power_status['solar_input_current_measured'])
        self._transmit(self._parameters_conf["total_current_out"], power_status["total_current_out_measured"])
        self._transmit(self._parameters_conf["current_draw_obc"], power_status["device_currents_measured"][CommonDevice.OBC])
        self._transmit(self._parameters_conf["current_draw_motor_controller"], power_status["device_currents_measured"][CommonDevice.MOTOR_CONTROLLER])
        self._transmit(self._parameters_conf["current_draw_neutron_spectrometer"], power_status["device_currents_measured"][CommonDevice.NEUTRON_SPECTROMETER])
        self._transmit(self._parameters_conf["current_draw_apxs"], power_status["device_currents_measured"][CommonDevice.APXS])
        self._transmit(self._parameters_conf["current_draw_camera"], power_status["device_currents_measured"][CommonDevice.CAMERA])
        self._transmit(self._parameters_conf["current_draw_radio"], power_status["device_currents_measured"][CommonDevice.RADIO])
        self._transmit(self._parameters_conf["current_draw_eps"], power_status["device_currents_measured"][CommonDevice.EPS])
        self._transmit(self._parameters_conf["motor_current"], power_status['motor_currents_measured'])

    def transmit_neutroun_count(self):
        neutron_counts = self._robot.subsystems.get_neutron_count()
        self._transmit(self._parameters_conf["neutron_counts"], neutron_counts)

    def transmit_obc_state(self):
        obc_state = self._robot.subsystems.get_obc_state()
        self._transmit(self._parameters_conf["obc_state"], obc_state.value)

    def transmit_imu_readings(self):
        imu_accelerometer, imu_gyroscope, orientation = self._robot.get_imu_readings()
        self._transmit(self._parameters_conf["imu_accelerometer"], imu_accelerometer)
        self._transmit(self._parameters_conf["imu_gyroscope"], imu_gyroscope)
        self._transmit(self._parameters_conf["imu_orientation"], orientation)
        
    def transmit_camera_streaming_state(self):
        is_camera_streaming = self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value)
        state = PowerState.ON if is_camera_streaming else PowerState.OFF
        self._transmit(self._parameters_conf["camera_streaming_state"], state)

    def transmit_go_nogo(self):
        go_nogo_state =  self._robot.subsystems.get_go_nogo_state().value
        self._transmit(self._parameters_conf["go_nogo"], go_nogo_state)

    def transmit_pose_of_base_link(self):
        position, orientation = self._robot_RG.get_pose_of_base_link()
        lander_pos = self._robot.subsystems.get_lander_position()
        position = position - lander_pos
        # euler_orient = transform_orientation_into_xyz(orientation)
        position = position.tolist()
        orientation = orientation.tolist()
        pose_of_base_link = {"position": {"x":position[0], "y":position[1], "z":position[2]}, 
                                "orientation":{"w":orientation[0],"x":orientation[1], "y":orientation[2], "z":orientation[3] }}
        self._transmit(self._parameters_conf["pose_of_base_link"], pose_of_base_link)