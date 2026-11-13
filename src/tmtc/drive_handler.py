__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import math

from src.subsystems.device import CommonDevice, HealthState, PowerState
from src.subsystems.robot_enums import GoNogoState, ObcState
from src.tmtc.intervals_handler import IntervalName


class DriveHandler:
    """
    Drive commands of a skid-steered robot: drive_straight (m/s, m) and drive_turn (deg/s, deg). The wheel speeds come
    from the `drive` section of the robot yaml (DriveConf): wheel radius, track width and skid-steer factor.
    """

    MIN_VELOCITY = 0.2  # m/s, slowest speed of the start and stop phases of a straight drive

    def __init__(self, robot, intervals_handler, obc_handler) -> None:
        self._robot = robot
        self._intervals_handler = intervals_handler
        self._obc_handler = obc_handler

    def drive_robot_straight(self, linear_velocity, distance):
        if not self._is_robot_able_to_drive() or not self._has_drive_conf():
            return

        if linear_velocity == 0 or distance == 0:
            self.stop_robot()
            return

        self._cancel_pending()
        self._obc_handler.set_obc_state(ObcState.MOTOR)
        self._drive_with_controlled_acceleration(linear_velocity, distance)

    def _cancel_pending(self):
        """A new drive command replaces the previous one: its next phase and its stop are cancelled."""
        self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)
        self._intervals_handler.remove_interval(IntervalName.STOP_ROBOT.value)

    def _has_drive_conf(self) -> bool:
        if self._robot.drive is None:
            print("[DriveHandler] The robot yaml has no `drive` section (wheel_radius, track_width): command ignored.")
            return False
        return True

    def _wheel_speed(self, linear_velocity):
        """Wheel speed in rad/s for a ground speed in m/s."""
        return linear_velocity / self._robot.drive.wheel_radius

    # A straight drive covers 5 % of the distance at the slow speed, 90 % at the requested speed and 5 % at the slow
    # speed again.
    def _drive_with_controlled_acceleration(self, orig_linear_velocity, orig_distance):
        slow_velocity = self._calculate_slow_velocity(orig_linear_velocity)
        self._robot.drive_straight(self._wheel_speed(slow_velocity))
        drive_time = 0.05 * orig_distance / abs(slow_velocity)
        self._switch_to_next_drive_mode_after_time(
            drive_time, orig_linear_velocity, orig_distance, self._drive_requested_speed
        )

    def _drive_requested_speed(self, orig_linear_velocity, orig_distance):
        self._robot.drive_straight(self._wheel_speed(orig_linear_velocity))
        drive_time = 0.9 * orig_distance / abs(orig_linear_velocity)
        self._switch_to_next_drive_mode_after_time(drive_time, orig_linear_velocity, orig_distance, self._slow_down)

    def _slow_down(self, orig_linear_velocity, orig_distance):
        slow_velocity = self._calculate_slow_velocity(orig_linear_velocity)
        self._robot.drive_straight(self._wheel_speed(slow_velocity))
        drive_time = 0.05 * orig_distance / abs(slow_velocity)
        self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)
        self._stop_robot_after_time(drive_time)

    def _calculate_slow_velocity(self, orig_linear_velocity):
        """Half the requested speed, but at least MIN_VELOCITY and at most the requested speed."""
        slow_velocity = min(abs(orig_linear_velocity), max(abs(orig_linear_velocity) / 2, self.MIN_VELOCITY))
        return math.copysign(slow_velocity, orig_linear_velocity)

    def _switch_to_next_drive_mode_after_time(self, drive_time, linear_velocity, distance, next_mode_func):
        if self._intervals_handler.does_exist(IntervalName.CONTROLLED_DRIVE.value):
            self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)

        self._intervals_handler.add_new_interval(
            name=IntervalName.CONTROLLED_DRIVE.value,
            seconds=drive_time,
            is_repeating=False,
            execute_immediately=False,
            function=next_mode_func,
            f_args=[linear_velocity, distance],
        )

    def drive_robot_turn(self, angular_velocity, angle):
        if not self._is_robot_able_to_drive() or not self._has_drive_conf():
            return

        if angular_velocity == 0 or angle == 0:
            self.stop_robot()
            return

        self._cancel_pending()
        turn_speed = self._calculate_turn_speed(angular_velocity)
        turn_time = angle / abs(angular_velocity)
        self._robot.drive_turn(turn_speed)
        self._stop_robot_after_time(turn_time)

    def _is_robot_able_to_drive(self):
        go_state: GoNogoState = self._robot.subsystems.get_go_nogo_state()
        motor_state: PowerState = self._robot.subsystems.get_device_power_state(CommonDevice.MOTOR_CONTROLLER)
        motor_health: HealthState = self._robot.subsystems.get_device_health_state(CommonDevice.MOTOR_CONTROLLER)

        return go_state == GoNogoState.GO and motor_state == PowerState.ON and motor_health == HealthState.NOMINAL

    def _calculate_turn_speed(self, angular_velocity):
        """Wheel speed in rad/s (right side; the left side turns the other way) for a yaw rate in deg/s."""
        drive = self._robot.drive
        return math.radians(angular_velocity) * (drive.track_width / 2) * drive.skid_steer_factor / drive.wheel_radius

    def _stop_robot_after_time(self, travel_time):
        self._obc_handler.set_obc_state(ObcState.MOTOR, travel_time)
        if self._intervals_handler.does_exist(IntervalName.STOP_ROBOT.value):
            self._intervals_handler.update_next_time(IntervalName.STOP_ROBOT.value, travel_time)
        else:
            self._intervals_handler.add_new_interval(
                name=IntervalName.STOP_ROBOT.value,
                seconds=travel_time,
                is_repeating=False,
                execute_immediately=False,
                function=self.stop_robot,
            )

    def stop_robot(self):
        self._intervals_handler.remove_interval(IntervalName.STOP_ROBOT.value)
        self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)
        self._robot.stop_drive()
        self._obc_handler.set_obc_state(ObcState.IDLE)
