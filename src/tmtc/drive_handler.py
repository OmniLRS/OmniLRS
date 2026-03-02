__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.robots.subsystems_manager import Electronics, GoNogoState, HealthStatus, ObcState, PowerState
import math
import omni.kit.app
from src.tmtc.intervals_handler import IntervalName

class DriveHandler:

    MIN_VELOCITY = 0.2

    def __init__(
        self,
        robot,
        intervals_handler,
        obc_handler
    ) -> None:
        self._robot = robot
        self._intervals_handler = intervals_handler
        self._obc_handler = obc_handler

    def drive_robot_straight(self, linear_velocity, distance):
        if not self._is_robot_able_to_drive():
            return
        
        if linear_velocity == 0 or distance == 0:
            self.stop_robot()
            return
        
        self._obc_handler.set_obc_state(ObcState.MOTOR)
        self._drive_with_controlled_acceleration(linear_velocity, distance)

    def _drive_with_controlled_acceleration(self, orig_linear_velocity, orig_distance):
        slow_velocity = self._calculate_slow_velocity(orig_linear_velocity)
        _linear_velocity, _distance = self._adjust_speed_and_distance(slow_velocity, 0.05*orig_distance)
        self._robot.drive_straight(_linear_velocity)
        drive_time = _distance / abs(_linear_velocity)
        self._switch_to_next_drive_mode_after_time(drive_time, orig_linear_velocity, orig_distance, self._drive_requested_speed)

    def _drive_requested_speed(self, orig_linear_velocity, orig_distance):
        _linear_velocity, _distance = self._adjust_speed_and_distance(orig_linear_velocity, 0.9*orig_distance)
        self._robot.drive_straight(_linear_velocity)
        drive_time = _distance / abs(_linear_velocity)
        self._switch_to_next_drive_mode_after_time(drive_time, orig_linear_velocity, orig_distance, self._slow_down)

    def _slow_down(self, orig_linear_velocity, orig_distance):
        if (self._robot.subsystems.get_obc_state() == ObcState.IDLE):
            return

        slow_velocity = self._calculate_slow_velocity(orig_linear_velocity)
        _linear_velocity, _distance = self._adjust_speed_and_distance(slow_velocity, 0.05*orig_distance)
        self._robot.drive_straight(_linear_velocity)
        drive_time = _distance / abs(_linear_velocity)
        self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)
        self._stop_robot_after_time(drive_time)

    def _calculate_slow_velocity(self, orig_linear_velocity):
        #NOTE assumpion is that the min speed is 0.25
        # and that the max speed is 1 
        # with speed higher than 1, rover does not move properly
        slow_velocity = abs(orig_linear_velocity) / 2

        if (slow_velocity < self.MIN_VELOCITY):
            slow_velocity = self.MIN_VELOCITY

        return math.copysign(slow_velocity, orig_linear_velocity)

    def _switch_to_next_drive_mode_after_time(self, drive_time, linear_velocity, distance, next_mode_func):
        if self._intervals_handler.does_exist(IntervalName.CONTROLLED_DRIVE.value):
            self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)

        self._intervals_handler.add_new_interval(name=IntervalName.CONTROLLED_DRIVE.value, seconds=drive_time, is_repeating=False, execute_immediately=False,
                                                function=next_mode_func, f_args=[linear_velocity, distance])

    def _adjust_speed_and_distance(self, linear_velocity, distance):
        linear_velocity *= 10
        distance *= 10

        return linear_velocity, distance

    def drive_robot_turn(self, angular_velocity, angle):
        if not self._is_robot_able_to_drive():
            return

        if angular_velocity == 0 or angle == 0:
            self.stop_robot()
            return
        
        turn_speed = self._calculate_turn_speed(angular_velocity)
        turn_time = angle / abs(angular_velocity)
        self._robot.drive_turn(turn_speed)
        self._stop_robot_after_time(turn_time)

    def _is_robot_able_to_drive(self):
        motor_state:PowerState = self._robot.subsystems.get_electronics_state(Electronics.MOTOR_CONTROLLER.value)
        go_state:GoNogoState = self._robot.subsystems.get_go_nogo_state()
        motor_health:HealthStatus = self._robot.subsystems.get_electronics_health(Electronics.MOTOR_CONTROLLER.value)

        return go_state == GoNogoState.GO and motor_state == PowerState.ON and motor_health == HealthStatus.NOMINAL

    def _calculate_turn_speed(self, angular_velocity):
        robot_width = self._robot.dimensions["width"]
        radians = math.radians(angular_velocity) 
        wheel_speed = radians * (robot_width / 2)
        adjusted_speed = wheel_speed * self._robot.turn_speed_coef

        return adjusted_speed

    def _stop_robot_after_time(self, travel_time):
        self._obc_handler.set_obc_state(ObcState.MOTOR, travel_time)
        if self._intervals_handler.does_exist(IntervalName.STOP_ROBOT.value):
            self._intervals_handler.update_next_time(IntervalName.STOP_ROBOT.value, travel_time)
        else:
            self._intervals_handler.add_new_interval(name=IntervalName.STOP_ROBOT.value, seconds=travel_time, is_repeating=False, execute_immediately=False,
                                                 function=self.stop_robot)

    def stop_robot(self):
        self._intervals_handler.remove_interval(IntervalName.STOP_ROBOT.value)
        self._intervals_handler.remove_interval(IntervalName.CONTROLLED_DRIVE.value)
        self._robot.stop_drive()
        self._obc_handler.set_obc_state(ObcState.IDLE)