"""Wheel kinematics of the Yamcs drive commands (src/tmtc/drive_handler.py, DriveConf), without Isaac Sim."""

import math
import sys
import types

import pytest

# src.tmtc.intervals_handler imports omni (Kit); the drive handler only needs its IntervalName enum.
for name in ("omni", "omni.kit", "omni.kit.app", "omni.timeline"):
    sys.modules.setdefault(name, types.ModuleType(name))

from src.configurations.robot_confs import DriveConf  # noqa: E402
from src.subsystems.device import HealthState, PowerState  # noqa: E402
from src.subsystems.robot_enums import GoNogoState, ObcState  # noqa: E402
from src.tmtc.drive_handler import DriveHandler  # noqa: E402

PRAGYAAN = DriveConf(wheel_radius=0.0972, track_width=0.882, skid_steer_factor=2.9)


class FakeSubsystems:
    def __init__(self):
        self.obc_state = ObcState.IDLE

    def get_go_nogo_state(self):
        return GoNogoState.GO

    def get_device_power_state(self, device):
        return PowerState.ON

    def get_device_health_state(self, device):
        return HealthState.NOMINAL

    def get_obc_state(self):
        return self.obc_state


class FakeRobot:
    def __init__(self, drive):
        self.drive = drive
        self.subsystems = FakeSubsystems()
        self.commands = []  # ("straight" | "turn" | "stop", wheel speed in rad/s)

    def drive_straight(self, wheel_speed):
        self.commands.append(("straight", wheel_speed))

    def drive_turn(self, wheel_speed):
        self.commands.append(("turn", wheel_speed))

    def stop_drive(self):
        self.commands.append(("stop", 0.0))


class FakeIntervals:
    """Records the intervals; run_next() runs the pending one-shot interval, as the sim would after its delay."""

    def __init__(self):
        self.intervals = {}

    def add_new_interval(self, name, seconds, is_repeating, execute_immediately, function, f_args=None):
        self.intervals[name] = (seconds, function, f_args or [])

    def does_exist(self, name):
        return name in self.intervals

    def remove_interval(self, name):
        self.intervals.pop(name, None)

    def update_next_time(self, name, seconds):
        seconds_old, function, f_args = self.intervals[name]
        self.intervals[name] = (seconds, function, f_args)

    def run_next(self, name):
        seconds, function, f_args = self.intervals.pop(name)
        function(*f_args)
        return seconds


class FakeObc:
    def __init__(self, robot):
        self._robot = robot

    def set_obc_state(self, state, duration=None):
        self._robot.subsystems.obc_state = state


def make_handler(drive=PRAGYAAN):
    robot = FakeRobot(drive)
    intervals = FakeIntervals()
    return DriveHandler(robot, intervals, FakeObc(robot)), robot, intervals


def test_straight_drive_wheel_speeds_and_times():
    handler, robot, intervals = make_handler()
    handler.drive_robot_straight(0.4, 2.0)
    # Start: 5 % of the distance at half the speed (0.2 m/s).
    assert robot.commands[-1] == ("straight", pytest.approx(0.2 / 0.0972))
    assert intervals.intervals["controlled_drive"][0] == pytest.approx(0.05 * 2.0 / 0.2)
    # Requested speed for 90 % of the distance.
    intervals.run_next("controlled_drive")
    assert robot.commands[-1] == ("straight", pytest.approx(0.4 / 0.0972))
    assert intervals.intervals["controlled_drive"][0] == pytest.approx(0.9 * 2.0 / 0.4)
    # Stop: 5 % of the distance at half the speed, then a stop.
    intervals.run_next("controlled_drive")
    assert robot.commands[-1] == ("straight", pytest.approx(0.2 / 0.0972))
    assert intervals.intervals["stop_robot"][0] == pytest.approx(0.05 * 2.0 / 0.2)


def test_slow_phase_never_faster_than_the_request():
    handler, robot, _ = make_handler()
    handler.drive_robot_straight(0.05, 1.0)
    assert robot.commands[-1] == ("straight", pytest.approx(0.05 / 0.0972))
    handler.drive_robot_straight(-0.6, 1.0)
    assert robot.commands[-1] == ("straight", pytest.approx(-0.3 / 0.0972))


def test_turn_wheel_speed_and_time():
    handler, robot, intervals = make_handler()
    handler.drive_robot_turn(8.0, 90.0)
    expected = math.radians(8.0) * (0.882 / 2) * 2.9 / 0.0972
    assert robot.commands[-1] == ("turn", pytest.approx(expected))
    assert intervals.intervals["stop_robot"][0] == pytest.approx(90.0 / 8.0)


def test_drive_commands_ignored_without_drive_conf():
    handler, robot, intervals = make_handler(drive=None)
    handler.drive_robot_straight(0.2, 1.0)
    handler.drive_robot_turn(8.0, 90.0)
    assert robot.commands == [] and intervals.intervals == {}


@pytest.mark.parametrize(
    "values",
    [
        {"wheel_radius": 0.0, "track_width": 0.5},
        {"wheel_radius": 0.1, "track_width": -0.5},
        {"wheel_radius": 0.1, "track_width": 0.5, "skid_steer_factor": 0.5},
    ],
)
def test_drive_conf_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        DriveConf(**values)


# The real IntervalsHandler, with a fake simulation clock (omni.timeline) and update event stream (omni.kit.app).


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def get_current_time(self):
        return self.now


class FakeSubscription:
    def __init__(self, stream, callback):
        self._stream, self.callback = stream, callback

    def unsubscribe(self):
        if self in self._stream.subscriptions:
            self._stream.subscriptions.remove(self)


class FakeUpdateStream:
    def __init__(self):
        self.subscriptions = []

    def create_subscription_to_pop(self, callback, name=None):
        subscription = FakeSubscription(self, callback)
        self.subscriptions.append(subscription)
        return subscription

    def pump(self):
        for subscription in list(self.subscriptions):  # like Kit: the callbacks of one update event
            subscription.callback(None)


@pytest.fixture
def intervals(monkeypatch):
    clock, stream = FakeClock(), FakeUpdateStream()
    omni, kit = sys.modules["omni"], sys.modules["omni.kit"]
    timeline, app = sys.modules["omni.timeline"], sys.modules["omni.kit.app"]
    monkeypatch.setattr(omni, "timeline", timeline, raising=False)
    monkeypatch.setattr(omni, "kit", kit, raising=False)
    monkeypatch.setattr(kit, "app", app, raising=False)
    monkeypatch.setattr(timeline, "get_timeline_interface", lambda: clock, raising=False)
    monkeypatch.setattr(
        app, "get_app", lambda: types.SimpleNamespace(get_update_event_stream=lambda: stream), raising=False
    )
    from src.tmtc.intervals_handler import IntervalsHandler

    handler = IntervalsHandler()

    def advance(seconds, step=0.1):
        for _ in range(round(seconds / step)):
            clock.now += step
            stream.pump()

    return handler, advance


def test_one_shot_interval_can_schedule_its_successor(intervals):
    handler, advance = intervals
    fired = []

    def first():
        fired.append("first")
        handler.remove_interval("phase")
        handler.add_new_interval(
            name="phase", seconds=1.0, is_repeating=False, execute_immediately=False, function=lambda: fired.append("2")
        )

    handler.add_new_interval(name="phase", seconds=1.0, is_repeating=False, execute_immediately=False, function=first)
    advance(1.2)
    # The successor stays tracked, so that it can be cancelled.
    assert fired == ["first"] and handler.does_exist("phase")
    handler.remove_interval("phase")
    advance(2.0)
    assert fired == ["first"]


def test_a_stopped_drive_does_not_cut_the_next_drive(intervals):
    handler, advance = intervals
    robot = FakeRobot(PRAGYAAN)
    drive = DriveHandler(robot, handler, FakeObc(robot))
    drive.drive_robot_straight(0.2, 2.0)  # 0.5 s slow, 9 s at speed, 0.5 s slow
    advance(2.0)
    drive.stop_robot()
    drive.drive_robot_straight(0.2, 2.0)  # must run its full 10 s
    advance(9.8)
    assert robot.commands[-1][0] == "straight"
    advance(0.4)
    assert robot.commands[-1] == ("stop", 0.0)
