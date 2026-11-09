import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest
import zenoh

REPO_ROOT = Path(__file__).resolve().parents[2]
STARTUP_TIMEOUT = int(os.environ.get("OMNILRS_TEST_TIMEOUT", "600"))
TELEMETRY_TIMEOUT = int(os.environ.get("OMNILRS_TELEMETRY_TIMEOUT", "60"))
JOINT_TIMEOUT = int(os.environ.get("OMNILRS_JOINT_TIMEOUT", "30"))

# The test listens here and the sim connects to it (no multicast scouting, which may not work in CI containers).
ZENOH_ENDPOINT = f"tcp/127.0.0.1:{os.environ.get('OMNILRS_ZENOH_PORT', '17447')}"

SIM_COMMAND = [
    "pixi",
    "run",
    "-e",
    "ci",
    "zenoh_lunaryard",
    "rendering.renderer.headless=true",
    # For some reason AWS EKS AMI's duplicate NVIDIA Vulkan ICDs, making one GPU enumerate
    # causing Isaac to think there are two gpus available and then die.
    # Disable and pin.
    "rendering.renderer.multi_gpu=false",
    "rendering.renderer.active_gpu=0",
    # Log every joint command received (checked by test_mixed_command).
    "environment.robots_settings.parameters.zenoh.joint_commands.is_logging=true",
    "mode.sub_log_every_n=1",
]

# The last startup message to appear; once seen, startup is complete.
FINAL_MARKER = "ArticulationTelemetry initialized"

# Fast fail on the following.
FATAL_MARKERS = [
    "Error executing job",
    "ModuleNotFoundError",
    "is not available in environment",
    "carb.crashreporter-breakpad",
    "[ZenohCommandReceiver] listener crashed",
]

IGNORED_WRITE_MARKER = "not allowed while simulation is running"
RESTART_MARKER = "Timeline restarted: resetting the environment and robot"

# Keys of cfg/mode/Zenoh.yaml, for robot=husky.
JOINT_TELEMETRY = "OmniLRS/husky/joint_telemetry"
JOINT_COMMANDS = "OmniLRS/husky/joint_cmd"
GROUND_TRUTH_POSE = "OmniLRS/husky/gt_pose"
CAMERA = "OmniLRS/husky/camera/**"
IMU = "OmniLRS/husky/imu/**"
SIM_STATE = "OmniLRS/sim/is_running"
SIM_PLAY = "OmniLRS/sim/play"
SIM_PAUSE = "OmniLRS/sim/pause"
SIM_STOP = "OmniLRS/sim/stop"

LEFT_WHEELS = ["front_left_wheel", "rear_left_wheel"]
RIGHT_WHEELS = ["front_right_wheel", "rear_right_wheel"]
WHEELS = LEFT_WHEELS + RIGHT_WHEELS

WHEEL_SPEED = 2.0  # rad/s
SPEED_TOLERANCE = 0.5  # rad/s, measured joint velocity vs target
SETTLE_FRAMES = 5  # consecutive joint telemetry frames (10 Hz) that must match


def _log_dir():
    # Artefacts upload dir if available, otherwise a local test dir.
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    log_dir = Path(upload_dir) if upload_dir else Path(__file__).resolve().parent
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir


class SimProcess:
    def __init__(self, zenoh_config_path):
        self.log_path = _log_dir() / "sim_startup_zenoh.log"
        self._log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            SIM_COMMAND,
            cwd=REPO_ROOT,
            env={**os.environ, "ZENOH_SESSION_CONFIG_URI": str(zenoh_config_path)},
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    @property
    def log(self):
        return self.log_path.read_text(errors="replace")

    def wait_for_startup(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            log = self.log
            if FINAL_MARKER in log:
                return
            if any(fatal in log for fatal in FATAL_MARKERS):
                return
            if self.proc.poll() is not None:
                return
            time.sleep(2)

    def wait_for_log(self, marker, count, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.log.count(marker) >= count:
                return
            time.sleep(0.5)
        pytest.fail(f"{marker!r} not logged {count} times within {timeout} s (see {self.log_path.name})")

    def assert_marker(self, marker):
        assert marker in self.log, (
            f"Expected startup marker {marker!r} not found in sim output (see {self.log_path.name})"
        )

    def stop(self):
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                self.proc.wait(timeout=60)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._log_file.close()


class ZenohClient:
    """Zenoh peer the sim connects to: keeps the first and latest sample of each subscribed key and publishes."""

    def __init__(self, session):
        self._session = session
        self._lock = threading.Lock()
        self._samples = {}  # keyexpr -> {"count", "time", "first", "latest"}
        self._subscribers = []

    def subscribe(self, keyexpr, decode=True):
        def on_sample(sample, keyexpr=keyexpr):
            payload = sample.payload.to_bytes()
            value = json.loads(payload) if decode else payload
            with self._lock:
                entry = self._samples.setdefault(keyexpr, {"count": 0, "first": value})
                entry.update(count=entry["count"] + 1, time=time.monotonic(), latest=value)

        self._subscribers.append(self._session.declare_subscriber(keyexpr, on_sample))

    def put(self, keyexpr, value=None):
        self._session.put(keyexpr, json.dumps(value if value is not None else {}))

    def first(self, keyexpr, timeout=TELEMETRY_TIMEOUT):
        self.wait_for(keyexpr, lambda _: True, timeout)
        with self._lock:
            return self._samples[keyexpr]["first"]

    def wait_for(self, keyexpr, predicate, timeout, settle=1, describe=None):
        """
        Wait for `settle` consecutive samples, received from now on, for which `predicate` holds; return the last.
        """
        start = time.monotonic()
        deadline = start + timeout
        seen = None
        matches = 0
        latest = None
        while time.monotonic() < deadline:
            with self._lock:
                entry = self._samples.get(keyexpr)
                count, received, latest = (
                    (entry["count"], entry["time"], entry["latest"]) if entry else (None, None, None)
                )
            if count is not None and count != seen and received > start:
                seen = count
                matches = matches + 1 if predicate(latest) else 0
                if matches >= settle:
                    return latest
            time.sleep(0.02)
        summary = describe(latest) if describe and latest is not None else latest
        pytest.fail(f"No {keyexpr} sample matching within {timeout} s; last: {summary}")

    def close(self):
        for subscriber in self._subscribers:
            subscriber.undeclare()


@pytest.fixture(scope="session")
def zenoh_client():
    config = {"mode": "peer", "listen": {"endpoints": [ZENOH_ENDPOINT]}, "scouting": {"multicast": {"enabled": False}}}
    session = zenoh.open(zenoh.Config.from_json5(json.dumps(config)))
    client = ZenohClient(session)
    for keyexpr in (JOINT_TELEMETRY, GROUND_TRUTH_POSE, SIM_STATE):
        client.subscribe(keyexpr)
    for keyexpr in (CAMERA, IMU):
        client.subscribe(keyexpr, decode=False)
    try:
        yield client
    finally:
        client.close()
        session.close()


# Depends on zenoh_client so the samples the sim publishes at startup (e.g. the spawn pose) are received.
@pytest.fixture(scope="session")
def sim(zenoh_client, tmp_path_factory):
    sim_config = {
        "mode": "peer",
        "connect": {"endpoints": [ZENOH_ENDPOINT]},
        "scouting": {"multicast": {"enabled": False}},
    }
    config_path = tmp_path_factory.mktemp("zenoh") / "sim_session.json5"
    config_path.write_text(json.dumps(sim_config))
    process = SimProcess(config_path)
    try:
        process.wait_for_startup(STARTUP_TIMEOUT)
        yield process
    finally:
        process.stop()


@pytest.fixture(scope="session")
def husky(sim, zenoh_client):
    """The Husky's joint commands and telemetry, with its spawn pose (first ground truth pose received)."""
    return Husky(zenoh_client)


class Husky:
    def __init__(self, client):
        self.client = client
        self.spawn_position = client.first(GROUND_TRUTH_POSE)["position"]

    def command_velocities(self, targets):
        self.client.put(JOINT_COMMANDS, {"joints": {name: {"velocity": value} for name, value in targets.items()}})

    def stop_wheels(self):
        self.command_velocities(dict.fromkeys(WHEELS, 0.0))

    def wait_for_wheels(self, expected, timeout=JOINT_TIMEOUT):
        """Wait until each wheel in `expected` turns at its expected velocity (rad/s); returns the joint frame."""

        def matches(frame):
            joints = frame["joints"]
            return all(abs(joints[name]["dq"] - dq) <= SPEED_TOLERANCE for name, dq in expected.items())

        def describe(frame):
            return {name: round(frame["joints"][name]["dq"], 2) for name in WHEELS}

        return self.client.wait_for(JOINT_TELEMETRY, matches, timeout, settle=SETTLE_FRAMES, describe=describe)

    def wait_for_playing(self, is_playing, timeout=JOINT_TIMEOUT):
        self.client.wait_for(SIM_STATE, lambda state: state.get("is_playing") is is_playing, timeout)

    def distance_from_spawn(self, pose):
        return math.dist(pose["position"][:2], self.spawn_position[:2])


"""
Start up test markers.
"""


def test_app_ready(sim):
    sim.assert_marker("app ready")


def test_terrain_mesh_built(sim):
    sim.assert_marker("mesh update took")


def test_articulation_control_initialized(sim):
    sim.assert_marker("ArticulationControl initialized")


def test_articulation_telemetry_initialized(sim):
    sim.assert_marker("ArticulationTelemetry initialized")


def test_no_fatal_errors(sim):
    for fatal in FATAL_MARKERS:
        assert fatal not in sim.log, f"Fatal marker {fatal!r} found in sim output"


def test_sim_process_alive(sim):
    assert sim.proc.poll() is None, f"Sim process exited (rc={sim.proc.returncode})"


"""
Telemetry tests.
"""


def test_sim_state_running_and_playing(sim, zenoh_client):
    zenoh_client.wait_for(SIM_STATE, lambda state: state == {"is_running": True, "is_playing": True}, TELEMETRY_TIMEOUT)


def test_joint_telemetry_has_wheel_joints(sim, zenoh_client):
    frame = zenoh_client.wait_for(JOINT_TELEMETRY, lambda _: True, TELEMETRY_TIMEOUT)
    for name in WHEELS:
        assert set(frame["joints"].get(name, {})) == {"q", "dq", "effort"}, f"{name} missing: {frame['joints']}"


def test_ground_truth_pose_has_position_and_orientation(sim, zenoh_client):
    pose = zenoh_client.wait_for(GROUND_TRUTH_POSE, lambda _: True, TELEMETRY_TIMEOUT)
    assert len(pose["position"]) == 3
    assert len(pose["orientation_xyzw"]) == 4


def test_camera_published(sim, zenoh_client):
    zenoh_client.wait_for(CAMERA, lambda image: len(image) > 0, TELEMETRY_TIMEOUT)


def test_imu_published(sim, zenoh_client):
    zenoh_client.wait_for(IMU, lambda reading: len(reading) > 0, TELEMETRY_TIMEOUT)


"""
Joint tests: velocity, mixed and partial commands, checked against the measured joint velocities.
"""


def test_wheels_velocity_then_stop(husky):
    start = husky.client.wait_for(GROUND_TRUTH_POSE, lambda _: True, TELEMETRY_TIMEOUT)["position"]
    husky.command_velocities(dict.fromkeys(WHEELS, WHEEL_SPEED))
    try:
        husky.wait_for_wheels(dict.fromkeys(WHEELS, WHEEL_SPEED))
        husky.client.wait_for(GROUND_TRUTH_POSE, lambda pose: math.dist(pose["position"][:2], start[:2]) > 0.1, 10)
    finally:
        husky.stop_wheels()
    husky.wait_for_wheels(dict.fromkeys(WHEELS, 0.0))


def test_commanding_some_wheels_keeps_the_others(husky):
    try:
        husky.command_velocities(dict.fromkeys(LEFT_WHEELS, WHEEL_SPEED))
        husky.wait_for_wheels({**dict.fromkeys(LEFT_WHEELS, WHEEL_SPEED), **dict.fromkeys(RIGHT_WHEELS, 0.0)})
        husky.command_velocities(dict.fromkeys(RIGHT_WHEELS, WHEEL_SPEED))
        husky.wait_for_wheels(dict.fromkeys(WHEELS, WHEEL_SPEED))  # left targets kept
        husky.command_velocities(dict.fromkeys(LEFT_WHEELS, 0.0))
        husky.wait_for_wheels({**dict.fromkeys(LEFT_WHEELS, 0.0), **dict.fromkeys(RIGHT_WHEELS, WHEEL_SPEED)})
    finally:
        husky.stop_wheels()
    husky.wait_for_wheels(dict.fromkeys(WHEELS, 0.0))


def test_mixed_command(husky, sim):
    # Husky wheel drives are velocity drives (stiffness 100, damping 1e8): a position target does not visibly move a
    # wheel, so the right wheels get their current angle. The mixed message must move the left wheels and keep the
    # right wheels' velocity targets; the receiver log shows both parts were accepted.
    frame = husky.client.wait_for(JOINT_TELEMETRY, lambda _: True, TELEMETRY_TIMEOUT)
    joints = {name: {"velocity": WHEEL_SPEED} for name in LEFT_WHEELS}
    joints.update({name: {"position": frame["joints"][name]["q"]} for name in RIGHT_WHEELS})
    husky.client.put(JOINT_COMMANDS, {"joints": joints})
    try:
        husky.wait_for_wheels({**dict.fromkeys(LEFT_WHEELS, WHEEL_SPEED), **dict.fromkeys(RIGHT_WHEELS, 0.0)})
    finally:
        husky.stop_wheels()
    husky.wait_for_wheels(dict.fromkeys(WHEELS, 0.0))
    sim.wait_for_log("ok=True position=2 velocity=2 effort=0", 1, 10)


"""
Simulation control tests: pause keeps the simulation state, stop then play restarts it.
"""


def test_pause_then_play_resumes(husky, sim):
    restarts = sim.log.count(RESTART_MARKER)
    husky.command_velocities(dict.fromkeys(WHEELS, WHEEL_SPEED))
    try:
        husky.wait_for_wheels(dict.fromkeys(WHEELS, WHEEL_SPEED))
        husky.client.put(SIM_PAUSE)
        husky.wait_for_playing(False)
        husky.client.put(SIM_PLAY)
        husky.wait_for_playing(True)
        husky.wait_for_wheels(dict.fromkeys(WHEELS, WHEEL_SPEED))  # targets kept through the pause
    finally:
        husky.stop_wheels()
    husky.wait_for_wheels(dict.fromkeys(WHEELS, 0.0))
    assert sim.log.count(RESTART_MARKER) == restarts, "A pause then play restarted the simulation"


def test_stop_then_play_restarts(husky, sim):
    restarts = sim.log.count(RESTART_MARKER)
    husky.command_velocities(dict.fromkeys(WHEELS, WHEEL_SPEED))
    try:
        husky.client.wait_for(GROUND_TRUTH_POSE, lambda pose: husky.distance_from_spawn(pose) > 0.5, JOINT_TIMEOUT)
    finally:
        husky.stop_wheels()

    husky.client.put(SIM_STOP)
    husky.wait_for_playing(False)
    husky.client.put(SIM_PLAY)
    husky.wait_for_playing(True)

    # The restart handling runs once: the Husky is back at its spawn pose, stopped, and takes new commands.
    sim.wait_for_log(RESTART_MARKER, restarts + 1, JOINT_TIMEOUT)
    husky.client.wait_for(GROUND_TRUTH_POSE, lambda pose: husky.distance_from_spawn(pose) < 0.3, JOINT_TIMEOUT)
    husky.wait_for_wheels(dict.fromkeys(WHEELS, 0.0))
    husky.command_velocities(dict.fromkeys(WHEELS, WHEEL_SPEED))
    try:
        husky.wait_for_wheels(dict.fromkeys(WHEELS, WHEEL_SPEED))
    finally:
        husky.stop_wheels()
    husky.wait_for_wheels(dict.fromkeys(WHEELS, 0.0))
    assert sim.log.count(RESTART_MARKER) == restarts + 1, "The restart handling ran more than once"


def test_no_ignored_joint_writes(sim):
    # Runs after the joint tests: PhysX ignores (and logs) joint drive writes made while it is stepping.
    assert IGNORED_WRITE_MARKER not in sim.log, f"PhysX ignored joint writes (see {sim.log_path.name})"
