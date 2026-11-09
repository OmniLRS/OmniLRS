import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest
import yaml

from src.configurations.robot_camera_confs import cameras_of_type, parse_cameras
from src.tmtc import yamcs_naming

REPO_ROOT = Path(__file__).resolve().parents[2]
STARTUP_TIMEOUT = int(os.environ.get("OMNILRS_TEST_TIMEOUT", "600"))
TELEMETRY_TIMEOUT = int(os.environ.get("OMNILRS_TELEMETRY_TIMEOUT", "120"))
JOINT_TIMEOUT = int(os.environ.get("OMNILRS_JOINT_TIMEOUT", "60"))

YAMCS_ADDRESS = os.environ.get("YAMCS_ADDRESS", "localhost:8090")
YAMCS_INSTANCE = "husky"  # mode.instance_conf.instance of the yamcs-husky task
YAMCS_PROCESSOR = os.environ.get("YAMCS_PROCESSOR", "realtime")

MC_DIR_CANDIDATES = [
    os.environ.get("YAMCS_HUSKY_MC_DIR"),
    "/workspace/yamcs-mission-control-husky",
    str(REPO_ROOT.parent / "yamcs-mission-control-husky"),
]

# Husky in Yamcs mode: environment=lunaryard_20m robot=husky controller=husky-controller (pixi task yamcs-husky).
SIM_COMMAND = [
    "pixi",
    "run",
    "--environment",
    "ci",
    "yamcs-husky",
    "rendering.renderer.headless=true",
    # For some reason AWS EKS AMI's duplicate NVIDIA Vulkan ICDs, making one GPU enumerate
    # causing Isaac to think there are two gpus available and then die.
    # Disable and pin.
    "rendering.renderer.multi_gpu=false",
    "rendering.renderer.active_gpu=0",
]

ROBOT = "husky"
CONTROLLER = "husky-controller"

# The last startup message to appear; once seen, startup is complete.
FINAL_MARKER = "ArticulationTelemetry initialized"

# Fast fail on the following.
FATAL_MARKERS = [
    "Error executing job",
    "ModuleNotFoundError",
    "is not available in environment",
    "carb.crashreporter-breakpad",
]

# Command tests: Yamcs commands (cfg/controller/husky-controller.yaml) and the telemetry they act on.
GO_NOGO = "/Rover/system/go_nogo_command"
GO_NOGO_STATE = "/Rover/go_nogo"
DRIVE_STRAIGHT = "/Rover/motor/drive_straight"
DRIVE_TURN = "/Rover/motor/drive_turn"
CAMERA_STREAMING = "/Rover/camera/camera_streaming"
CAMERA_CAPTURE_HIGH = "/Rover/camera/camera_capture_high"
CAMERA_CAPTURE_DEPTH = "/Rover/camera/camera_capture_depth"
# Wheel joint angles, sent every 1 s of sim time: left wheels (front, rear), then right wheels (front, rear).
ENCODER = "/Rover/motor_encoder"
ENCODER_COUNTS = 1024  # per turn
# The drive commands below set wheel velocities of about 2 rad/s (straight) and 1.5 rad/s (turn), 330 and 240 counts
# per sample (under half a turn, so the signed change between samples is unambiguous).
TURNING_COUNTS = 50  # minimum change per sample of a turning wheel
STILL_COUNTS = 10  # maximum change per sample of a stopped wheel
STOP_SETTLE_S = 2.0  # time for the rover to slow down after a stop command, before the wheels are checked
IGNORED_WRITE_MARKER = "not allowed while simulation is running"


def _log_dir():
    # Artefacts upload dir if available, otherwise a local test dir.
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    log_dir = Path(upload_dir) if upload_dir else Path(__file__).resolve().parent
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir


def _port_open(address):
    host, port = address.rsplit(":", 1)
    try:
        with socket.create_connection((host, int(port)), timeout=2):
            return True
    except OSError:
        return False


def _robot_cameras():
    with open(REPO_ROOT / "cfg" / "robot" / f"{ROBOT}.yaml") as f:
        return parse_cameras(yaml.safe_load(f)["robots_settings"]["parameters"]["camera"])


def _controller_parameters():
    with open(REPO_ROOT / "cfg" / "controller" / f"{CONTROLLER}.yaml") as f:
        return yaml.safe_load(f)["robots_settings"]["yamcs_tmtc"]["parameters"]


class YamcsServer:
    def __init__(self, mc_dir):
        self.log_path = _log_dir() / "yamcs_server.log"
        self._log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            ["./mvnw", "-B", "yamcs:run"],
            cwd=Path(mc_dir) / "yamcs-server",
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def wait_until_reachable(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if _port_open(YAMCS_ADDRESS):
                return True
            if self.proc.poll() is not None:
                return False
            time.sleep(2)
        return False

    def stop(self):
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                self.proc.wait(timeout=30)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._log_file.close()


class SimProcess:
    def __init__(self):
        self.log_path = _log_dir() / "sim_startup.log"
        self._log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            SIM_COMMAND,
            cwd=REPO_ROOT,
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


class FreshTelemetry:
    """Parameter values received from this sim run only, waited for up to a shared deadline."""

    def __init__(self, names, subscription, timeout):
        self.names = names
        self._subscription = subscription
        self._deadline = time.monotonic() + timeout

    def _wait(self, match):
        while True:
            hits = {
                name: pval.eng_value
                for name, pval in list(self._subscription.value_cache.items())
                if match(name) and pval.eng_value is not None
            }
            if hits or time.monotonic() >= self._deadline:
                return hits
            time.sleep(1)

    def get(self, name):
        assert name in self.names, f"{name!r} is not defined in the MDB of instance {YAMCS_INSTANCE!r}"
        return self._wait(lambda n: n == name).get(name)


class Rover:
    """Sends Yamcs commands to the rover and reads the telemetry they change."""

    def __init__(self, processor, subscription):
        self._processor = processor
        self._subscription = subscription

    def command(self, name, **args):
        self._processor.issue_command(name, args=args)

    def next_value(self, name, timeout=JOINT_TIMEOUT):
        """The first value of `name` the sim sends after this call."""
        pval = self._subscription.value_cache.get(name)
        after = None if pval is None else pval.generation_time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pval = self._subscription.value_cache.get(name)
            if pval is not None and pval.eng_value is not None and (after is None or pval.generation_time > after):
                return pval.eng_value
            time.sleep(0.1)
        pytest.fail(f"No new {name} value within {timeout} s")

    def wait_for(self, name, expected, timeout=JOINT_TIMEOUT):
        deadline = time.monotonic() + timeout
        value = None
        while time.monotonic() < deadline:
            pval = self._subscription.value_cache.get(name)
            value = None if pval is None else pval.eng_value
            if value == expected:
                return
            time.sleep(0.5)
        pytest.fail(f"{name} is {value!r}, expected {expected!r} within {timeout} s")

    def encoder_deltas(self, samples=2):
        """
        Signed change of each wheel encoder between consecutive samples, for `samples` sample intervals.
        The first sample is skipped so a command just sent has taken effect.
        """
        half = ENCODER_COUNTS // 2
        self.next_value(ENCODER)
        prev = self.next_value(ENCODER)
        deltas = []
        for _ in range(samples):
            cur = self.next_value(ENCODER)
            deltas.append([(c - p + half) % ENCODER_COUNTS - half for p, c in zip(prev, cur)])
            prev = cur
        return deltas


@pytest.fixture(scope="session")
def yamcs_server():
    # In case you're using an already-running server
    if _port_open(YAMCS_ADDRESS):
        yield None
        return

    mc_dir = next((d for d in MC_DIR_CANDIDATES if d and Path(d).is_dir()), None)
    if mc_dir is None:
        pytest.skip(
            "yamcs-mission-control-husky checkout not found "
            "(set YAMCS_HUSKY_MC_DIR) and no server running at " + YAMCS_ADDRESS
        )

    server = YamcsServer(mc_dir)
    try:
        assert server.wait_until_reachable(STARTUP_TIMEOUT), (
            f"Yamcs server did not come up at {YAMCS_ADDRESS} (see {server.log_path.name})"
        )
        yield server
    finally:
        server.stop()


@pytest.fixture(scope="session")
def processor(yamcs_server):
    from yamcs.client import YamcsClient

    return YamcsClient(YAMCS_ADDRESS).get_processor(instance=YAMCS_INSTANCE, processor=YAMCS_PROCESSOR)


@pytest.fixture(scope="session")
def subscription(yamcs_server):
    from yamcs.client import YamcsClient

    client = YamcsClient(YAMCS_ADDRESS)
    mdb = client.get_mdb(instance=YAMCS_INSTANCE)
    names = [p.qualified_name for p in mdb.list_parameters() if not p.qualified_name.startswith("/yamcs")]
    assert names, f"No parameters defined in the MDB of instance {YAMCS_INSTANCE!r}"

    processor = client.get_processor(instance=YAMCS_INSTANCE, processor=YAMCS_PROCESSOR)
    # Enforce no cached values because a reused Yamcs server may hold values from an earlier sim run.
    sub = processor.create_parameter_subscription(names, send_from_cache=False)
    try:
        yield names, sub
    finally:
        print("\nTelemetry received from this sim run:")
        for name, pval in sorted(sub.value_cache.items()):
            print(f"  {name} = {pval.eng_value}")
        sub.cancel()


# Depends on subscription so values the sim publishes once at startup are captured.
@pytest.fixture(scope="session")
def sim(yamcs_server, subscription):
    process = SimProcess()
    try:
        process.wait_for_startup(STARTUP_TIMEOUT)
        yield process
    finally:
        process.stop()


@pytest.fixture(scope="session")
def telemetry(sim, subscription):
    names, sub = subscription
    return FreshTelemetry(names, sub, TELEMETRY_TIMEOUT)


@pytest.fixture(scope="session")
def cameras():
    """The robot yaml cameras, which define the camera buckets, telemetry and command arguments."""
    return _robot_cameras()


# Requested by the command tests only, which run last: the telemetry tests expect the startup state (streaming off).
@pytest.fixture(scope="session")
def rover(sim, processor, subscription):
    """The Husky ready to drive and capture: it starts in GO and has no power commands."""
    return Rover(processor, subscription[1])


"""
Start up test markers.
"""


def test_sim_connected_to_yamcs(sim):
    sim.assert_marker("is reachable. Continuing startup.")


def test_app_ready(sim):
    sim.assert_marker("app ready")


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


PARAMETERS = _controller_parameters()


@pytest.mark.parametrize("name", list(PARAMETERS.values()), ids=list(PARAMETERS))
def test_telemetry_of_controller_yaml(telemetry, name):
    assert telemetry.get(name) is not None, f"No {name} value from this sim run"


def test_telemetry_go_nogo_is_go(telemetry):
    assert telemetry.get(GO_NOGO_STATE) == "GO"


def test_telemetry_pose_ground_truth_has_position_and_orientation(telemetry):
    pose = telemetry.get("/Rover/pose_ground_truth")
    assert pose is not None
    assert "position" in pose
    assert "orientation" in pose


def test_telemetry_motor_encoder_has_4_wheels(telemetry):
    encoder = telemetry.get(ENCODER)
    assert encoder is not None and len(encoder) == 4, f"Expected 4 wheel angles, got {encoder}"


def test_telemetry_image_metadata_for_every_camera_and_kind(telemetry, cameras):
    # OmniLRS uploads a "no data" image to every bucket at startup, which sets <meta path>/number etc.
    paths = [path for *_, path in yamcs_naming.camera_buckets(cameras)]
    assert paths, f"No camera image buckets computed for robot {ROBOT!r}"
    missing = [path for path in paths if telemetry.get(f"{path}/number") is None]
    assert not missing, f"No image metadata from this sim run for {missing}"


def test_telemetry_camera_streaming_off_at_startup(telemetry, cameras):
    states = {
        name: telemetry.get(yamcs_naming.streaming_param(name, field="state"))
        for name in cameras_of_type(cameras, "rgba")
    }
    assert states and all(state == "OFF" for state in states.values()), f"Streaming states at startup: {states}"


"""
Camera command tests: each image the sim uploads sets the image metadata of its bucket (number increments).
"""


def test_camera_capture_high_every_rgba_camera(rover, telemetry, cameras):
    for name in cameras_of_type(cameras, "rgba"):
        number = f"{yamcs_naming.meta_path(name, 'oncommand')}/number"
        before = telemetry.get(number)
        rover.command(CAMERA_CAPTURE_HIGH, camera_name=name)
        after = rover.next_value(number)
        assert after > before, f"{name}: on-command image number {after} after capture, {before} before"


def test_camera_capture_depth_every_depth_camera(rover, telemetry, cameras):
    depth_cameras = cameras_of_type(cameras, "depth")
    assert depth_cameras, f"No depth camera in the {ROBOT} robot yaml"
    for name in depth_cameras:
        number = f"{yamcs_naming.meta_path(name, 'depth')}/number"
        before = telemetry.get(number)
        rover.command(CAMERA_CAPTURE_DEPTH, camera_name=name)
        after = rover.next_value(number)
        assert after > before, f"{name}: depth image number {after} after capture, {before} before"


def test_camera_streaming_start_then_stop_every_rgba_camera(rover, telemetry, cameras):
    names = cameras_of_type(cameras, "rgba")
    before = {name: telemetry.get(f"{yamcs_naming.meta_path(name, 'streaming')}/number") for name in names}
    for name in names:
        rover.command(CAMERA_STREAMING, camera_name=name, action="START")
    try:
        for name in names:
            rover.wait_for(yamcs_naming.streaming_param(name, field="state"), "ON")
            after = rover.next_value(f"{yamcs_naming.meta_path(name, 'streaming')}/number")
            assert after > before[name], f"{name}: streaming image number {after} after START, {before[name]} before"
    finally:
        for name in names:
            rover.command(CAMERA_STREAMING, camera_name=name, action="STOP")
    for name in names:
        rover.wait_for(yamcs_naming.streaming_param(name, field="state"), "OFF")


"""
Joint tests: wheel velocity commands, checked against the measured joint motion.
"""

LEFT_BACKWARD_RIGHT_FORWARD = [-1, -1, 1, 1]
ALL_FORWARD = [1] * 4


def _assert_wheels_turning(deltas, directions):
    for delta in deltas:
        for wheel, (change, direction) in enumerate(zip(delta, directions)):
            assert change * direction >= TURNING_COUNTS, (
                f"Wheel {wheel} not turning in direction {direction}: encoder changes per sample {deltas}"
            )


def _assert_wheels_still(deltas):
    assert all(abs(change) <= STILL_COUNTS for delta in deltas for change in delta), (
        f"Wheels still turning after stop: encoder changes per sample {deltas}"
    )


def _stop(rover):
    rover.command(DRIVE_STRAIGHT, linear_velocity=0.0, distance=0.0)


def test_wheels_drive_straight_then_stop(rover):
    rover.command(DRIVE_STRAIGHT, linear_velocity=0.2, distance=4.0)
    try:
        _assert_wheels_turning(rover.encoder_deltas(), ALL_FORWARD)
    finally:
        _stop(rover)
    time.sleep(STOP_SETTLE_S)
    _assert_wheels_still(rover.encoder_deltas())


def test_wheels_turn_sides_independently(rover):
    # Left and right wheels are commanded separately: the right side command must keep the left side targets.
    # 16 deg/s for 10 s: wheel speed 16 deg/s * width / 2 * turn_speed_coef (cfg/robot/husky.yaml), about 1.5 rad/s.
    rover.command(DRIVE_TURN, angular_velocity=16.0, angle=160.0)
    try:
        _assert_wheels_turning(rover.encoder_deltas(), LEFT_BACKWARD_RIGHT_FORWARD)
    finally:
        _stop(rover)


def test_nogo_stops_and_blocks_driving(rover):
    rover.command(DRIVE_STRAIGHT, linear_velocity=0.2, distance=4.0)
    try:
        _assert_wheels_turning(rover.encoder_deltas(), ALL_FORWARD)
        rover.command(GO_NOGO, decision="NOGO")
        rover.wait_for(GO_NOGO_STATE, "NOGO")
        time.sleep(STOP_SETTLE_S)  # NOGO stops the rover
        _assert_wheels_still(rover.encoder_deltas())
        rover.command(DRIVE_STRAIGHT, linear_velocity=0.2, distance=4.0)
        _assert_wheels_still(rover.encoder_deltas())
    finally:
        # Back to the startup state: GO first, since the stop command is a drive command too.
        rover.command(GO_NOGO, decision="GO")
        rover.wait_for(GO_NOGO_STATE, "GO")
        _stop(rover)


def test_no_ignored_joint_writes(sim):
    # Runs after the joint tests: PhysX ignores (and logs) joint drive writes made while it is stepping.
    assert IGNORED_WRITE_MARKER not in sim.log, f"PhysX ignored joint writes (see {sim.log_path.name})"


def test_durations_within_baseline(check_durations, sim):
    # Keep this test last: it compares the durations of this run with durations_baseline.yaml (Artefacts infra only).
    check_durations(sim.log)
