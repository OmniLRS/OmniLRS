import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest
import yaml

from src.configurations.robot_camera_confs import parse_cameras
from src.tmtc import yamcs_naming

REPO_ROOT = Path(__file__).resolve().parents[2]
STARTUP_TIMEOUT = int(os.environ.get("OMNILRS_TEST_TIMEOUT", "600"))
TELEMETRY_TIMEOUT = int(os.environ.get("OMNILRS_TELEMETRY_TIMEOUT", "120"))
JOINT_TIMEOUT = int(os.environ.get("OMNILRS_JOINT_TIMEOUT", "60"))

YAMCS_ADDRESS = os.environ.get("YAMCS_ADDRESS", "localhost:8090")
YAMCS_INSTANCE = os.environ.get("YAMCS_INSTANCE", "workshop")
YAMCS_PROCESSOR = os.environ.get("YAMCS_PROCESSOR", "realtime")

MC_DIR_CANDIDATES = [
    os.environ.get("YAMCS_MC_DIR"),
    "/workspace/yamcs-mission-control-pragyaan",
    str(REPO_ROOT.parent / "yamcs-mission-control-pragyaan"),
]

SIM_COMMAND = [
    "pixi",
    "run",
    "--environment",
    "ci",
    "yamcs",
    "rendering.renderer.headless=true",
    # For some reason AWS EKS AMI's duplicate NVIDIA Vulkan ICDs, making one GPU enumerate
    # causing Isaac to think there are two gpus available and then die.
    # Disable and pin.
    "rendering.renderer.multi_gpu=false",
    "rendering.renderer.active_gpu=0",
]

# Robot of the sim task (robot=pragyaan); its yaml camera list defines the camera image buckets.
ROBOT = "pragyaan"

# The last startup message to appear; once seen, startup is complete.
FINAL_MARKER = "ArticulationTelemetry initialized"

# Fast fail on the following.
FATAL_MARKERS = [
    "Error executing job",
    "ModuleNotFoundError",
    "is not available in environment",
    "carb.crashreporter-breakpad",
]

# Joint tests: Yamcs commands (cfg/controller/pragyaan-controller.yaml) and the telemetry they act on.
DRIVE_STRAIGHT = "/Rover/motor/drive_straight"
DRIVE_TURN = "/Rover/motor/drive_turn"
SOLAR_PANEL = "/Rover/system/deploy_solar_panel"
POWER_ELECTRONICS = "/Rover/power/power_electronics"
GO_NOGO = "/Rover/system/go_nogo_command"
SOLAR_PANEL_STATE = "/Rover/solar_panel_state"  # DEPLOYED/STOWED once the measured panel angle is within 10 deg
# Wheel joint angles, sent every 1 s of sim time: left wheels (fl, ml, bl), then right wheels (fr, mr, br).
ENCODER = "/Rover/motor_encoder"
ENCODER_COUNTS = 1024  # per turn
# The drive commands below set wheel velocities of about 2 rad/s, ~330 counts per sample (under half a turn, so the
# signed change between samples is unambiguous).
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

    def with_prefix(self, prefix):
        return self._wait(lambda n: n.startswith(prefix))


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
            "yamcs-mission-control-pragyaan checkout not found "
            "(set YAMCS_MC_DIR) and no server running at " + YAMCS_ADDRESS
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
def camera_image_meta_paths():
    """Image metadata space system of every (camera, kind) of the robot yaml, named by src/tmtc/yamcs_naming.py."""
    with open(REPO_ROOT / "cfg" / "robot" / f"{ROBOT}.yaml") as f:
        cameras = yaml.safe_load(f)["robots_settings"]["parameters"]["camera"]
    return [path for *_, path in yamcs_naming.camera_buckets(parse_cameras(cameras))]


# Requested by the joint tests only, which run last: the telemetry tests expect the startup state (e.g. NOGO).
@pytest.fixture(scope="session")
def rover(sim, processor, subscription):
    """The rover ready to drive: motor controller on and GO."""
    rover = Rover(processor, subscription[1])
    rover.command(POWER_ELECTRONICS, subsystem_id="MOTOR_CONTROLLER", power_state="ON")
    rover.command(GO_NOGO, decision="GO")
    rover.wait_for("/Rover/go_nogo", "GO")
    return rover


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


def test_telemetry_battery_charge_exists(telemetry):
    assert telemetry.get("/Rover/battery_charge") is not None


def test_telemetry_battery_voltage_exists(telemetry):
    assert telemetry.get("/Rover/battery_voltage") is not None


def test_telemetry_camera_exists(telemetry):
    assert telemetry.with_prefix("/Rover/camera/")


def test_telemetry_image_metadata_for_every_camera_and_kind(telemetry, camera_image_meta_paths):
    # OmniLRS uploads a "no data" image to every bucket at startup, which sets <meta path>/number etc.
    assert camera_image_meta_paths, f"No camera image buckets computed for robot {ROBOT!r}"
    missing = [path for path in camera_image_meta_paths if telemetry.get(f"{path}/number") is None]
    assert not missing, f"No image metadata from this sim run for {missing}"


def test_telemetry_go_nogo_is_nogo(telemetry):
    assert telemetry.get("/Rover/go_nogo") == "NOGO"


def test_telemetry_motor_current_not_empty(telemetry):
    assert telemetry.get("/Rover/motor_current"), "motor_current missing or empty list"


def test_telemetry_payload_exists(telemetry):
    assert telemetry.with_prefix("/Rover/payload/")


def test_telemetry_pose_ground_truth_has_position_and_orientation(telemetry):
    pose = telemetry.get("/Rover/pose_ground_truth")
    assert pose is not None
    assert "position" in pose
    assert "orientation" in pose


def test_telemetry_has_7_temperatures(telemetry):
    temperatures = [name for name in telemetry.names if name.startswith("/Rover/temperature_")]
    assert len(temperatures) == 7, f"Expected 7 temperature parameters in the MDB, got {temperatures}"
    missing = [name for name in temperatures if telemetry.get(name) is None]
    assert not missing, f"No value from this sim run for {missing}"


def test_telemetry_total_current_in_exists(telemetry):
    assert telemetry.get("/Rover/total_current_in") is not None


def test_telemetry_total_current_out_exists(telemetry):
    assert telemetry.get("/Rover/total_current_out") is not None


def test_telemetry_obc_state_exists(telemetry):
    assert telemetry.get("/Rover/obc_state") is not None


"""
Joint tests: wheel velocity and solar panel position commands, checked against the measured joint motion.
"""

LEFT_BACKWARD_RIGHT_FORWARD = [-1, -1, -1, 1, 1, 1]
ALL_FORWARD = [1] * 6


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
    rover.command(DRIVE_TURN, angular_velocity=8.0, angle=40.0)
    try:
        _assert_wheels_turning(rover.encoder_deltas(), LEFT_BACKWARD_RIGHT_FORWARD)
    finally:
        _stop(rover)


def test_solar_panel_deploy_and_stow(rover):
    rover.wait_for(SOLAR_PANEL_STATE, "STOWED")
    rover.command(DRIVE_STRAIGHT, linear_velocity=0.2, distance=4.0)
    rover.command(SOLAR_PANEL, deployment="DEPLOY")
    try:
        # The solar panel position command must keep the wheel velocity targets.
        _assert_wheels_turning(rover.encoder_deltas(), ALL_FORWARD)
    finally:
        _stop(rover)
    rover.wait_for(SOLAR_PANEL_STATE, "DEPLOYED")
    rover.command(SOLAR_PANEL, deployment="STOW")
    rover.wait_for(SOLAR_PANEL_STATE, "STOWED")


def test_no_ignored_joint_writes(sim):
    # Runs after the joint tests: PhysX ignores (and logs) joint drive writes made while it is stepping.
    assert IGNORED_WRITE_MARKER not in sim.log, f"PhysX ignored joint writes (see {sim.log_path.name})"
