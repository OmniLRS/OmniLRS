import math
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
    # Start position of the flat area tests, heading +y as in cfg/robot/pragyaan.yaml. The spawn of that file, (26, 16.6),
    # is on a slope (roll 2.2 deg, pitch 2.8 deg at rest). On the workshop terrain (seed 42), the rover tilts 1 deg or
    # less here over the 2 m path and the turns: plane through the wheel contacts on the terrain DEM, 2026-11-13.
    "environment.robots_settings.parameters.pose.position=[22.0,13.0,0.42]",
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
# Maximum change per sample of a stopped wheel (0.12 rad/s ~ 1rpm). A stopped rover on a slope keeps a small residual drift (up
# to about 0.05 rad/s at the wheels, accepted on 2026-11-13).
STILL_COUNTS = 20
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
        # Top and side video of the rover (src/environments_wrappers/yamcs/robot_video_recorder.py), next to the logs.
        self.video_path = _log_dir() / "rover_yamcs_sanity.mp4"
        self.proc = subprocess.Popen(
            SIM_COMMAND,
            cwd=REPO_ROOT,
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env={**os.environ, "OMNILRS_ROBOT_VIDEO": str(self.video_path)},
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
        value = self.poll_for(name, expected, timeout)
        if value != expected:
            pytest.fail(f"{name} is {value!r}, expected {expected!r} within {timeout} s")

    def poll_for(self, name, expected, timeout=JOINT_TIMEOUT):
        """`expected` as soon as `name` has this value, otherwise the last value of `name` after `timeout` s."""
        deadline = time.monotonic() + timeout
        value = None
        while time.monotonic() < deadline:
            pval = self._subscription.value_cache.get(name)
            value = None if pval is None else pval.eng_value
            if value == expected:
                return value
            time.sleep(0.5)
        return value

    def record_until_still(self, names, min_samples, timeout=None):
        """
        Every new value of `names` and of the wheel encoder, as {name: [(generation_time, value), ...]}, until the
        wheels stop. Looks for still wheels only after `min_samples` encoder samples (one per second of sim time).
        """
        timeout = MOTION_TIMEOUT_S if timeout is None else timeout
        names = list(dict.fromkeys([*names, ENCODER]))
        last = {}
        for name in names:
            pval = self._subscription.value_cache.get(name)
            last[name] = None if pval is None else pval.generation_time
        samples = {name: [] for name in names}
        half = ENCODER_COUNTS // 2
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for name in names:
                pval = self._subscription.value_cache.get(name)
                if pval is None or pval.eng_value is None:
                    continue
                if last[name] is None or pval.generation_time > last[name]:
                    last[name] = pval.generation_time
                    samples[name].append((pval.generation_time, pval.eng_value))
            encoder = samples[ENCODER]
            if len(encoder) > max(min_samples, 1):
                (_, prev), (_, cur) = encoder[-2], encoder[-1]
                if all(abs((c - p + half) % ENCODER_COUNTS - half) <= STILL_COUNTS for p, c in zip(prev, cur)):
                    return samples
            time.sleep(0.05)
        pytest.fail(f"Wheels still turning {timeout} s after the command ({len(samples[ENCODER])} encoder samples)")

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


@pytest.fixture(autouse=True)
def video_step(request, sim):
    """Writes the name of each test to the steps file of the rover video, which shows it as the current test step."""
    with open(sim.video_path.with_suffix(".steps.log"), "a") as steps:
        steps.write(f"{request.node.name}\n")


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
Joint tests: wheel velocity and solar panel position commands, checked against the measured joint motion, the ground
truth pose and the IMU.

The flat area tests run first, at the start position of SIM_COMMAND (a flat area of the workshop terrain), so that they
measure the drive and not the slopes. The rover drives forward, drives backward to the spawn, turns left, then turns right
to the start heading. The other joint tests run after them and can leave the flat area.
"""

POSE = "/Rover/pose_ground_truth"  # base_link position relative to the lander, world orientation {w, x, y, z}
IMU_ORIENTATION = "/Rover/imu_orientation"  # roll, pitch, yaw in rad (raw SI values; the checks work in deg)
IMU_GYROSCOPE = "/Rover/imu_gyroscope"  # gx, gy, gz in rad/s (raw SI values)

LEFT_BACKWARD_RIGHT_FORWARD = [-1, -1, -1, 1, 1, 1]
ALL_FORWARD = [1] * 6

STRAIGHT_SPEED = 0.2  # m/s
STRAIGHT_DISTANCE = 2.0  # m
TURN_RATE = 8.0  # deg/s; positive turns left (counterclockwise seen from above, REP-103)
TURN_ANGLE = 90.0  # deg
FLAT_MAX_TILT_DEG = 2.0  # roll and pitch of the rover at rest at the spawn
DISTANCE_TOLERANCE = 0.10  # fraction of the commanded distance
DIRECTION_TOLERANCE_DEG = 5.0  # travel direction against the heading (forward) or the opposite heading (backward)
STRAIGHT_YAW_TOLERANCE_DEG = 3.0  # heading change during a straight drive
RETURN_MAX_SHIFT_M = 0.2  # distance to the spawn after the forward and the backward drive
# On the workshop terrain, the turns fall 5 to 8 % short of the flat-ground calibration of skid_steer_factor (83.8 and
# -82.2 deg for 90 deg, 2026-11-13): the turn tolerances are wide.
TURN_ANGLE_TOLERANCE_DEG = 15.0  # also the heading after the left and the right turn, against the start heading
TURN_RATE_TOLERANCE = 0.20  # fraction of the commanded rate
TURN_MAX_SHIFT_M = 0.4  # position change during a turn in place
IMU_TOLERANCE_DEG = 2.0  # IMU roll, pitch and yaw against the ground truth pose
PAIR_MAX_DT_S = 0.1  # IMU and pose samples sent closer in time than this are compared
RAMP_SAMPLES = 2  # encoder and gyroscope samples skipped at the start and at the end of a motion
MOTION_TIMEOUT_S = 180


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


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _rpy_deg(orientation):
    """Roll, pitch and yaw in deg of a {w, x, y, z} quaternion: yaw about z, then pitch about y, then roll about x."""
    w, x, y, z = (orientation[k] for k in ("w", "x", "y", "z"))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


def _yaw_deg(pose):
    return _rpy_deg(pose["orientation"])[2]


def _shift(start, end):
    """Horizontal displacement (dx, dy) in m between two poses."""
    return end["position"]["x"] - start["position"]["x"], end["position"]["y"] - start["position"]["y"]


def _imu_deg(imu):
    """IMU roll, pitch and yaw in deg (the telemetry is in rad)."""
    return tuple(math.degrees(imu[k]) for k in ("roll", "pitch", "yaw"))


def _imu_errors(pose, imu):
    """IMU roll, pitch and yaw minus the roll, pitch and yaw of the ground truth pose, in deg."""
    return tuple(_wrap_deg(i - p) for i, p in zip(_imu_deg(imu), _rpy_deg(pose["orientation"])))


def _paired(samples):
    """(pose, imu) pairs of recorded samples sent within PAIR_MAX_DT_S of each other."""
    poses = samples[POSE]
    pairs = []
    for t_imu, imu in samples[IMU_ORIENTATION]:
        if not poses:
            break
        t_pose, pose = min(poses, key=lambda sample: abs((sample[0] - t_imu).total_seconds()))
        if abs((t_pose - t_imu).total_seconds()) <= PAIR_MAX_DT_S:
            pairs.append((pose, imu))
    return pairs


def _encoder_changes(samples):
    """Signed change of each wheel encoder between consecutive recorded samples (one sample per second of sim time)."""
    half = ENCODER_COUNTS // 2
    encoder = [value for _, value in samples[ENCODER]]
    return [
        [(c - p + half) % ENCODER_COUNTS - half for p, c in zip(prev, cur)] for prev, cur in zip(encoder, encoder[1:])
    ]


def _print_slow_stop(samples, nominal_s, label):
    """If a motion took more than 5 s longer than commanded, print the pose and encoder changes of each second."""
    if len(samples[ENCODER]) <= nominal_s + 5:
        return
    print(f"  {label}: {len(samples[ENCODER])} s until the wheels stopped (commanded {nominal_s:.0f} s)")
    poses = samples[POSE]
    for i in range(1, len(poses)):
        dx, dy = _shift(poses[i - 1][1], poses[i][1])
        print(f"    pose {i:2d}: moved {math.hypot(dx, dy) * 1000:6.1f} mm, yaw {_yaw_deg(poses[i][1]):7.2f} deg")
    for i, changes in enumerate(_encoder_changes(samples), start=1):
        print(f"    encoder {i:2d}: {changes}")


class _Checks:
    """Collects the failed checks, so that one run prints every measurement before the test fails."""

    def __init__(self):
        self.failures = []

    def expect(self, ok, message):
        print(f"  {'ok  ' if ok else 'FAIL'}  {message}")
        if not ok:
            self.failures.append(message)

    def assert_passed(self, name):
        assert not self.failures, f"{name} failures:\n" + "\n".join(self.failures)

    def wheels_turning(self, samples, directions, where):
        """Each wheel turns in its direction (1 forward, -1 backward), without the samples of the start and the stop."""
        changes = _encoder_changes(samples)[RAMP_SAMPLES:-RAMP_SAMPLES]
        ok = bool(changes) and all(c * d >= TURNING_COUNTS for change in changes for c, d in zip(change, directions))
        self.expect(
            ok,
            f"wheels {where}: directions {directions}, encoder changes per second {changes} "
            f"(min {TURNING_COUNTS} counts)",
        )

    def imu_at_rest(self, pose, imu, where):
        errors = _imu_errors(pose, imu)
        self.expect(
            all(abs(e) <= IMU_TOLERANCE_DEG for e in errors),
            f"IMU {where}: roll/pitch/yaw {'/'.join(f'{a:.1f}' for a in _imu_deg(imu))} deg, "
            f"pose {'/'.join(f'{a:.1f}' for a in _rpy_deg(pose['orientation']))} deg "
            f"(error {'/'.join(f'{e:.1f}' for e in errors)}, max {IMU_TOLERANCE_DEG})",
        )

    def imu_in_motion(self, samples, where):
        pairs = _paired(samples)
        self.expect(
            len(pairs) >= 3,
            f"IMU {where}: {len(pairs)} IMU samples sent with a pose sample "
            f"({len(samples[IMU_ORIENTATION])} IMU, {len(samples[POSE])} pose samples)",
        )
        if pairs:
            worst = [max(abs(_imu_errors(pose, imu)[i]) for pose, imu in pairs) for i in range(3)]
            self.expect(
                all(e <= IMU_TOLERANCE_DEG for e in worst),
                f"IMU {where}: largest roll/pitch/yaw error {'/'.join(f'{e:.1f}' for e in worst)} deg "
                f"(max {IMU_TOLERANCE_DEG})",
            )


def _drive_straight(rover, checks, speed, label):
    """
    Drives STRAIGHT_DISTANCE at `speed` (m/s, negative drives backward) and checks the wheels, the distance, the travel
    direction, the heading and the IMU. Returns the ground truth pose at rest after the drive.
    """
    before = rover.next_value(POSE)
    checks.imu_at_rest(before, rover.next_value(IMU_ORIENTATION), f"at rest before the {label}")
    nominal_s = STRAIGHT_DISTANCE / abs(speed)
    try:
        rover.command(DRIVE_STRAIGHT, linear_velocity=speed, distance=STRAIGHT_DISTANCE)
        samples = rover.record_until_still([POSE, IMU_ORIENTATION], min_samples=int(nominal_s) - 2)
    finally:
        _stop(rover)
    time.sleep(STOP_SETTLE_S)
    after = rover.next_value(POSE)
    _print_slow_stop(samples, nominal_s, label)

    forward = speed > 0
    checks.wheels_turning(samples, ALL_FORWARD if forward else [-d for d in ALL_FORWARD], f"during the {label}")
    dx, dy = _shift(before, after)
    distance = math.hypot(dx, dy)
    direction_error = _wrap_deg(math.degrees(math.atan2(dy, dx)) - _yaw_deg(before) - (0.0 if forward else 180.0))
    yaw_change = _wrap_deg(_yaw_deg(after) - _yaw_deg(before))
    checks.expect(
        abs(distance - STRAIGHT_DISTANCE) <= DISTANCE_TOLERANCE * STRAIGHT_DISTANCE,
        f"{label} distance {distance:.3f} m, commanded {STRAIGHT_DISTANCE} m at {speed} m/s "
        f"(max error {DISTANCE_TOLERANCE:.0%}, {len(samples[ENCODER])} s)",
    )
    checks.expect(
        abs(direction_error) <= DIRECTION_TOLERANCE_DEG,
        f"{label} travel direction {direction_error:.1f} deg from the {'heading' if forward else 'opposite heading'} "
        f"(max {DIRECTION_TOLERANCE_DEG})",
    )
    checks.expect(
        abs(yaw_change) <= STRAIGHT_YAW_TOLERANCE_DEG,
        f"{label} heading change {yaw_change:.1f} deg (max {STRAIGHT_YAW_TOLERANCE_DEG})",
    )
    tilts = [_rpy_deg(pose["orientation"])[:2] for _, pose in samples[POSE]]
    if tilts:
        print(
            f"  {label}: largest roll {max(abs(roll) for roll, _ in tilts):.1f} deg, "
            f"largest pitch {max(abs(pitch) for _, pitch in tilts):.1f} deg along the path"
        )
    checks.imu_in_motion(samples, f"during the {label}")
    checks.imu_at_rest(after, rover.next_value(IMU_ORIENTATION), f"at rest after the {label}")
    return after


def _spot_turn(rover, checks, rate, label):
    """
    Turns TURN_ANGLE in place at `rate` (deg/s, positive turns left) and checks the wheels, the turn angle, the position
    change, the gyroscope yaw rate and the IMU. Returns the ground truth pose at rest after the turn. The left and the
    right wheels get separate commands: the wheel check also makes sure that one side command keeps the other side targets.
    """
    before = rover.next_value(POSE)
    checks.imu_at_rest(before, rover.next_value(IMU_ORIENTATION), f"at rest before the {label}")
    nominal_s = TURN_ANGLE / abs(rate)
    try:
        rover.command(DRIVE_TURN, angular_velocity=rate, angle=TURN_ANGLE)
        samples = rover.record_until_still([POSE, IMU_ORIENTATION, IMU_GYROSCOPE], min_samples=int(nominal_s) - 2)
    finally:
        _stop(rover)
    time.sleep(STOP_SETTLE_S)
    after = rover.next_value(POSE)
    _print_slow_stop(samples, nominal_s, label)

    directions = LEFT_BACKWARD_RIGHT_FORWARD if rate > 0 else [-d for d in LEFT_BACKWARD_RIGHT_FORWARD]
    checks.wheels_turning(samples, directions, f"during the {label}")
    start_yaw = _yaw_deg(before)
    trace = " ".join(f"{_wrap_deg(_yaw_deg(pose) - start_yaw):.0f}" for _, pose in samples[POSE])
    rates = " ".join(f"{math.degrees(gyro['gz']):.1f}" for _, gyro in samples[IMU_GYROSCOPE])
    print(f"  {label} trace, yaw from the start each second (deg): {trace}")
    print(f"  {label} trace, gyroscope gz each second (deg/s): {rates}")
    commanded = math.copysign(TURN_ANGLE, rate)
    angle = _wrap_deg(_yaw_deg(after) - start_yaw)
    shift = math.hypot(*_shift(before, after))
    checks.expect(
        abs(angle - commanded) <= TURN_ANGLE_TOLERANCE_DEG,
        f"{label} angle {angle:.1f} deg, commanded {commanded:.1f} deg at {rate} deg/s "
        f"(max error {TURN_ANGLE_TOLERANCE_DEG}, {len(samples[ENCODER])} s)",
    )
    checks.expect(shift <= TURN_MAX_SHIFT_M, f"{label} position change {shift:.3f} m (max {TURN_MAX_SHIFT_M})")
    gz = sorted(math.degrees(gyro["gz"]) for _, gyro in samples[IMU_GYROSCOPE][RAMP_SAMPLES:-RAMP_SAMPLES])
    median = gz[len(gz) // 2] if gz else float("nan")
    checks.expect(
        abs(median - rate) <= TURN_RATE_TOLERANCE * abs(rate),
        f"{label} gyroscope yaw rate (median gz) {median:.3f} deg/s, commanded {rate} deg/s "
        f"(max error {TURN_RATE_TOLERANCE:.0%}, {len(gz)} samples)",
    )
    checks.imu_in_motion(samples, f"during the {label}")
    checks.imu_at_rest(after, rover.next_value(IMU_ORIENTATION), f"at rest after the {label}")
    return after


@pytest.fixture(scope="session")
def start_pose(rover):
    """Ground truth pose of the rover at rest at the spawn, solar panel stowed, before the first drive command."""
    rover.wait_for(SOLAR_PANEL_STATE, "STOWED")
    return rover.next_value(POSE)


def test_start_pose_is_flat(rover, start_pose):
    """The flat area tests measure the drive, not the slopes: the rover starts on flat ground."""
    checks = _Checks()
    print("\nStart pose:")
    roll, pitch, yaw = _rpy_deg(start_pose["orientation"])
    position = start_pose["position"]
    checks.expect(
        abs(roll) <= FLAT_MAX_TILT_DEG and abs(pitch) <= FLAT_MAX_TILT_DEG,
        f"start pose roll {roll:.1f} deg, pitch {pitch:.1f} deg (max {FLAT_MAX_TILT_DEG}), yaw {yaw:.1f} deg, "
        f"x {position['x']:.2f} m, y {position['y']:.2f} m from the lander",
    )
    checks.imu_at_rest(start_pose, rover.next_value(IMU_ORIENTATION), "at rest at the start")
    checks.assert_passed("Start pose")


def test_drive_forward(rover, start_pose):
    checks = _Checks()
    print("\nDrive forward:")
    _drive_straight(rover, checks, STRAIGHT_SPEED, "forward drive")
    checks.assert_passed("Drive forward")


def test_drive_backward(rover, start_pose):
    """Drives back over the path of the forward drive, to the start position."""
    checks = _Checks()
    print("\nDrive backward:")
    after = _drive_straight(rover, checks, -STRAIGHT_SPEED, "backward drive")
    shift = math.hypot(*_shift(start_pose, after))
    checks.expect(
        shift <= RETURN_MAX_SHIFT_M,
        f"distance to the start position after the forward and the backward drive {shift:.3f} m "
        f"(max {RETURN_MAX_SHIFT_M})",
    )
    checks.assert_passed("Drive backward")


def test_spot_turn_left(rover, start_pose):
    checks = _Checks()
    print("\nSpot turn left:")
    _spot_turn(rover, checks, TURN_RATE, "left turn")
    checks.assert_passed("Spot turn left")


def test_spot_turn_right(rover, start_pose):
    """Turns back to the start heading."""
    checks = _Checks()
    print("\nSpot turn right:")
    after = _spot_turn(rover, checks, -TURN_RATE, "right turn")
    heading_error = _wrap_deg(_yaw_deg(after) - _yaw_deg(start_pose))
    checks.expect(
        abs(heading_error) <= TURN_ANGLE_TOLERANCE_DEG,
        f"heading after the left and the right turn {heading_error:.1f} deg from the start heading "
        f"(max {TURN_ANGLE_TOLERANCE_DEG})",
    )
    checks.assert_passed("Spot turn right")


"""
Other joint tests, after the flat area tests: they can leave the flat area.
"""


def test_wheels_drive_straight_then_stop(rover):
    rover.command(DRIVE_STRAIGHT, linear_velocity=0.2, distance=4.0)
    try:
        _assert_wheels_turning(rover.encoder_deltas(), ALL_FORWARD)
    finally:
        _stop(rover)
    time.sleep(STOP_SETTLE_S)
    _assert_wheels_still(rover.encoder_deltas())


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
