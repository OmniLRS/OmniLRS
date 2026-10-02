import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
STARTUP_TIMEOUT = int(os.environ.get("OMNILRS_TEST_TIMEOUT", "600"))
TELEMETRY_TIMEOUT = int(os.environ.get("OMNILRS_TELEMETRY_TIMEOUT", "120"))

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

# The last startup message to appear; once seen, startup is complete.
FINAL_MARKER = "ArticulationTelemetry initialized"

# Fast fail on the following.
FATAL_MARKERS = [
    "Error executing job",
    "ModuleNotFoundError",
    "is not available in environment",
    "carb.crashreporter-breakpad",
]


def _log_dir():
    # Artefacts upload dir if available, otherwise a local test dir.
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    log_dir = Path(upload_dir) if upload_dir else REPO_ROOT / "test"
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
def sim(yamcs_server):
    process = SimProcess()
    try:
        process.wait_for_startup(STARTUP_TIMEOUT)
        yield process
    finally:
        process.stop()


@pytest.fixture(scope="session")
def telemetry(sim):
    from yamcs.client import YamcsClient

    client = YamcsClient(YAMCS_ADDRESS)
    mdb = client.get_mdb(instance=YAMCS_INSTANCE)
    processor = client.get_processor(instance=YAMCS_INSTANCE, processor=YAMCS_PROCESSOR)

    names = [p.qualified_name for p in mdb.list_parameters() if not p.qualified_name.startswith("/yamcs")]
    assert names, f"No parameters defined in the MDB of instance {YAMCS_INSTANCE!r}"

    published = {}
    deadline = time.monotonic() + TELEMETRY_TIMEOUT
    while time.monotonic() < deadline:
        values = processor.get_parameter_values(names, from_cache=True)
        before = len(published)
        published.update(
            {
                name: pval.eng_value
                for name, pval in zip(names, values)
                if pval is not None and pval.eng_value is not None
            }
        )
        if published and len(published) == before:
            break
        time.sleep(2)

    print("\nPublished rover telemetry parameters:")
    for name in sorted(published):
        print(f"  {name} = {published[name]}")
    return published


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
    assert any(name.startswith("/Rover/camera/") for name in telemetry)


def test_telemetry_go_nogo_is_nogo(telemetry):
    assert telemetry.get("/Rover/go_nogo") == "NOGO"


def test_telemetry_motor_current_not_empty(telemetry):
    assert telemetry.get("/Rover/motor_current"), "motor_current missing or empty list"


def test_telemetry_payload_exists(telemetry):
    assert any(name.startswith("/Rover/payload/") for name in telemetry)


def test_telemetry_pose_ground_truth_has_position_and_orientation(telemetry):
    pose = telemetry.get("/Rover/pose_ground_truth")
    assert pose is not None
    assert "position" in pose
    assert "orientation" in pose


def test_telemetry_has_7_temperatures(telemetry):
    temperatures = [name for name in telemetry if name.startswith("/Rover/temperature_")]
    assert len(temperatures) == 7, f"Expected 7 temperature parameters, got {temperatures}"


def test_telemetry_total_current_in_exists(telemetry):
    assert telemetry.get("/Rover/total_current_in") is not None


def test_telemetry_total_current_out_exists(telemetry):
    assert telemetry.get("/Rover/total_current_out") is not None


def test_telemetry_obc_state_exists(telemetry):
    assert telemetry.get("/Rover/obc_state") is not None
