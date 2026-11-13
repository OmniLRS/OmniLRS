import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
STARTUP_TIMEOUT = int(os.environ.get("OMNILRS_TEST_TIMEOUT", "600"))

SIM_COMMAND = [
    "pixi",
    "run",
    "-e",
    "ci",
    "ros2",
    # The EX1 robot yaml (cfg/robot/ex1.yaml): the action graphs of its USD publish and subscribe to the topics
    # checked below (Ackermann drive on /ackermann_cmd).
    "robot=ex1",
    "rendering.renderer.headless=True",
    # For some reason AWS EKS AMI's duplicate NVIDIA Vulkan ICDs, making one GPU enumerate
    # causing Isaac to think there are two gpus available and then die.
    # Disable and pin.
    "rendering.renderer.multi_gpu=false",
    "rendering.renderer.active_gpu=0",
]

# The last startup message to appear; once seen, startup is complete.
FINAL_MARKER = "ArticulationTelemetry initialized"

# Topics the tests check; the topics fixture waits up to TOPICS_TIMEOUT seconds for all of them.
EXPECTED_TOPICS = {"/ackermann_cmd", "/clock", "/odom", "/parameter_events", "/rosout", "/tf"}
TOPICS_TIMEOUT = int(os.environ.get("OMNILRS_TOPICS_TIMEOUT", "60"))

# Fast fail on the following.
FATAL_MARKERS = [
    "Error executing job",
    "ModuleNotFoundError",
    "ROS2 Bridge startup failed",
    "carb.crashreporter-breakpad",
]


class SimProcess:
    def __init__(self):
        # Artefacts upload dir if available, otherwise a local test dir.
        upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
        log_dir = Path(upload_dir) if upload_dir else Path(__file__).resolve().parent
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / "sim_startup.log"
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
def sim():
    process = SimProcess()
    try:
        process.wait_for_startup(STARTUP_TIMEOUT)
        yield process
    finally:
        process.stop()


@pytest.fixture(scope="session")
def topics(sim):
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    setup = f"/opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash"
    if Path(setup).exists():
        cmd = f"source {setup} && ros2 topic list"
    elif shutil.which("ros2"):
        cmd = "ros2 topic list"
    else:
        pytest.skip("No system ROS2 CLI available to verify topics")
    # DDS discovery of the sim's participants can lag behind the startup marker (slow CI runners), so poll.
    deadline = time.monotonic() + TOPICS_TIMEOUT
    while True:
        result = subprocess.run(["bash", "-c", cmd], env=env, capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, f"'ros2 topic list' failed: {result.stderr}"
        topics = result.stdout.split()
        if EXPECTED_TOPICS.issubset(topics) or time.monotonic() > deadline:
            return topics
        time.sleep(2)


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
ROS2 topics tests.
"""


def test_topic_clock(topics):
    assert "/clock" in topics


def test_topic_ackermann_cmd(topics):
    assert "/ackermann_cmd" in topics


def test_topic_odom(topics):
    assert "/odom" in topics


def test_topic_parameter_events(topics):
    assert "/parameter_events" in topics


def test_topic_rosout(topics):
    assert "/rosout" in topics


def test_topic_tf(topics):
    assert "/tf" in topics


def test_omnilrs_control_topics(topics):
    omnilrs = [t for t in topics if t.startswith("/OmniLRS/")]
    assert omnilrs, f"No /OmniLRS/* control topics advertised. Advertised: {topics}"


def test_durations_within_baseline(check_durations, sim):
    # Keep this test last: it compares the durations of this run with durations_baseline.yaml (Artefacts infra only).
    check_durations(sim.log)
