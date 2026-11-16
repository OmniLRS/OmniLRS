import os
import time
from pathlib import Path

import pytest

# A scenario whose tests take a few seconds after startup still gives a video of at least this many frames: 10 s of
# simulated time at the 5 frames per simulated second of src/environments_wrappers/robot_video_recorder.py.
MIN_VIDEO_FRAMES = 50
VIDEO_TAIL_TIMEOUT_S = 60


def pytest_configure(config):
    """junit output path for the Artefacts CI scenarios (artefacts.yaml), written to its upload directory"""
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    if upload_dir and not config.option.xmlpath:
        config.option.xmlpath = os.path.join(upload_dir, "tests_junit.xml")


def _write_step(video_path, step):
    with open(Path(video_path).with_suffix(".steps.log"), "a") as steps:
        steps.write(f"{step}\n")


def _video_frames(video_path):
    """Frames written to the video so far: the recorder writes one MP4 fragment ('moof' box) per frame."""
    try:
        return Path(video_path).read_bytes().count(b"moof")
    except OSError:
        return 0


@pytest.fixture(autouse=True)
def video_step(request):
    """
    Writes the name of each test to the steps file of the robot video, which shows it as the current test step. A test
    module records the video when it defines VIDEO_PATH and passes it to the simulator in OMNILRS_ROBOT_VIDEO
    (src/environments_wrappers/robot_video_recorder.py).
    """
    video_path = getattr(request.module, "VIDEO_PATH", None)
    if video_path is not None:
        _write_step(video_path, request.node.name)


@pytest.fixture(scope="module", autouse=True)
def video_tail(request):
    """
    After the last test of a module that records a video, keeps the simulator running until the video has
    MIN_VIDEO_FRAMES frames (at most VIDEO_TAIL_TIMEOUT_S). It requests the `sim` fixture of the module, so it runs
    before the simulator stops.
    """
    video_path = getattr(request.module, "VIDEO_PATH", None)
    if video_path is None:
        yield
        return
    request.getfixturevalue("sim")
    yield
    _write_step(video_path, "after the last test")
    deadline = time.monotonic() + VIDEO_TAIL_TIMEOUT_S
    while _video_frames(video_path) < MIN_VIDEO_FRAMES and time.monotonic() < deadline:
        time.sleep(1)
