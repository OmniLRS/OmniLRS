import os
from pathlib import Path

import pytest


def pytest_configure(config):
    """junit output path for the Artefacts CI scenarios (artefacts.yaml), written to its upload directory"""
    upload_dir = os.environ.get("ARTEFACTS_SCENARIO_UPLOAD_DIR")
    if upload_dir and not config.option.xmlpath:
        config.option.xmlpath = os.path.join(upload_dir, "tests_junit.xml")


@pytest.fixture(autouse=True)
def video_step(request):
    """
    Writes the name of each test to the steps file of the robot video, which shows it as the current test step. A test
    module records the video when it defines VIDEO_PATH and passes it to the simulator in OMNILRS_ROBOT_VIDEO
    (src/environments_wrappers/robot_video_recorder.py).
    """
    video_path = getattr(request.module, "VIDEO_PATH", None)
    if video_path is not None:
        with open(Path(video_path).with_suffix(".steps.log"), "a") as steps:
            steps.write(f"{request.node.name}\n")
