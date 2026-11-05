"""Unit tests for the robot camera config in src/configurations/robot_camera_confs.py (no Isaac Sim required).

python -m pytest -v test/test_robot_camera_confs.py
"""

from pathlib import Path

import pytest
import yaml

from src.configurations.robot_camera_confs import RobotCameraConf, cameras_of_type, parse_cameras
from src.configurations.robot_confs import RobotParameters

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_YAMLS = sorted((REPO_ROOT / "cfg" / "robot").glob("*.yaml"))

CAMERA_CONF = [
    {"name": "cam_depth", "prim_path": "/Robot/depth", "type": ["depth"], "resolutions": {"high": [1440, 1080]}},
    {"name": "NavCam_left", "prim_path": "/Robot/left", "type": ["rgba"], "resolutions": {"low": [640, 480]}},
    {"name": "both", "prim_path": "/Robot/both", "type": ["rgba", "depth"], "resolutions": {"low": [64, 48]}},
]


def test_parse_cameras_returns_robot_camera_confs():
    cameras = parse_cameras(CAMERA_CONF)
    assert [cam.name for cam in cameras] == ["cam_depth", "NavCam_left", "both"]
    assert cameras[1] == RobotCameraConf(
        name="NavCam_left", prim_path="/Robot/left", type=("rgba",), resolutions={"low": (640, 480)}
    )


@pytest.mark.parametrize("conf", [None, []])
def test_parse_cameras_without_cameras(conf):
    assert parse_cameras(conf) == []


@pytest.mark.parametrize(
    "conf",
    [
        {"name": "cam", "prim_path": "/p", "type": ["rgba"], "resolutions": {"low": [1, 1]}},  # not a list
        [{"name": "cam", "prim_path": "/p", "resolutions": {"low": [1, 1]}}],  # no type
        [{"name": "cam", "prim_path": "/p", "type": ["ir"], "resolutions": {"low": [1, 1]}}],  # unknown type
        [{"name": "cam", "prim_path": "/p", "type": [], "resolutions": {"low": [1, 1]}}],  # empty type
        [{"name": "cam/x", "prim_path": "/p", "type": ["rgba"], "resolutions": {"low": [1, 1]}}],  # bad name
        [{"name": "cam\n", "prim_path": "/p", "type": ["rgba"], "resolutions": {"low": [1, 1]}}],  # trailing newline
        [{"name": "cam", "prim_path": "/p", "type": ["rgba"], "resolutions": {}}],  # no resolution
        [CAMERA_CONF[0], CAMERA_CONF[0]],  # duplicate
    ],
)
def test_parse_cameras_rejects_invalid_config(conf):
    with pytest.raises(ValueError):
        parse_cameras(conf)


def test_cameras_of_type_in_yaml_order():
    cameras = parse_cameras(CAMERA_CONF)
    assert cameras_of_type(cameras, "rgba") == ["NavCam_left", "both"]
    assert cameras_of_type(cameras, "depth") == ["cam_depth", "both"]
    assert cameras[2].has_type("rgba") and cameras[2].has_type("depth") and not cameras[1].has_type("depth")


def test_robot_parameters_parse_the_camera_list():
    parameters = RobotParameters(camera=CAMERA_CONF[:1])
    assert parameters.camera == [
        RobotCameraConf(name="cam_depth", prim_path="/Robot/depth", type=("depth",), resolutions={"high": (1440, 1080)})
    ]
    with pytest.raises(ValueError):
        RobotParameters(camera=[{"name": "cam", "prim_path": "/p", "type": ["ir"], "resolutions": {"low": [1, 1]}}])


@pytest.mark.parametrize("robot_yaml", ROBOT_YAMLS, ids=lambda path: path.stem)
def test_robot_yaml_cameras_parse(robot_yaml):
    # RobotParameters parses the camera list in every mode (ROS2, Yamcs, Zenoh): a robot yaml that does not parse
    # breaks the robot everywhere, not just in Yamcs mode.
    parse_cameras(yaml.safe_load(robot_yaml.read_text())["robots_settings"]["parameters"].get("camera"))
