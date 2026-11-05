"""Unit tests for the Yamcs camera naming rule in src/tmtc/yamcs_naming.py (no Isaac Sim required).

python -m pytest -v test/test_yamcs_naming.py
"""

from pathlib import Path

import pytest
import yaml

from src.tmtc import yamcs_naming

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_YAMLS = sorted((REPO_ROOT / "cfg" / "robot").glob("*.yaml"))
CONTROLLER_YAMLS = sorted((REPO_ROOT / "cfg" / "controller").glob("*.yaml"))

CAMERA_CONF = [
    {"name": "cam_depth", "prim_path": "/Robot/depth", "type": ["depth"], "resolutions": {"high": [1440, 1080]}},
    {"name": "NavCam_left", "prim_path": "/Robot/left", "type": ["rgba"], "resolutions": {"low": [640, 480]}},
    {"name": "both", "prim_path": "/Robot/both", "type": ["rgba", "depth"], "resolutions": {"low": [64, 48]}},
]


def test_parse_cameras_returns_plain_dicts():
    cameras = yamcs_naming.parse_cameras(CAMERA_CONF)
    assert [cam["name"] for cam in cameras] == ["cam_depth", "NavCam_left", "both"]
    assert cameras[1] == {
        "name": "NavCam_left",
        "prim_path": "/Robot/left",
        "type": ("rgba",),
        "resolutions": {"low": (640, 480)},
    }


@pytest.mark.parametrize("conf", [None, []])
def test_parse_cameras_without_cameras(conf):
    assert yamcs_naming.parse_cameras(conf) == []


@pytest.mark.parametrize(
    "conf",
    [
        {"name": "cam", "prim_path": "/p", "type": ["rgba"], "resolutions": {"low": [1, 1]}},  # not a list
        [{"name": "cam", "prim_path": "/p", "resolutions": {"low": [1, 1]}}],  # no type
        [{"name": "cam", "prim_path": "/p", "type": ["ir"], "resolutions": {"low": [1, 1]}}],  # unknown type
        [{"name": "cam", "prim_path": "/p", "type": [], "resolutions": {"low": [1, 1]}}],  # empty type
        [{"name": "cam/x", "prim_path": "/p", "type": ["rgba"], "resolutions": {"low": [1, 1]}}],  # bad name
        [{"name": "cam", "prim_path": "/p", "type": ["rgba"], "resolutions": {}}],  # no resolution
        [CAMERA_CONF[0], CAMERA_CONF[0]],  # duplicate
    ],
)
def test_parse_cameras_rejects_invalid_config(conf):
    with pytest.raises(ValueError):
        yamcs_naming.parse_cameras(conf)


def test_kinds_follow_camera_type():
    cameras = yamcs_naming.parse_cameras(CAMERA_CONF)
    assert [yamcs_naming.camera_kinds(cam) for cam in cameras] == [
        ("depth",),
        ("streaming", "oncommand"),
        ("streaming", "oncommand", "depth"),
    ]


def test_paths_and_bucket_names():
    assert yamcs_naming.camera_system("NavCam_left") == "/Rover/camera/NavCam_left"
    assert yamcs_naming.meta_path("NavCam_left", "oncommand") == "/Rover/camera/NavCam_left/oncommand"
    assert yamcs_naming.streaming_param("NavCam_left", "state") == "/Rover/camera/NavCam_left/streaming/state"
    assert yamcs_naming.bucket_name("NavCam_left", "streaming") == "images_NavCam_left_streaming"


def test_camera_buckets():
    cameras = yamcs_naming.parse_cameras(CAMERA_CONF[:2])
    assert yamcs_naming.camera_buckets(cameras) == [
        ("cam_depth", "depth", "images_cam_depth_depth", "/Rover/camera/cam_depth/depth"),
        ("NavCam_left", "streaming", "images_NavCam_left_streaming", "/Rover/camera/NavCam_left/streaming"),
        ("NavCam_left", "oncommand", "images_NavCam_left_oncommand", "/Rover/camera/NavCam_left/oncommand"),
    ]


def test_cameras_for_command_by_type_in_yaml_order():
    cameras = yamcs_naming.parse_cameras(CAMERA_CONF)
    assert yamcs_naming.cameras_for_command(cameras, "camera_capture_depth") == ["cam_depth", "both"]
    for command in ("camera_capture_high", "camera_streaming", "set_camera_streaming_params"):
        assert yamcs_naming.cameras_for_command(cameras, command) == ["NavCam_left", "both"]


def test_streaming_fields():
    assert yamcs_naming.streaming_fields("pragyaan") == (
        "state",
        "bandwidth",
        "packets_sent",
        "packets_lost",
        "decode_errors",
    )
    with pytest.raises(KeyError):
        yamcs_naming.streaming_fields("unknown_robot")


def test_image_buckets_cover_cameras_roles_and_payloads():
    cameras = yamcs_naming.parse_cameras(
        [{"name": "NavCam_left", "prim_path": "/l", "type": ["rgba"], "resolutions": {"low": [4, 3]}}]
    )
    yamcs_tmtc = {
        "images": {"buckets": {"lander": {"name": "images_lander", "path": "/Lander/camera/images_oncommand"}}},
        "payload": {"apxs": {"resolution": [4, 3], "bucket": {"name": "images_apxs", "path": "/Rover/payload/apxs"}}},
    }
    assert yamcs_naming.image_buckets(cameras, yamcs_tmtc) == [
        ("images_NavCam_left_streaming", "/Rover/camera/NavCam_left/streaming"),
        ("images_NavCam_left_oncommand", "/Rover/camera/NavCam_left/oncommand"),
        ("images_lander", "/Lander/camera/images_oncommand"),
        ("images_apxs", "/Rover/payload/apxs"),
    ]


def _robots_settings(path: Path) -> dict:
    return yaml.safe_load(path.read_text())["robots_settings"]


@pytest.mark.parametrize("robot_yaml", ROBOT_YAMLS, ids=lambda path: path.stem)
def test_robot_yaml_cameras_parse(robot_yaml):
    # Robot() parses the camera list in every mode (ROS2, Yamcs, Zenoh): a robot yaml that does not parse breaks the
    # robot everywhere, not just in Yamcs mode.
    parameters = _robots_settings(robot_yaml)["parameters"]
    cameras = yamcs_naming.parse_cameras(parameters.get("camera"))
    if any(yamcs_naming.has_type(camera, "rgba") for camera in cameras):
        assert yamcs_naming.streaming_fields(parameters["robot_name"])


@pytest.mark.parametrize("controller_yaml", CONTROLLER_YAMLS, ids=lambda path: path.stem)
def test_controller_yaml_has_the_yamcs_startup_keys(controller_yaml):
    # <robot>-controller.yaml pairs with cfg/robot/<robot>.yaml
    robot = controller_yaml.stem.removesuffix("-controller")
    cameras = yamcs_naming.parse_cameras(
        _robots_settings(REPO_ROOT / "cfg" / "robot" / f"{robot}.yaml")["parameters"].get("camera")
    )
    yamcs_tmtc = _robots_settings(controller_yaml)["yamcs_tmtc"]
    assert yamcs_tmtc["mdb_space_systems"]
    assert isinstance(yamcs_tmtc["images"]["clear_buckets_on_startup"], bool)
    buckets = yamcs_naming.image_buckets(cameras, yamcs_tmtc)  # images.buckets is keyed by role
    assert len({name for name, _ in buckets}) == len(buckets), f"duplicate image bucket names: {buckets}"
