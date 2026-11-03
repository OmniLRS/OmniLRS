"""Unit tests for the Yamcs camera naming rule in src/tmtc/yamcs_naming.py (no Isaac Sim required).

python -m pytest -v test/test_yamcs_naming.py
"""

import pytest

from src.tmtc import yamcs_naming

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
