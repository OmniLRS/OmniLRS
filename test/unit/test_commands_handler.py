"""Unit tests for the camera argument check of src/tmtc/commands_handler.py (no Isaac Sim, no Yamcs server).

python -m pytest -v test/unit/test_commands_handler.py
"""

import pytest

from src.configurations.robot_camera_confs import parse_cameras
from src.tmtc.commands_handler import check_camera_argument

CAMERAS = parse_cameras(
    [
        {"name": "cam_left", "prim_path": "/l", "type": ["rgba", "depth"], "resolutions": {"high": [4, 3]}},
        {"name": "cam_color", "prim_path": "/c", "type": ["rgba"], "resolutions": {"high": [4, 3]}},
    ]
)


def _recording():
    calls = []
    return calls, lambda *args: calls.append(args)


def test_camera_of_the_command_type_reaches_the_handler():
    calls, handler = _recording()
    checked = check_camera_argument(handler, "/Rover/camera/camera_streaming", ["camera_name", "action"], CAMERAS)
    checked("cam_color", "START")
    assert calls == [("cam_color", "START")]


@pytest.mark.parametrize(
    "path, camera",
    [
        ("/Rover/camera/camera_capture_depth", "cam_color"),  # rgba only, the command takes depth cameras
        ("/Rover/camera/camera_capture_high", "NavCam_left"),  # in the server MDB, not on this robot
    ],
)
def test_other_cameras_are_dropped(capsys, path, camera):
    calls, handler = _recording()
    check_camera_argument(handler, path, ["camera_name"], CAMERAS)(camera)
    assert calls == []
    assert "command dropped" in capsys.readouterr().out


def test_camera_name_found_by_argument_name():
    calls, handler = _recording()
    args = ["bitrate", "camera_name"]
    check_camera_argument(handler, "/Rover/camera/set_camera_streaming_params", args, CAMERAS)(100, "cam_left")
    assert calls == [(100, "cam_left")]


def test_other_commands_are_unchanged():
    _, handler = _recording()
    assert check_camera_argument(handler, "/Rover/motor/drive_straight", ["linear_velocity"], CAMERAS) is handler


def test_camera_command_handler_must_take_camera_name():
    _, handler = _recording()
    with pytest.raises(ValueError, match="must take 'camera_name'"):
        check_camera_argument(handler, "/Rover/camera/camera_capture_high", [], CAMERAS)
