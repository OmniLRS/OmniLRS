"""Unit tests for the startup check against the Yamcs server MDB and buckets in src/tmtc/yamcs_contract.py
(no Isaac Sim required).

python -m pytest -v test/unit/test_yamcs_contract.py
"""

import pytest

from src.configurations.robot_camera_confs import parse_cameras
from src.tmtc import yamcs_naming
from src.tmtc.mdb_parsing_service import ServerMdb
from src.tmtc.yamcs_contract import check_contract

CAMERAS = parse_cameras(
    [
        {"name": "cam_depth", "prim_path": "/d", "type": ["depth"], "resolutions": {"high": [4, 3]}},
        {"name": "NavCam_left", "prim_path": "/l", "type": ["rgba"], "resolutions": {"low": [4, 3]}},
    ]
)
STREAMING_FIELDS = ("state", "bandwidth", "packets_sent", "packets_lost", "decode_errors")
YAMCS_CONF = {
    "images": {"buckets": {"lander": {"name": "images_lander", "path": "/Lander/camera/images_oncommand"}}},
    "payload": {
        "apxs": {"resolution": [4, 3], "bucket": {"name": "images_apxs", "path": "/Rover/payload/images_apxs"}}
    },
    "parameters": {"go_nogo": "/Rover/go_nogo"},
    "commands": {
        "drive_straight": "/Rover/motor/drive_straight",
        "camera_capture_high": "/Rover/camera/camera_capture_high",
        "camera_capture_depth": "/Rover/camera/camera_capture_depth",
    },
}
# Argument names of the command handlers, as returned by a controller's command_handlers()
COMMAND_ARGS = {
    "drive_straight": ["linear_velocity", "distance"],
    "camera_capture_high": ["camera_name"],
    "camera_capture_depth": ["camera_name"],
}


def _enum(*labels):
    return {"kind": "enum", "size_bits": 8, "byte_order": "big", "mapping": dict(enumerate(labels))}


def _command(path, **args):
    return {
        "name": path.rsplit("/", 1)[1],
        "full_name": path,
        "root_system": path.split("/")[1],
        "args": [{"name": name, "type": name, "spec": spec} for name, spec in args.items()],
    }


@pytest.fixture
def mdb():
    """A server MDB that has exactly what CAMERAS, YAMCS_CONF and COMMAND_ARGS need."""
    float32 = {"kind": "float", "size_bits": 32, "byte_order": "big"}
    meta_paths = [
        "/Rover/camera/cam_depth/depth",
        "/Rover/camera/NavCam_left/streaming",
        "/Rover/camera/NavCam_left/oncommand",
        "/Lander/camera/images_oncommand",
        "/Rover/payload/images_apxs",
    ]
    mdb = ServerMdb()
    mdb.space_systems = {"/Rover", "/Rover/camera", "/Rover/camera/cam_depth", "/Rover/camera/NavCam_left"}
    mdb.space_systems |= set(meta_paths)
    mdb.parameters = {f"{path}/{field}" for path in meta_paths for field in yamcs_naming.IMAGE_META_FIELDS}
    mdb.parameters |= {f"/Rover/camera/NavCam_left/streaming/{field}" for field in STREAMING_FIELDS}
    mdb.parameters.add("/Rover/go_nogo")
    for command in (
        _command("/Rover/motor/drive_straight", linear_velocity=float32, distance=float32),
        _command("/Rover/camera/camera_capture_high", camera_name=_enum("NavCam_left")),
        _command("/Rover/camera/camera_capture_depth", camera_name=_enum("cam_depth")),
    ):
        mdb.commands[command["full_name"]] = command
    return mdb


# Storage buckets on the server: exactly the image buckets CAMERAS and YAMCS_CONF need, plus a Yamcs one
BUCKETS = {
    "images_cam_depth_depth",
    "images_NavCam_left_streaming",
    "images_NavCam_left_oncommand",
    "images_lander",
    "images_apxs",
    "stacks",
}


def _check(mdb, yamcs_conf=YAMCS_CONF, command_args=COMMAND_ARGS, buckets=BUCKETS):
    return check_contract(
        mdb=mdb,
        buckets=buckets,
        cameras=CAMERAS,
        streaming_fields=STREAMING_FIELDS,
        yamcs_conf=yamcs_conf,
        command_args=command_args,
    )


def test_complete_mdb_passes(mdb):
    report = _check(mdb)
    assert report.ok
    assert report.missing == [] and report.extra == []
    assert report.checked > 0
    assert "OK" in report.summary(["/Rover", "/Lander"])


@pytest.mark.parametrize(
    "parameter",
    [
        "/Rover/go_nogo",  # controller yaml parameter
        "/Rover/camera/cam_depth/depth/url_full",  # camera image metadata
        "/Rover/camera/NavCam_left/streaming/decode_errors",  # streaming telemetry
        "/Lander/camera/images_oncommand/number",  # images.buckets role
        "/Rover/payload/images_apxs/name",  # payload bucket
    ],
)
def test_missing_parameter_fails(mdb, parameter):
    mdb.parameters.discard(parameter)
    report = _check(mdb)
    assert not report.ok
    assert report.missing == [f"parameter {parameter}"]
    assert "FAILED: 1 missing" in report.summary(["/Rover"])


def test_missing_image_metadata_space_system_fails(mdb):
    mdb.space_systems.discard("/Rover/camera/NavCam_left/oncommand")
    assert _check(mdb).missing == ["space system /Rover/camera/NavCam_left/oncommand"]


def test_missing_command_fails(mdb):
    del mdb.commands["/Rover/motor/drive_straight"]
    assert _check(mdb).missing == ["command /Rover/motor/drive_straight"]


def test_missing_handler_argument_fails(mdb):
    mdb.commands["/Rover/motor/drive_straight"]["args"].pop()
    assert _check(mdb).missing == ["argument distance of command /Rover/motor/drive_straight"]


def test_handler_key_not_in_controller_yaml_fails(mdb):
    report = _check(mdb, command_args={**COMMAND_ARGS, "drive_turn": ["angle"]})
    assert report.missing == ["commands.drive_turn in the controller yaml (a command handler needs it)"]


def test_handler_arguments_not_checked_without_command_args(mdb):
    # The standalone check cannot import the controllers (Isaac Sim), so it passes command_args=None
    mdb.commands["/Rover/motor/drive_straight"]["args"].pop()
    assert _check(mdb, command_args=None).ok


def test_missing_camera_name_label_fails(mdb):
    mdb.commands["/Rover/camera/camera_capture_depth"]["args"][0]["spec"] = _enum("other_cam")
    report = _check(mdb)
    assert report.missing == ["camera_name value cam_depth of command /Rover/camera/camera_capture_depth"]
    assert report.extra == ["camera_name value other_cam of command /Rover/camera/camera_capture_depth"]


def test_camera_name_must_be_an_enum(mdb):
    mdb.commands["/Rover/camera/camera_capture_high"]["args"][0]["spec"] = {"kind": "string", "termination_char": 0}
    assert _check(mdb).missing == ["enumerated argument camera_name of command /Rover/camera/camera_capture_high"]


def test_extra_camera_name_label_warns(mdb):
    mdb.commands["/Rover/camera/camera_capture_high"]["args"][0]["spec"] = _enum("NavCam_left", "NavCam_right")
    report = _check(mdb)
    assert report.ok
    assert report.extra == ["camera_name value NavCam_right of command /Rover/camera/camera_capture_high"]
    assert "different robot config" in report.summary(["/Rover"])


def test_extra_camera_and_kind_space_systems_warn(mdb):
    mdb.space_systems |= {
        "/Rover/camera/NavCam_right",
        "/Rover/camera/NavCam_right/streaming",
        "/Rover/camera/cam_depth/streaming",
    }
    report = _check(mdb)
    assert report.ok
    assert report.extra == [
        "space system /Rover/camera/NavCam_right (no such camera in the robot yaml)",
        "space system /Rover/camera/cam_depth/streaming (camera cam_depth does not produce streaming images)",
    ]


def test_camera_commands_not_handled_by_the_controller_are_not_checked(mdb):
    conf = {**YAMCS_CONF, "commands": {"drive_straight": "/Rover/motor/drive_straight"}}
    report = _check(mdb, yamcs_conf=conf, command_args={"drive_straight": ["linear_velocity", "distance"]})
    assert report.ok and report.extra == []


def test_missing_camera_name_argument_is_reported_once(mdb):
    mdb.commands["/Rover/camera/camera_capture_high"]["args"] = []
    assert _check(mdb).missing == ["argument camera_name of command /Rover/camera/camera_capture_high"]


@pytest.mark.parametrize("bucket", ["images_NavCam_left_streaming", "images_lander", "images_apxs"])
def test_missing_bucket_fails(mdb, bucket):
    report = _check(mdb, buckets=BUCKETS - {bucket})
    assert report.missing == [f"bucket {bucket}"]


def test_unused_image_bucket_warns(mdb):
    report = _check(mdb, buckets=BUCKETS | {"images_NavCam_right_streaming", "displays"})
    assert report.ok
    assert report.extra == ["bucket images_NavCam_right_streaming (not used by this robot config)"]
