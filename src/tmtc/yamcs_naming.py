"""Yamcs naming rule for robot cameras.

Single definition of how robot cameras (cfg/robot/<robot>.yaml) map to Yamcs MDB paths and storage buckets.
Used by OmniLRS at runtime and by the yamcs-mission-control MDB generators, which load this file by path
from $OMNILRS_ROOT. Keep it standard-library only and free of OmniLRS imports: it works on the robot cameras parsed by
src/configurations/robot_camera_confs.py (RobotCameraConf: name, type, resolutions, has_type) without importing it.

MDB layout (one space system per camera, one child per kind):
    /Rover/camera/<camera>/<kind>/{number,name,url_storage,url_full,url_full_nginx}
    /Rover/camera/<camera>/streaming/{<streaming telemetry fields>}
Buckets are flat (Yamcs bucket names cannot contain "/"): images_<camera>_<kind>.
Names are only ever built from (camera, kind), never parsed back.
"""

from typing import Iterable, Mapping

CAMERA_ROOT = "/Rover/camera"
BUCKET_PREFIX = "images_"

# Image kinds of each camera type (robot_camera_confs.CAMERA_TYPES)
KINDS_BY_TYPE = {"rgba": ("streaming", "oncommand"), "depth": ("depth",)}

IMAGE_META_FIELDS = ("number", "name", "url_storage", "url_full", "url_full_nginx")

# Camera commands (MDB command name -> camera type its camera_name argument accepts).
CAMERA_COMMAND_TYPE = {
    "camera_capture_high": "rgba",
    "camera_streaming": "rgba",
    "set_camera_streaming_params": "rgba",
    "camera_capture_depth": "depth",
}
CAMERA_NAME_ARG = "camera_name"


def camera_kinds(camera) -> tuple[str, ...]:
    """Image kinds a camera produces, from its type (e.g. rgba -> streaming, oncommand)."""
    return tuple(kind for t in camera.type for kind in KINDS_BY_TYPE[t])


def check_kind_resolutions(cameras: Iterable, kind_resolution: Mapping[str, str]) -> None:
    """Raise ValueError if a camera lacks the resolution used for one of its kinds.

    kind_resolution: image kind -> resolutions key of the robot yaml (e.g. streaming -> low), defined per robot in
    src/mission_specific/<robot>/tmtc/enums.py and used by its camera handler.
    """
    missing = [
        f"camera {cam.name!r}: {kind} images use resolution {kind_resolution[kind]!r}"
        for cam in cameras
        for kind in camera_kinds(cam)
        if kind_resolution[kind] not in cam.resolutions
    ]
    if missing:
        raise ValueError("robot yaml camera resolutions are missing: " + "; ".join(missing))


def camera_system(camera_name: str) -> str:
    """/Rover/camera/<camera>"""
    return f"{CAMERA_ROOT}/{camera_name}"


def meta_path(camera_name: str, kind: str) -> str:
    """/Rover/camera/<camera>/<kind>: space system holding the image metadata of that bucket."""
    return f"{camera_system(camera_name)}/{kind}"


def streaming_param(camera_name: str, field: str) -> str:
    """/Rover/camera/<camera>/streaming/<field>"""
    return f"{meta_path(camera_name, kind='streaming')}/{field}"


def bucket_name(camera_name: str, kind: str) -> str:
    """images_<camera>_<kind>"""
    return f"{BUCKET_PREFIX}{camera_name}_{kind}"


def camera_buckets(cameras: Iterable) -> list[tuple[str, str, str, str]]:
    """(camera, kind, bucket_name, meta_path) for every image bucket the cameras produce."""
    return [
        (cam.name, kind, bucket_name(cam.name, kind), meta_path(cam.name, kind))
        for cam in cameras
        for kind in camera_kinds(cam)
    ]


def image_buckets(cameras: Iterable, yamcs_tmtc: Mapping) -> list[tuple[str, str]]:
    """(bucket name, metadata space system) of every image bucket OmniLRS uploads to: the cameras' buckets, then the
    images.buckets roles and the payload buckets of the controller yaml `yamcs_tmtc` section."""
    buckets = [(name, path) for _, _, name, path in camera_buckets(cameras)]
    buckets += [(bucket["name"], bucket["path"]) for bucket in yamcs_tmtc["images"]["buckets"].values()]
    buckets += [
        (payload["bucket"]["name"], payload["bucket"]["path"])
        for payload in yamcs_tmtc.get("payload", {}).values()
        if "bucket" in payload
    ]
    return buckets


def cameras_for_command(cameras: Iterable, command: str) -> list[str]:
    """Camera names allowed for a camera command's camera_name argument, in robot yaml order."""
    camera_type = CAMERA_COMMAND_TYPE[command]
    return [cam.name for cam in cameras if cam.has_type(camera_type)]
