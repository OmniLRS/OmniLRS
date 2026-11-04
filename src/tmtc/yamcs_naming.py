"""Yamcs naming rule for robot cameras.

Single definition of how robot cameras (cfg/robot/<robot>.yaml) map to Yamcs MDB paths and storage buckets.
Used by OmniLRS at runtime and by the yamcs-mission-control MDB generators, which load this file by path
from $OMNILRS_ROOT. Keep it standard-library only.

MDB layout (one space system per camera, one child per kind):
    /Rover/camera/<camera>/<kind>/{number,name,url_storage,url_full,url_full_nginx}
    /Rover/camera/<camera>/streaming/{<streaming telemetry fields>}
Buckets are flat (Yamcs bucket names cannot contain "/"): images_<camera>_<kind>.
Names are only ever built from (camera, kind), never parsed back.
"""

import re
from typing import Any, Iterable, Mapping

CAMERA_ROOT = "/Rover/camera"
BUCKET_PREFIX = "images_"

CAMERA_TYPES = ("rgba", "depth")
KINDS_BY_TYPE = {"rgba": ("streaming", "oncommand"), "depth": ("depth",)}

IMAGE_META_FIELDS = ("number", "name", "url_storage", "url_full", "url_full_nginx")

# Streaming telemetry sent per rgba camera, under streaming_param(camera, field).
STREAMING_FIELDS = {
    "pragyaan": ("state", "bandwidth", "packets_sent", "packets_lost", "decode_errors"),
    "husky": ("state",),
}

# Camera commands (MDB command name -> camera type its camera_name argument accepts).
CAMERA_COMMAND_TYPE = {
    "camera_capture_high": "rgba",
    "camera_streaming": "rgba",
    "set_camera_streaming_params": "rgba",
    "camera_capture_depth": "depth",
}
CAMERA_NAME_ARG = "camera_name"

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def parse_cameras(camera_conf: Any) -> list[dict]:
    """Validate the robot yaml `camera` list and return plain dicts: name, prim_path, type (tuple), resolutions."""
    if camera_conf is None:
        return []
    if isinstance(camera_conf, Mapping) or isinstance(camera_conf, str):
        raise ValueError("robot camera config must be a list of cameras, each with name, prim_path, type, resolutions")

    cameras = []
    seen = set()
    for i, cam in enumerate(camera_conf):
        for key in ("name", "prim_path", "type", "resolutions"):
            if key not in cam:
                raise ValueError(f"camera #{i} is missing '{key}'")
        name = str(cam["name"])
        if not _NAME_RE.match(name):
            raise ValueError(f"camera name {name!r} must match {_NAME_RE.pattern} (used in MDB paths and bucket names)")
        if name in seen:
            raise ValueError(f"duplicate camera name {name!r}")
        seen.add(name)
        types = tuple(str(t) for t in cam["type"])
        if not types or any(t not in CAMERA_TYPES for t in types) or len(set(types)) != len(types):
            raise ValueError(f"camera {name!r}: type must be a non-empty list of distinct values from {CAMERA_TYPES}")
        resolutions = {str(res): (int(wh[0]), int(wh[1])) for res, wh in dict(cam["resolutions"]).items()}
        if not resolutions:
            raise ValueError(f"camera {name!r}: resolutions must not be empty")
        cameras.append({"name": name, "prim_path": str(cam["prim_path"]), "type": types, "resolutions": resolutions})
    return cameras


def camera_kinds(camera: Mapping) -> tuple[str, ...]:
    """Image kinds a camera produces, from its type (e.g. rgba -> streaming, oncommand)."""
    return tuple(kind for t in camera["type"] for kind in KINDS_BY_TYPE[t])


def has_type(camera: Mapping, camera_type: str) -> bool:
    return camera_type in camera["type"]


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


def camera_buckets(cameras: Iterable[Mapping]) -> list[tuple[str, str, str, str]]:
    """(camera, kind, bucket_name, meta_path) for every image bucket the cameras produce."""
    return [
        (cam["name"], kind, bucket_name(cam["name"], kind), meta_path(cam["name"], kind))
        for cam in cameras
        for kind in camera_kinds(cam)
    ]


def image_buckets(cameras: Iterable[Mapping], yamcs_tmtc: Mapping) -> list[tuple[str, str]]:
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


def cameras_for_command(cameras: Iterable[Mapping], command: str) -> list[str]:
    """Camera names allowed for a camera command's camera_name argument, in robot yaml order."""
    camera_type = CAMERA_COMMAND_TYPE[command]
    return [cam["name"] for cam in cameras if has_type(cam, camera_type)]


def streaming_fields(robot_name: str) -> tuple[str, ...]:
    if robot_name not in STREAMING_FIELDS:
        raise KeyError(
            f"no streaming telemetry fields defined for robot {robot_name!r} in yamcs_naming.STREAMING_FIELDS"
        )
    return STREAMING_FIELDS[robot_name]
