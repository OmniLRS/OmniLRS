"""Robot cameras: the `camera` list of the robot yaml (cfg/robot/<robot>.yaml), the same in every mode.

Parsed and validated when the robot config is loaded (RobotParameters). The Yamcs MDB generators
(yamcs-mission-control repositories) also load this file by path from $OMNILRS_ROOT: keep it standard-library only.
"""

__author__ = "Louis Burtz"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import re
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

CAMERA_TYPES = ("rgba", "depth")

# Camera names are used in Zenoh keys, Yamcs MDB paths and Yamcs bucket names
_NAME_RE = re.compile(r"[A-Za-z][A-Za-z0-9_]*")


@dataclass(frozen=True)
class RobotCameraConf:
    name: str
    prim_path: str
    type: tuple[str, ...]  # one or more of CAMERA_TYPES
    resolutions: dict[str, tuple[int, int]] = field(hash=False)  # resolution key (e.g. low, high) -> (width, height)

    def has_type(self, camera_type: str) -> bool:
        return camera_type in self.type


def parse_cameras(camera_conf: Any) -> list[RobotCameraConf]:
    """Validate the robot yaml `camera` list: name, prim_path, type (rgba and/or depth), resolutions."""
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
        if not _NAME_RE.fullmatch(name):
            raise ValueError(f"camera name {name!r} must match {_NAME_RE.pattern} (used in Zenoh keys and Yamcs names)")
        if name in seen:
            raise ValueError(f"duplicate camera name {name!r}")
        seen.add(name)
        types = tuple(str(t) for t in cam["type"])
        if not types or any(t not in CAMERA_TYPES for t in types) or len(set(types)) != len(types):
            raise ValueError(f"camera {name!r}: type must be a non-empty list of distinct values from {CAMERA_TYPES}")
        resolutions = {str(res): (int(wh[0]), int(wh[1])) for res, wh in dict(cam["resolutions"]).items()}
        if not resolutions:
            raise ValueError(f"camera {name!r}: resolutions must not be empty")
        cameras.append(RobotCameraConf(name=name, prim_path=str(cam["prim_path"]), type=types, resolutions=resolutions))
    return cameras


def cameras_of_type(cameras: Iterable[RobotCameraConf], camera_type: str) -> list[str]:
    """Names of the cameras of that type (rgba or depth), in robot yaml order."""
    return [camera.name for camera in cameras if camera.has_type(camera_type)]
