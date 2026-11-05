__author__ = "Amaan Javed"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import StrEnum

import numpy as np
from PIL import Image

from src.mission_specific.husky.tmtc.enums import KIND_RESOLUTION
from src.tmtc import yamcs_naming
from src.tmtc.yamcs_TMTC import ImagesHandler


class CameraViewType(StrEnum):
    RGBA = "RGBA"


class HuskyCameraHandler:
    """
    Camera handler for the Husky UGV.

    Manages the rover's cameras (robot yaml `camera` list): they snap RGB views (type rgba: streaming and oncommand
    images) and/or DEPTH views (type depth: depth images), uploaded to the camera's buckets through the ImagesHandler.

    Mirrors the structure of PragyaanCameraHandler; the lander and monitoring cameras and the streaming compression
    model are left out because the Husky has none.
    """

    def __init__(self, images_handler: ImagesHandler, robot) -> None:
        self._images_handler = images_handler
        self._robot = robot
        yamcs_naming.check_kind_resolutions(robot.cameras, KIND_RESOLUTION)

    def cameras_of_type(self, camera_type: str) -> list[str]:
        """Names of the rover cameras of that type (rgba or depth), in robot yaml order."""
        return [camera["name"] for camera in self._robot.cameras if yamcs_naming.has_type(camera, camera_type)]

    # ------------------------------------------------------------------
    # Internal snap helpers
    # ------------------------------------------------------------------

    def _snap_camera_view_rgb(self, camera_name: str, resolution: str) -> Image.Image | None:
        frame = self._robot.get_rgba_camera_view(camera_name, resolution)
        if frame is None:  # camera still warming up
            return None
        frame_uint8 = frame.astype(np.uint8)
        return Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

    def _snap_camera_view_depth(self, camera_name: str, resolution: str) -> Image.Image | None:
        frame = self._robot.get_depth_camera_view(camera_name, resolution)
        if frame is None:  # camera still warming up
            return None
        depth = np.nan_to_num(frame, nan=0.0, posinf=0.0, neginf=0.0)

        valid = depth > 0
        if not np.any(valid):
            return Image.fromarray(np.zeros_like(depth, dtype=np.uint8), "L")

        near, far = 0.0, 10.0
        d = np.clip(depth, near, far)
        d = (d - near) / (far - near)
        d = (1.0 - d) * 255.0
        return Image.fromarray(d.astype(np.uint8), mode="L")

    # ------------------------------------------------------------------
    # Transmit helpers (called by commander / controller intervals)
    # ------------------------------------------------------------------

    def transmit_camera_view(self, camera_name: str, kind: str):
        """Snap and upload one image of a rover camera; kind is streaming, oncommand (rgba) or depth (depth)."""
        if kind not in KIND_RESOLUTION:
            print("HuskyCameraHandler.transmit_camera_view: unknown kind:", kind)
            return
        resolution = KIND_RESOLUTION[kind].value
        if kind == "depth":
            camera_view = self._snap_camera_view_depth(camera_name, resolution)
        else:
            camera_view = self._snap_camera_view_rgb(camera_name, resolution)

        if camera_view is None:  # camera still warming up
            return
        self._images_handler.save_camera_image(camera_view, camera_name=camera_name, kind=kind)
