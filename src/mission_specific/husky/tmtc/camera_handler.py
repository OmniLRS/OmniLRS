__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import StrEnum

import numpy as np
from PIL import Image

from src.tmtc.yamcs_TMTC import ImagesHandler


class CameraViewType(StrEnum):
    RGBA = "RGBA"
    DEPTH = "DEPTH"


class HuskyCameraHandler:
    """
    Camera handler for the Husky UGV.

    Manages the rover's onboard camera. Depth and RGBA capture are both
    supported.

    Mirrors the structure of PragyaanCameraHandler; lander-camera logic has
    been removed because Husky does not have an associated lander.
    """

    BUCKET_STREAMING = "images_streaming"
    BUCKET_ONCOMMAND = "images_oncommand"
    BUCKET_DEPTH = "images_depth"

    def __init__(self, images_handler: ImagesHandler, robot) -> None:
        self._images_handler = images_handler
        self._robot = robot

    # ------------------------------------------------------------------
    # Internal snap helpers
    # ------------------------------------------------------------------

    def _snap_camera_view_rgb(self, resolution: str) -> Image:
        frame = self._robot.get_rgba_camera_view(resolution)
        frame_uint8 = frame.astype(np.uint8)
        return Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

    def _snap_camera_view_depth(self, resolution: str) -> Image:
        frame = self._robot.get_depth_camera_view(resolution)
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

    def transmit_camera_view(
        self,
        bucket: str,
        resolution: str,
        type: CameraViewType = CameraViewType.RGBA,
    ):
        if type == CameraViewType.DEPTH:
            camera_view = self._snap_camera_view_depth(resolution)
        elif type == CameraViewType.RGBA:
            camera_view = self._snap_camera_view_rgb(resolution)
        else:
            print("HuskyCameraHandler.transmit_camera_view: unknown type:", type)
            return
        self._images_handler.save_image(camera_view, bucket)
