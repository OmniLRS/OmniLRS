__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import StrEnum

from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.tmtc.yamcs_TMTC import ImagesHandler

import omni.kit.app
import numpy as np
from omni.isaac.sensor import Camera
from PIL import Image


class CameraViewType(StrEnum):
    RGBA = "RGBA"
    RGB = "RGB"
    DEPTH = "DEPTH"


class HuskyCameraHandler:
    """
    Camera handler for the Husky UGV.

    Manages the rover's onboard camera and an optional monitoring camera
    in the environment.  Depth and RGBA capture are both supported.

    Mirrors the structure of PragyaanCameraHandler; lander-camera logic has
    been removed because Husky does not have an associated lander.
    """

    BUCKET_STREAMING = "images_streaming"
    BUCKET_ONCOMMAND = "images_oncommand"
    BUCKET_DEPTH = "images_depth"
    BUCKET_MONITORING = "images_monitoring"

    def __init__(self, images_handler: ImagesHandler, robot) -> None:
        self._images_handler = images_handler
        self._robot = robot
        self._monitoring_cam = None
        self._initialize_monitoring_cam()

    def _initialize_monitoring_cam(self) -> None:
        if len(list(MonitoringCamerasManager.cameras.keys())) == 0:
            return
        camera_name = list(MonitoringCamerasManager.cameras.keys())[0]
        self._monitoring_cam = MonitoringCamerasManager.cameras[camera_name]

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

    def _snap_monitoring_camera_view(self) -> Image:
        if self._monitoring_cam is None:
            return None
        frame = self._monitoring_cam.get_rgb()
        if len(frame) == 0:
            return None
        return Image.fromarray(frame.astype(np.uint8), CameraViewType.RGB.value)

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

    def transmit_monitoring_camera_view(self):
        camera_view = self._snap_monitoring_camera_view()
        if camera_view is None:
            return
        self._images_handler.save_image(camera_view, self.BUCKET_MONITORING)