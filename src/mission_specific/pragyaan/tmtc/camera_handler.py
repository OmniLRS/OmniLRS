__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from enum import StrEnum

import numpy as np
from isaacsim.sensors.experimental.rtx import CameraSensor
from PIL import Image

from src.configurations.robot_camera_confs import cameras_of_type
from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.mission_specific.pragyaan.tmtc.enums import KIND_RESOLUTION
from src.subsystems.robot_physics_models.camera_compression_model import CameraCompressionModel
from src.tmtc import yamcs_naming
from src.tmtc.yamcs_TMTC import ImagesHandler


class CameraViewType(StrEnum):
    RGBA = "RGBA"
    RGB = "RGB"  # only for monitoring, as is more light weight


class PragyaanCameraHandler:
    """
    This file contains methods for handling ALL cameras related to Rragyaan's setup.

    Rover's own camera(s) are initialized in the file for robot initialization.
    This file initializes additional cameras in the environment, such as lander's camera, and monitoring camera.

    Camera handler contains the methods for snapping camera views of rover's cameras, and lander's and monitoring camera.
    Rover's cameras (robot yaml `camera` list) snap RGB views (type rgba: streaming and oncommand images) and/or
    DEPTH views (type depth: depth images). Streaming images are compressed by one model per rgba camera.

    Handler also contains methods for transmitting snapped views to Yamcs, by utilizing saving methods of the images handler property.

    CameraHandler is a layer on top of the ImagesHandler, and in that way is implemented a separation between controlling different
    instances of camera, and concers of low-level image saving to the Yamcs buckets.
    Thus CameraHandler(s) are intended to reuse functionalities provided by the ImagesHandler, and not reimplement the communication
    with Yamcs from the scratch.

    The reference to _robot provides access to rover's cameras, and desired use-case implementation.
    """

    def __init__(
        self,
        images_handler: ImagesHandler,
        robot,
        lander_camera_conf=None,
        *,
        streaming_interval: float,
        compression_conf,
    ) -> None:
        self._images_handler = images_handler
        self._robot = robot
        yamcs_naming.check_kind_resolutions(robot.cameras, KIND_RESOLUTION)
        self._lander_cam = None
        self._monitoring_cam = None
        compression_conf = dict(compression_conf)
        # Use the same nominal rate as the transmission scheduler.
        compression_conf["fps"] = 1 / streaming_interval
        # One model per streaming (rgba) camera
        self._compression_models: dict[str, CameraCompressionModel] = {}
        for camera_name in cameras_of_type(robot.cameras, "rgba"):
            model = CameraCompressionModel()
            model.initialize(**compression_conf)
            self._compression_models[camera_name] = model
        self._initialize_lander_cam(lander_camera_conf)
        self._initialize_monitoring_cam()

    def _initialize_lander_cam(self, lander_camera_conf) -> None:
        if lander_camera_conf is None:
            return

        width, height = lander_camera_conf["resolution"]
        self._lander_cam = CameraSensor(
            lander_camera_conf["prim_path"], resolution=(height, width), annotators=["rgba"]
        )

    def _initialize_monitoring_cam(self) -> None:
        # NOTE TODO implemented for one camera right now, in the future make for multiple idea: dict same as for cameras in MonCamManager,
        # and just iterate over them, make generic names that differ only in _id (mon_cam_1, mon_cam_2, ...) and make such yamcs buckets
        if len(list(MonitoringCamerasManager.cameras.keys())) == 0:
            return

        camera_name = list(MonitoringCamerasManager.cameras.keys())[0]
        self._monitoring_cam = MonitoringCamerasManager.cameras[camera_name]

    def _snap_camera_view_rgb(self, camera_name: str, resolution: str) -> Image:
        frame = self._robot.get_rgba_camera_view(camera_name, resolution)
        if frame is None:  # camera still warming up
            return None
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

        return camera_view

    def _snap_camera_view_depth(self, camera_name: str, resolution: str) -> Image:
        frame = self._robot.get_depth_camera_view(camera_name, resolution)
        if frame is None:  # camera still warming up
            return None
        depth = np.nan_to_num(frame, nan=0.0, posinf=0.0, neginf=0.0)

        valid = depth > 0
        if not np.any(valid):
            return Image.fromarray(np.zeros_like(depth, dtype=np.uint8), "L")

        near, far = 0.0, 10.0  # fixed 0-10 m range for consistent depth scaling

        d = np.clip(depth, near, far)
        d = (d - near) / (far - near)
        d = (1.0 - d) * 255.0
        d_uint8 = d.astype(np.uint8)

        return Image.fromarray(d_uint8, mode="L")

    def _snap_lander_camera_view(self) -> Image:
        if self._lander_cam is None:
            return

        frame, _ = self._lander_cam.get_data("rgba")
        if frame is None:  # camera still warming up
            return None
        frame_uint8 = frame.numpy().astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

        return camera_view

    def _snap_monitoring_camera_view(self) -> Image:
        if self._monitoring_cam is None:
            return

        frame, _ = self._monitoring_cam.get_data("rgb")

        # NOTE interval tries to trigger it before initialization: programmatically created cameras
        # require more time to be initialized than the ones already existing inside the models
        if frame is None:
            return None

        frame_uint8 = frame.numpy().astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGB.value)

        return camera_view

    def _compress(self, image: Image, camera_name: str) -> Image:
        model = self._compression_models[camera_name]
        model.set_inputs(image)
        model.compute(0.0)
        return model.get_outputs()["compressed_image"]

    def get_compression_model_outputs(self, camera_name: str):
        return self._compression_models[camera_name].get_outputs()

    def set_streaming_compression_params(self, camera_name: str, **params):
        self._compression_models[camera_name].update_parameters(**params)

    def transmit_camera_view(self, camera_name: str, kind: str):
        """Snap and upload one image of a rover camera; kind is streaming, oncommand (rgba) or depth (depth)."""
        if kind not in KIND_RESOLUTION:
            print("in transmit_camera_view: unknown kind:", kind)
            return
        resolution = KIND_RESOLUTION[kind].value
        if kind == "depth":
            camera_view: Image = self._snap_camera_view_depth(camera_name, resolution)
        else:
            camera_view: Image = self._snap_camera_view_rgb(camera_name, resolution)
            # only streaming images are compressed
            if kind == "streaming" and camera_view is not None:
                camera_view = self._compress(camera_view, camera_name)

        if camera_view is None:  # camera still warming up
            return
        self._images_handler.save_camera_image(camera_view, camera_name=camera_name, kind=kind)

    def transmit_lander_camera_view(self):
        if self._lander_cam is None:
            return

        camera_view: Image = self._snap_lander_camera_view()
        if camera_view is None:
            return
        self._images_handler.save_image(camera_view, role="lander")

    def transmit_monitoring_camera_view(self):
        camera_view: Image = self._snap_monitoring_camera_view()

        if camera_view is None:
            return

        self._images_handler.save_image(camera_view, role="monitoring")
