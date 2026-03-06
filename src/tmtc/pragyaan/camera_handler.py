__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import Enum
from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.tmtc.yamcs_TMTC import ImagesHandler
import omni.kit.app
import numpy as np
from omni.isaac.sensor import Camera
from PIL import Image

class CameraViewType(Enum):
    RGBA = "RGBA"
    RGB = "RGB" # only for monitoring, as is more light weight
    DEPTH = "DEPTH"


class PragyaanCameraHandler:
    """
    This file contains methods for handling ALL cameras related to Rragyaan's setup. 

    Rover's own camera(s) are initialized in the file for robot initialization.
    This file initializes additional cameras in the environment, such as lander's camera, and monitoring camera.

    Camera handler contains the methods for snapping camera views of rover's camera, and lander's and monitoring camera.
    Rover's camera is able to snap DEPTH and RGB views.

    Handler also contains methods for transmitting snapped views to Yamcs, by utilizing saving methods of the images handler property. 

    CameraHandler is a layer on top of the ImagesHandler, and in that way is implemented a separation between controlling different
    instances of camera, and concers of low-level image saving to the Yamcs buckets and/or disk.
    Thus CameraHandler(s) are intended to reuse functionalities provided by the ImagesHandler, and not reimplement the communication 
    with Yamcs from the scratch.

    The reference to _robot provides access to rover's cameras, and desired use-case implementation.
    """
    
    # reflects the buckets defined in yaml
    BUCKET_STREAMING = "images_streaming"
    BUCKET_ONCOMMAND = "images_oncommand"
    BUCKET_DEPTH = "images_depth"
    BUCKET_LANDER = "images_lander"
    BUCKET_MONITORING = "images_monitoring"

    def __init__(self, images_handler:ImagesHandler, robot, lander_camera_conf=None) -> None:
        self._images_handler = images_handler
        self._robot = robot
        self._lander_cam = None
        self._monitoring_cam = None
        self._initialize_lander_cam(lander_camera_conf)
        self._initialize_monitoring_cam()


    def _initialize_lander_cam(self, lander_camera_conf) -> None:
        if (lander_camera_conf == None):
            return
        
        self._lander_cam = Camera(lander_camera_conf["prim_path"], 
                                resolution=(lander_camera_conf["resolution"][0], lander_camera_conf["resolution"][1]))
        self._lander_cam.initialize()
    
    def _initialize_monitoring_cam(self) -> None:
        #NOTE TODO implemented for one camera right now, in the future make for multiple idea: dict same as for cameras in MonCamManager, 
        # and just iterate over them, make generic names that differ only in _id (mon_cam_1, mon_cam_2, ...) and make such yamcs buckets
        if (len(list(MonitoringCamerasManager.cameras.keys())) == 0):
            return

        camera_name = list(MonitoringCamerasManager.cameras.keys())[0]
        self._monitoring_cam = MonitoringCamerasManager.cameras[camera_name]

    def _snap_camera_view_rgb(self, resolution:str) -> Image:
        frame = self._robot.get_rgba_camera_view(resolution)
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

        return camera_view
    
    def _snap_camera_view_depth(self, resolution:str) -> Image:
        frame = self._robot.get_depth_camera_view(resolution)
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
        if self._lander_cam == None:
            return
        
        frame = self._lander_cam.get_rgba()
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

        return camera_view

    def _snap_monitoring_camera_view(self) -> Image:
        if self._monitoring_cam == None:
            return
        
        frame = self._monitoring_cam.get_rgb()#get_rgba()

        #NOTE interval tries to trigger it before initialization, and then breaks because programmatically created cameras
        # require more time to be initialized than the ones already existing inside the models
        if (len(frame) == 0):
            return None
        
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGB.value)

        return camera_view
    
    def transmit_camera_view(self, bucket:str, resolution:str, type:CameraViewType=CameraViewType.RGBA):
        camera_view:Image = None

        if type == CameraViewType.DEPTH:
            camera_view:Image = self._snap_camera_view_depth(resolution)
        elif type == CameraViewType.RGBA:
            camera_view:Image = self._snap_camera_view_rgb(resolution)
        else:
            print("in transmit_camera_view: unknown type:", type)
            return
        
        self._images_handler.save_image(camera_view, bucket)
    
    def transmit_lander_camera_view(self):
        if self._lander_cam == None:
            return

        camera_view:Image = self._snap_lander_camera_view()
        self._images_handler.save_image(camera_view, self.BUCKET_LANDER)

    def transmit_monitoring_camera_view(self):
        camera_view:Image = self._snap_monitoring_camera_view()

        if camera_view == None:
            return
        
        self._images_handler.save_image(camera_view, self.BUCKET_MONITORING)