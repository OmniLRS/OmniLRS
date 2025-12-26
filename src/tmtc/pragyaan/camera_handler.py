__author__ = "Aleksa Stanivuk"
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
    # reflects the buckets defined in yaml
    BUCKET_STREAMING = "images_streaming"
    BUCKET_ONCOMMAND = "images_oncommand"
    BUCKET_DEPTH = "images_depth"
    BUCKET_LANDER = "images_lander"
    BUCKET_MONITORING = "images_monitoring"

    def __init__(self, images_handler:ImagesHandler, robot, lander_camera_conf=None) -> None:
        self._images_handler = images_handler
        self._robot = robot
        self._initialize_lander_cam(lander_camera_conf)
        self._initialize_monitoring_cam()

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

        near, far = 0.0, 10.0  # fixed 0-20 m range for consistent depth scaling

        d = np.clip(depth, near, far)
        d = (d - near) / (far - near)
        d = (1.0 - d) * 255.0 
        d_uint8 = d.astype(np.uint8)

        return Image.fromarray(d_uint8, mode="L")
    
    def transmit_lander_camera_view(self):
        if self.lander_cam == None:
            return

        camera_view:Image = self._snap_lander_camera_view()
        self._images_handler.save_image(camera_view, self.BUCKET_LANDER)

    def transmit_monitoring_camera_view(self):
        camera_view:Image = self._snap_monitoring_camera_view()

        if camera_view == None:
            return
        
        self._images_handler.save_image(camera_view, self.BUCKET_MONITORING)

    def _snap_lander_camera_view(self) -> Image:
        frame = self.lander_cam.get_rgba()
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGBA.value)

        return camera_view

    def _snap_monitoring_camera_view(self) -> Image:
        if self.monitoring_cam == None:
            return
        
        frame = self.monitoring_cam.get_rgb()#get_rgba()

        #NOTE interval tries to trigger it before initialization, and then breaks because programmatically created cameras
        # require more time to be initialized than the ones already existing inside the models
        if (len(frame) == 0):
            return None
        
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, CameraViewType.RGB.value)

        return camera_view
    
    def _initialize_lander_cam(self, lander_camera_conf) -> None:
        if (lander_camera_conf == None):
            return
        
        self.lander_cam = Camera(lander_camera_conf["prim_path"], 
                                resolution=(lander_camera_conf["resolution"][0], lander_camera_conf["resolution"][1]))
        self.lander_cam.initialize()
    
    def _initialize_monitoring_cam(self) -> None:
        #NOTE TODO implemented for one camera right now, in the future make for multiple idea: dict same as for cameras in MonCamManager, 
        # and just iterate over them, make generic names that differ only in _id (mon_cam_1, mon_cam_2, ...) and make such yamcs buckets
        if (list(MonitoringCamerasManager.cameras.keys()) == 0):
            return

        camera_name = list(MonitoringCamerasManager.cameras.keys())[0]
        self.monitoring_cam = MonitoringCamerasManager.cameras[camera_name]