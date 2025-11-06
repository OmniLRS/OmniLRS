__author__ = "Aleksa Stanivuk, Cristian-Augustin Susanu"
__status__ = "development"

from typing import Dict
import os
import omni
from pxr import UsdGeom, Gf
import omni.replicator.core as rep
import omni.syntheticdata as sd
from src.environments.utils import set_xform_pose

class MonitoringCamerasManager:
    """
    Spawns one camera per entry under monitoring camera configs in yaml.
    """

    def __init__(self, cameras_cfg):
        self._cfg = cameras_cfg
        self._root_path = cameras_cfg["root_path"]
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn(self):
        if "camera_definitions" not in self._cfg:
            return 

        for c in self._cfg["camera_definitions"]:
            name = c["name"]
            prim_path = os.path.join(self._root_path, name)
            prim = self._stage.DefinePrim(prim_path, "Camera") # "Camera" here is the USD type identifier, not name of the camera
            cam = UsdGeom.Camera(prim)

            prim = self._stage.GetPrimAtPath(prim_path)
            xform = UsdGeom.Xformable(prim)

            pose = c["pose"]
            set_xform_pose(xform,
                # prim_path,
                pose["position"],
                pose["orientation"],
            )
            self._set_camera_attributes(cam, c["camera_params"])
            self._set_publisher( prim_path, c["ros2"])

    def _set_publisher(self, prim_path, ros2_cfg:Dict):
        # inspired by https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/tutorial_ros2_camera_publishing.html
        # but simplified
        rp = rep.create.render_product(prim_path, (ros2_cfg["resolution"][0], ros2_cfg["resolution"][1])) 

        rv = sd.SyntheticData.convert_sensor_type_to_rendervar("Rgb")
        w = rep.writers.get(rv + "ROS2PublishImage")
        w.initialize(topicName=ros2_cfg["topic"], frameId=ros2_cfg["frame_id"], queueSize=1, nodeNamespace="")
        w.attach([rp])

    def  _set_camera_attributes(self, camera, cfg_params):
        camera.CreateFocalLengthAttr().Set(float(cfg_params["focal_length"]))
        camera.CreateHorizontalApertureAttr().Set(float(cfg_params["horizontal_aperture"]))
        camera.CreateVerticalApertureAttr().Set(float(cfg_params["vertical_aperture"]))
        camera.CreateFStopAttr().Set(float(cfg_params["fstop"]))
        camera.CreateFocusDistanceAttr().Set(float(cfg_params["focus_distance"]))

        clip_near, clip_far  = (cfg_params["clipping_range"] + [None, None])[:2]
        clip_near = float(clip_near)
        clip_far  = float(clip_far)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(clip_near, clip_far))