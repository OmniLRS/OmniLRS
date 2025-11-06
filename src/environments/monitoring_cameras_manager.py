__author__ = "Aleksa Stanivuk, Cristian-Augustin Susanu"
__status__ = "development"

from typing import Dict
import os
import omni
from pxr import UsdGeom, Gf
import omni.replicator.core as rep
import omni.syntheticdata as sd

class MonitoringCamerasManager:
    """
    Spawns one camera per entry under monitoring camera configs in yaml.
    """

    def __init__(self, cameras_cfg):
        self._cfg = cameras_cfg
        self._root_path = cameras_cfg["root_path"]
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn_from_config(self):
        if "camera_definitions" not in self._cfg:
            return 

        for c in self._cfg["camera_definitions"]:
            name = c["name"]
            prim_path = os.path.join(self._root_path, name)
            prim = self._stage.DefinePrim(prim_path, "Camera") # "Camera" here is the USD type identifier, not name of the camera
            cam = UsdGeom.Camera(prim)
            pose = c["pose"]
            self._set_pose(
                prim_path,
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

    def _set_pose(self, prim_path: str, position, orientation):
        prim = self._stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)
        self._set_translate(xform, position)
        self._set_orientation(xform, orientation)

    def _set_translate(self, xform, position):
        translate_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xform.AddTranslateOp()

        if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
            translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
        else:
            translate_op.Set(Gf.Vec3f(position[0], position[1], position[2]))

    def _set_orientation(self, xform, orientation):
        orient_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
                break
        if orient_op is None:
            orient_op = xform.AddOrientOp()

        if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
            q = Gf.Quatd(orientation[3], Gf.Vec3d(orientation[0], orientation[1], orientation[2]))
        else:
            q = Gf.Quatf(orientation[3], Gf.Vec3f(orientation[0], orientation[1], orientation[2]))
        orient_op.Set(q)