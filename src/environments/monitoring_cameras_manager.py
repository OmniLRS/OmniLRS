__author__ = "Aleksa Stanivuk, Cristian-Augustin Susanu"
__status__ = "development"

from typing import Dict
import os
import omni
from pxr import UsdGeom, Gf

class MonitoringCamerasManager:
    """
    Spawns one camera per entry under monitoring camera configs in yaml.
    """

    def __init__(self, root_path: str = "/MonitoringCameras"):
        self._root_path = root_path
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn_from_config(self, monitoring_cameras: Dict):
        if not monitoring_cameras or "parameters" not in monitoring_cameras:
            return

        for c in monitoring_cameras["parameters"]:
            name = c["name"]
            prim_path = os.path.join(self._root_path, name)
            # prim = self._create_camera_prim(prim_path)
            prim = self._stage.DefinePrim(prim_path, "Camera") # "Camera" here is the USD type identifier, not name of the camera
            pose = c["pose"]
            self._set_pose(
                prim_path,
                pose.get("position", [0, 0, 0]),
                pose.get("orientation", [0, 0, 0, 1]),
            )

            resolution = c["resolution"]
            fov_deg = float(c["field_of_view_deg"])
            self._set_camera_intrinsics(prim, resolution, fov_deg)

            #TODO publish through ros2
    
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

    def _set_camera_intrinsics(self, prim, resolution, fov_deg: float):
        #NOTE: the following link describes camera intrinsics 
        # https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html?utm_source=chatgpt.com#isaaclab.sensors.Camera.set_intrinsic_matrices
        cam = UsdGeom.Camera(prim)
        width = int(resolution[0])
        height = int(resolution[1])

        aspect = max(width, 1) / max(height, 1) # pixel aspect #NOTE(Due to limitations of Omniverse camera, we need to assume that the camera is a spherical lens, i.e. has square pixels), reffer to above link

        #NOTE camera creation https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/usd/cameras/create-perspective-camera.html

        #NOTE what to do with resolution? -> it is used only for render_product ... https://youtu.be/T5Zlb5Ujtes?si=T3PVl_1t5bS1GmEE
        # 