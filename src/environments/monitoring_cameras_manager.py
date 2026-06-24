__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import os

import omni
from isaacsim.sensors.camera import Camera
from pxr import Gf, UsdGeom

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.utils import set_xform_pose


class MonitoringCamerasManager:
    """
    Spawns one camera per entry under monitoring camera configs in yaml.
    """

    cameras = {}

    def __init__(self, mode: SimulatorMode, cameras_cfg):
        self._mode = mode
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
            prim = self._stage.DefinePrim(
                prim_path, "Camera"
            )  # "Camera" here is the USD type identifier, not name of the camera
            cam = UsdGeom.Camera(prim)

            prim = self._stage.GetPrimAtPath(prim_path)
            xform = UsdGeom.Xformable(prim)

            pose = c["pose"]
            set_xform_pose(
                xform,
                # prim_path,prim_path
                pose["position"],
                pose["orientation"],
            )
            self._set_camera_attributes(cam, c["camera_params"])
            self.cameras[name] = Camera(
                prim_path,
                resolution=(c["resolution"][0], c["resolution"][1]),
            )
            self.cameras[name].initialize()
            # NOTE render products do not work, because it requires rep.orchestrator.step() which classhes with world.step()
            # rp = rep.create.render_product(prim_path, (c["resolution"][0], c["resolution"][1]))

            # TODO implement camera streaming in ROS mode

    def _set_camera_attributes(self, camera, cfg_params):
        camera.CreateFocalLengthAttr().Set(float(cfg_params["focal_length"]))
        camera.CreateHorizontalApertureAttr().Set(float(cfg_params["horizontal_aperture"]))
        camera.CreateVerticalApertureAttr().Set(float(cfg_params["vertical_aperture"]))
        camera.CreateFStopAttr().Set(float(cfg_params["fstop"]))
        camera.CreateFocusDistanceAttr().Set(float(cfg_params["focus_distance"]))

        clip_near, clip_far = (cfg_params["clipping_range"] + [None, None])[:2]
        clip_near = float(clip_near)
        clip_far = float(clip_far)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(clip_near, clip_far))
