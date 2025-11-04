__author__ = "Aleksa Stanivuk, Cristian-Augustin Susanu"
__status__ = "development"

from pathlib import Path
from typing import Dict
import os
from assets import get_assets_path
import omni
from pxr import UsdGeom, Gf, UsdPhysics

class StaticAssetsManager:
    """
    Spawns one USD prim per entry under static assets configs in yaml.
    """

    def __init__(self, static_assets_cfg):
        self._cfg = static_assets_cfg
        self._root_path = static_assets_cfg["root_path"]
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn_from_config(self):
        if "parameters" not in self._cfg:
            return 

        for a in self._cfg["parameters"]:
            name = a["asset_name"]
            prim_path = os.path.join(self._root_path, name)
            self._create_reference(prim_path, a["usd_path"])
            pose = a.get("pose", {})
            self._set_pose(prim_path, pose.get("position", [0,0,0]), pose.get("orientation", [0,0,0,1]))
            self._set_collision(prim_path, a.get("collision", True))

    def _create_reference(self, prim_path: str, usd_path: str):
        assets_root = Path(get_assets_path())  
        real_usd_path = str(assets_root / usd_path.lstrip("/"))
        prim = self._stage.DefinePrim(prim_path, "Xform")       # this in essence creates an empty wrapper / holder 
        prim.GetReferences().AddReference(real_usd_path)         # that will reference to USD model in an external file

        return prim

    def _set_pose(self, prim_path: str, position, orientation):
        prim = self._stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)
        self._set_translate(xform, position)
        self._set_orientation(xform, orientation)

    def _set_translate(self, xform, position ):
        translate_op = None

        for op in xform.GetOrderedXformOps():
            t = op.GetOpType()
            if t == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
        
        if translate_op is None:
            translate_op = xform.AddTranslateOp()

        if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble: 
            translate_op.Set(Gf.Vec3d(Gf.Vec3d(position[0], position[1], position[2])))
        else:
            translate_op.Set(Gf.Vec3f(Gf.Vec3f(position[0], position[1], position[2])))

    def _set_orientation(self, xform, orientation ):
        #NOTE: why does precision depend on the format of orientation [x,y,z,w] ?
        orient_op = None

        for op in xform.GetOrderedXformOps():
            t = op.GetOpType()
            if t == UsdGeom.XformOp.TypeOrient:
                orient_op = op
        
        if orient_op is None:
            orient_op = xform.AddOrientOp()

        if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble: # NOTE: if orientation is as [1,0,0,0] then it is double, if [0,0,0,1] then it sees it as float -> why?
            q = Gf.Quatd(orientation[3], Gf.Vec3d(orientation[0], orientation[1], orientation[2])) 
        else:
            q = Gf.Quatf(orientation[3], Gf.Vec3f(orientation[0], orientation[1], orientation[2]))

        orient_op.Set(q)


    def _set_collision(self, prim_path: str, enabled: bool):
        if UsdPhysics is None:
            return

        # the asset may not be a single model, but it may consist of of multiple parts - therefore this function iterates over every part
        stack = [self._stage.GetPrimAtPath(prim_path)] 
        while stack:
            p = stack.pop()

            if not p or not p.IsValid():
                continue

            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(enabled)

            for c in p.GetChildren():
                stack.append(c)

    