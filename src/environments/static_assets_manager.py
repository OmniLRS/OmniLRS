__author__ = "Aleksa Stanivuk, Cristian-Augustin Susanu"
__status__ = "development"

from typing import Dict
import os
import omni
from pxr import Usd, UsdGeom, Gf, UsdPhysics

class StaticAssetManager:
    """
    Spawns one USD prim per entry under /StaticAssets/<asset_name>.
    No instancers, no ROS. Pure stage composition via references.
    """

    def __init__(self, root_path: str = "/StaticAssets"):
        self._root_path = root_path
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn_from_config(self, static_assets: Dict):
        if not static_assets or "parameters" not in static_assets:
            return

        for a in static_assets["parameters"]:
            name = a["asset_name"]
            prim_path = os.path.join(self._root_path, name)
            self._create_reference(prim_path, a["usd_path"])
            pose = a.get("pose", {})
            self._set_pose(prim_path, pose.get("position", [0,0,0]), pose.get("orientation", [0,0,0,1]))
            self._set_collision(prim_path, a.get("collision", True))

    def _create_reference(self, prim_path: str, usd_path: str):
        prim = self._stage.DefinePrim(prim_path, "Xform")   # this in essence creates an empty wrapper / holder 
        prim.GetReferences().AddReference(usd_path)         # that will reference to USD model in an external file

        return prim

    def _set_pose(self, prim_path: str, position, orientation):
        # prim = self._stage.GetPrimAtPath(prim_path) # gets previously created prim of the static asset
        # UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(position[0], position[1], position[2]))
        
        # q = Gf.Quatf(orientation[3], Gf.Vec3f(orientation[0], orientation[1], orientation[2])) # q = (w, (x,y,z)) | w - angle of rotation; x,y,z - axis of rotation
        # UsdGeom.Xformable(prim).AddOrientOp().Set(q)

        # xform = UsdGeom.Xformable(prim)
        # orient_op = xform.GetOrderedXformOpsAttr().GetXformOp("xformOp:orient") # have to check first if orentOp already exists

        # if orient_op is None:
        #     orient_op = xform.AddOrientOp()
        # orient_op.Set(q)

        prim = self._stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)

        # Find existing ops (don’t recreate if present)
        translate_op = None
        orient_op = None
        # (scale_op only if you need scale)

        for op in xform.GetOrderedXformOps():
            t = op.GetOpType()
            if t == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif t == UsdGeom.XformOp.TypeOrient:
                orient_op = op
            # elif t == UsdGeom.XformOp.TypeScale:
            #     scale_op = op

        # Create missing ops in a clean order
        if translate_op is None:
            translate_op = xform.AddTranslateOp()
        if orient_op is None:
            orient_op = xform.AddOrientOp()
        # if scale_op is None:
        #     scale_op = xform.AddScaleOp()

        # Set values
        translate_op.Set(Gf.Vec3d(Gf.Vec3d(position[0], position[1], position[2])))
        q = Gf.Quatd(orientation[3], Gf.Vec3d(orientation[0], orientation[1], orientation[2])) # tried using Quatf and Vec3f (float), but had to use Quatd and Vec3d (double)
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

    