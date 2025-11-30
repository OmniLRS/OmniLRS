__author__ = "Aleksa Stanivuk, Cristian-Augustin Susanu"
__status__ = "development"

from pathlib import Path
from typing import Dict
import os
from assets import get_assets_path
import omni
from pxr import UsdGeom, UsdPhysics
from src.environments.utils import set_xform_pose

class StaticAssetsManager:
    """
    Spawns one USD prim per entry under static assets configs in yaml.
    """

    def __init__(self, static_assets_cfg):
        self._cfg = static_assets_cfg
        self._root_path = static_assets_cfg["root_path"]
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn(self):
        if "parameters" not in self._cfg:
            return 

        for a in self._cfg["parameters"]:
            name = a["asset_name"]
            prim_path = os.path.join(self._root_path, name)
            self._create_reference(prim_path, a["usd_path"])
            pose = a.get("pose", {})

            prim = self._stage.GetPrimAtPath(prim_path)
            xform = UsdGeom.Xformable(prim)

            set_xform_pose(xform, pose["position"], pose["orientation"])
            self._set_collision(prim_path, a.get("collision", True))

    def _create_reference(self, prim_path: str, usd_path: str):
        assets_root = Path(get_assets_path())  
        real_usd_path = str(assets_root / usd_path.lstrip("/"))
        prim = self._stage.DefinePrim(prim_path, "Xform")           # this in essence creates an empty wrapper / holder 
        prim.GetReferences().AddReference(real_usd_path)            # that will reference to USD model in an external file

        return prim

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

    