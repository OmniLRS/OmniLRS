__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import os
from pathlib import Path

import omni
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from assets import get_assets_path


class StaticAssetsManager:
    """
    Spawns one USD prim per entry under static assets configs in yaml.
    """

    def __init__(self, static_assets_cfg):
        self._cfg = static_assets_cfg
        self._root_path = static_assets_cfg["root_path"]
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim(self._root_path, "Xform")

    def spawn(self, get_height_func=None):
        if "parameters" not in self._cfg:
            return

        for a in self._cfg["parameters"]:
            name = a["asset_name"]
            prim_path = os.path.join(self._root_path, name)
            self._create_reference(prim_path, a["usd_path"])
            pose = a.get("pose", {})

            position = list(pose["position"])
            if get_height_func is not None:
                position[2] = get_height_func((position[0], position[1])) + position[2]

            self._apply_pose(prim_path, position, pose["orientation"])
            self._set_collision(prim_path, a.get("collision", True))
            # Keep static assets fixed by default even if the referenced USD contains rigid bodies.
            self._set_simulation_enabled(prim_path, a.get("simulate_physics", False))

    def _apply_pose(self, prim_path: str, position, orientation):
        """
        Places the asset by authoring dedicated ``:placement`` translate/orient ops
        that are kept as the OUTERMOST ops in the xform stack. The referenced
        asset's own transform ops (scale/rotation/pivot from the source USD) are
        preserved as inner ops. This matches how the Isaac Sim UI places a prim,
        and avoids the placement translation being applied inside the asset's
        local (scaled/rotated) space, which caused the asset to end up at the
        wrong world position while the UI still showed the configured values.
        """
        prim = self._stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)

        # Ops coming from the referenced asset (everything that is not ours).
        asset_ops = [op for op in xform.GetOrderedXformOps() if not op.GetOpName().endswith(":placement")]

        translate_op = None
        orient_op = None
        for op in xform.GetOrderedXformOps():
            name = op.GetOpName()
            if name == "xformOp:translate:placement":
                translate_op = op
            elif name == "xformOp:orient:placement":
                orient_op = op

        if translate_op is None:
            translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "placement")
        if orient_op is None:
            orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble, "placement")

        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
        orient_op.Set(Gf.Quatd(orientation[3], Gf.Vec3d(orientation[0], orientation[1], orientation[2])))

        # Placement ops must be outermost so the pose is applied in parent/world
        # space; the asset's own ops stay inner to keep its geometry intact.
        xform.SetXformOpOrder([translate_op, orient_op] + asset_ops)

    def _create_reference(self, prim_path: str, usd_path: str):
        assets_root = Path(get_assets_path())
        real_usd_path = str(assets_root / usd_path.lstrip("/"))
        prim = self._stage.DefinePrim(prim_path, "Xform")  # this in essence creates an empty wrapper / holder
        prim.GetReferences().AddReference(real_usd_path)  # that will reference to USD model in an external file

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

    def _set_simulation_enabled(self, prim_path: str, enabled: bool):
        if UsdPhysics is None:
            return

        stack = [self._stage.GetPrimAtPath(prim_path)]
        while stack:
            p = stack.pop()

            if not p or not p.IsValid():
                continue

            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                rb_api = UsdPhysics.RigidBodyAPI(p)
                rb_api.CreateRigidBodyEnabledAttr().Set(enabled)
                # If simulation is disabled, keep bodies kinematic to prevent solver updates.
                if not enabled:
                    rb_api.CreateKinematicEnabledAttr().Set(True)

            if p.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                physx_rb_api = PhysxSchema.PhysxRigidBodyAPI(p)
                physx_rb_api.CreateDisableGravityAttr().Set(not enabled)

            for c in p.GetChildren():
                stack.append(c)
