"""Rebuild the Vikram lander USD from its source copy (docs/todo_pragyaan_physics.md, Phase 3).

Vikram_source.usd is the lander asset before this rebuild, kept unchanged. It takes its textures from
Chandrayaan_3_vikram_lander.usdz. This script writes Vikram.usd, the file that the workshop environments load as the
static asset "lander", as one flattened layer:
  - without the unused /Render and /GroundPlane;
  - Z-up (the source is Y-up), x unchanged;
  - no scale on any Xform or camera: each mesh transform (Blender scales of 0.5, 36 and 50) is baked into its points
    and normals, so that each mesh has an identity transform;
  - at the published size: the main body (foil box Plane_009_foil_0) measures 2.35 x 2.40 m and 1.34 m high in the
    source, and Vikram measures 2.0 x 2.0 m and 1.166 m high. Every point and position is scaled by SCALE about the
    root origin, where the environments place the lander.
It ends with checks of the result against the source and stops with an error if one fails.

    pixi run -e yamcs python scripts/assets/rebuild_vikram.py

The environments place the lander with an orientation (x, y, z, w, read by static_assets_manager._apply_pose) outside
the asset transform. ENV_ORIENTATION_OLD on the source and ENV_ORIENTATION_NEW on the output give the same world pose.
"""

import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom
from usd_bake import apply_material_binding_api, bake_mesh, rigid, set_local_transform, world_points

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = REPO_ROOT / "assets" / "USD_Assets" / "common" / "lander" / "Vikram"
SOURCE = ASSET_DIR / "Vikram_source.usd"
OUTPUT = ASSET_DIR / "Vikram.usd"
ROOT = "/Vikram"
UNUSED_ROOT_PRIMS = ["/Render", "/GroundPlane"]
BODY_MESH = f"{ROOT}/Lander/Plane_009/Plane_009_foil_0/Plane_009_foil_0"
PUBLISHED_BODY_M = (2.0, 2.0, 1.166)  # x, y, z (height) in the output frame
# Mean of the three ratios published / source body size (0.850, 0.833, 0.873).
SCALE = 0.85
# Y-up to Z-up, x unchanged. Row i is the image of the old axis i: x -> x, y -> z, z -> -y.
Y_UP_TO_Z_UP = Gf.Matrix4d(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1)
# Environment placement (x, y, z, w) of the source and of the output, for the same world pose.
ENV_ORIENTATION_OLD = (0.5, 0.5, 0.5, 0.5)
ENV_ORIENTATION_NEW = (0.0, 0.0, 0.70711, 0.70711)
POINT_TOLERANCE_M = 1e-5
ANGLE_TOLERANCE_DEG = 1e-3


def placement(orientation_xyzw: tuple) -> Gf.Matrix4d:
    x, y, z, w = orientation_xyzw
    out = Gf.Matrix4d(1.0)
    out.SetRotateOnly(Gf.Quatd(w, Gf.Vec3d(x, y, z)).GetNormalized())
    return out


def clean(source: Usd.Stage, stage: Usd.Stage) -> Gf.Matrix4d:
    """Z-up, baked meshes and SCALE on `stage` (a flattened copy of `source`). Returns the old-to-new matrix."""
    xforms = UsdGeom.XformCache()
    old_world = {p.GetPath(): xforms.GetLocalToWorldTransform(p) for p in source.Traverse() if p.IsA(UsdGeom.Xformable)}
    to_new = old_world[Sdf.Path(ROOT)].GetInverse() * Y_UP_TO_Z_UP * Gf.Matrix4d(1.0).SetScale(SCALE)
    new_world = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
        if not prim.IsA(UsdGeom.Xformable):
            continue
        path = prim.GetPath()
        parent_new = new_world.get(path.GetParentPath(), Gf.Matrix4d(1.0))
        target = old_world[path] * to_new
        if path == Sdf.Path(ROOT):
            world = Gf.Matrix4d(1.0)
        elif prim.IsA(UsdGeom.Mesh):
            world = parent_new
            bake_mesh(UsdGeom.Mesh(prim), target * parent_new.GetInverse())
        else:
            world = rigid(target)
        new_world[path] = world
        local = world * parent_new.GetInverse()
        set_local_transform(prim, None if prim.IsA(UsdGeom.Mesh) or local == Gf.Matrix4d(1.0) else local)
    return to_new


def check(output: Usd.Stage, source: Usd.Stage) -> None:
    failures = []
    new_x, old_x = UsdGeom.XformCache(), UsdGeom.XformCache()
    # World pose in an environment: the placement is outside the asset transform.
    old_env, new_env = placement(ENV_ORIENTATION_OLD), placement(ENV_ORIENTATION_NEW)
    scale = Gf.Matrix4d(1.0).SetScale(SCALE)
    worst = 0.0
    for prim in Usd.PrimRange(output.GetPrimAtPath(ROOT)):
        old = source.GetPrimAtPath(prim.GetPath())
        if prim.IsA(UsdGeom.Xformable):
            factors = Gf.Transform(new_x.GetLocalToWorldTransform(prim)).GetScale()
            if any(abs(f - 1.0) > 1e-6 for f in factors):
                failures.append(f"{prim.GetPath()}: world scale {tuple(factors)}")
        if prim.IsA(UsdGeom.Mesh):
            expected = world_points(old_x, old, old_env * scale)
            worst = max(worst, float(np.abs(world_points(new_x, prim, new_env) - expected).max()))
        if prim.GetTypeName() == "Camera":
            new_pose = new_x.GetLocalToWorldTransform(prim) * new_env
            old_pose = rigid(old_x.GetLocalToWorldTransform(old) * old_env * scale)
            moved = (new_pose.ExtractTranslation() - old_pose.ExtractTranslation()).GetLength()
            turned = (new_pose.ExtractRotation() * old_pose.ExtractRotation().GetInverse()).GetAngle()
            if moved > POINT_TOLERANCE_M or min(turned, 360 - turned) > ANGLE_TOLERANCE_DEG:
                failures.append(f"{prim.GetPath()}: camera moved {moved:.2e} m, turned {turned:.2e} deg")
    if worst > POINT_TOLERANCE_M:
        failures.append(f"mesh points differ from the scaled source in the environment by up to {worst:.2e} m")
    body = world_points(new_x, output.GetPrimAtPath(BODY_MESH))
    size = body.max(0) - body.min(0)
    print(f"Main body {size[0]:.3f} x {size[1]:.3f} m, {size[2]:.3f} m high (published {PUBLISHED_BODY_M})")
    if any(abs(s - p) / p > 0.05 for s, p in zip(size, PUBLISHED_BODY_M)):
        failures.append(f"main body size {np.round(size, 3)} differs from {PUBLISHED_BODY_M} by more than 5 %")
    if UsdGeom.GetStageUpAxis(output) != UsdGeom.Tokens.z:
        failures.append("stage up axis is not Z")
    if output.GetRootLayer().externalReferences:
        failures.append(f"composition arcs to other files: {sorted(output.GetRootLayer().externalReferences)}")
    for prim in output.Traverse():
        texture = prim.GetAttribute("inputs:file")
        if texture and texture.HasAuthoredValue() and not texture.Get().resolvedPath:
            failures.append(f"{prim.GetPath()}: texture {texture.Get().path} does not resolve")
    print(f"Mesh points match the scaled source in the environment within {worst:.1e} m")
    if failures:
        sys.exit("Checks failed:\n  " + "\n  ".join(failures))
    print("Checks passed: unit scales, meshes, camera, body size, up axis, textures")


def main() -> None:
    source = Usd.Stage.Open(str(SOURCE))
    layer = source.Flatten()
    stage = Usd.Stage.Open(layer)
    for path in UNUSED_ROOT_PRIMS:
        stage.RemovePrim(path)
    clean(source, stage)
    apply_material_binding_api(stage)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # Written next to OUTPUT (same relative texture paths), and moved to OUTPUT only if the checks pass.
    candidate = OUTPUT.with_name(OUTPUT.stem + ".candidate.usd")
    layer.Export(str(candidate))
    output = Usd.Stage.Open(str(candidate))
    try:
        check(output, source)
    finally:
        del output
    candidate.replace(OUTPUT)
    print(f"Wrote {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
