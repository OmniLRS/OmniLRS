"""Rebuild the Pragyaan robot USD from its source copy (docs/todo_pragyaan_physics.md).

nograph_pragyaan_source.usd is the asset as exported from Blender, kept unchanged. It references rocker_bogie.usd
(which references motor_black.usd) and takes its textures from chandrayaan_3_rover.usdz. This script writes
nograph_pragyaan.usd, the file that cfg/robot/pragyaan.yaml loads, as one flattened layer with the fix steps applied.
Each run starts again from the source copy, so the script is safe to run twice. It ends with checks of the result
against the source and stops with an error if one fails.

    pixi run -e yamcs python scripts/assets/rebuild_pragyaan.py

Steps:
  1. Mass properties. The real rover weighs 26 kg. Without authored masses, PhysX used its default density of
     1000 kg/m3 on the collider volumes (675 kg in total), and base_link, which has no collider, got a 1 kg fallback
     and the "invalid inertia tensor ... negative mass" warning. Each rigid body gets a mass. PhysX computes the
     inertia and the center of mass from the colliders of the body (also from disabled colliders, as on the solar
     panel). base_link gets an explicit small inertia and center of mass.
  2. Structure and frames, same size and same world placement of every part:
     - one flattened layer, without the unused /FlatGrid (stray physics scene, Isaac Sim 5.0 S3 reference) and /Render;
     - root frame REP-103 (x forward, y left, z up), origin unchanged (base_link, near the chassis center). The old
       root frame was the Blender Y-up frame (x left, y up, z forward) turned by the root transform;
     - no scale on any Xform, rigid body or camera: each mesh transform (Blender scales up to 186, five mirrored
       meshes) is baked into its points, normals and face order, so that each mesh has an identity transform;
     - rigid body frames aligned with the root, with their origin on their joint: rockers and bogies on their pivot,
       wheels on their axle center (spin angle 0), solar panel on its hinge;
     - joint frames expressed again in the new body frames, with the same world pose and axis. The rocker joints use
       the Body rigid body as body0 (was the body_top collider mesh);
     - base_link at the pose that PhysX gave it at runtime (REP-103), so the FixedJoint has identity frames. The old
       FixedJoint had a 120 deg rotation that PhysX applied to base_link and its IMU on the first physics step;
     - MaterialBindingAPI on each prim with a material binding.
"""

import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = REPO_ROOT / "assets" / "USD_Assets" / "robots" / "pragyaan"
SOURCE = ASSET_DIR / "nograph_pragyaan_source.usd"
OUTPUT = ASSET_DIR / "nograph_pragyaan.usd"
ROOT = "/Pragyaan"
UNUSED_ROOT_PRIMS = ["/FlatGrid", "/Render"]

TOTAL_MASS_KG = 26.0
# kg per rigid body, relative to ROOT. Split agreed in docs/todo_pragyaan_physics.md (D2).
MASSES_KG = {
    "Body": 16.99,
    "solarPanel": 1.5,
    "Mobility/Wheel_front_left": 0.75,
    "Mobility/Wheel_middle_left": 0.75,
    "Mobility/Wheel_back_left": 0.75,
    "Mobility/Wheel_front_right": 0.75,
    "Mobility/Wheel_middle_right": 0.75,
    "Mobility/Wheel_back_right": 0.75,
    "rocker_bogie/rocker_left": 0.9,
    "rocker_bogie/rocker_right": 0.9,
    "rocker_bogie/bogie_left": 0.6,
    "rocker_bogie/bogie_right": 0.6,
    "base_link": 0.01,
}
# base_link (IMU mount) has no collider, so PhysX cannot compute its inertia: a 1 cm sphere-like value is authored.
BASE_LINK_INERTIA_KGM2 = 1e-6

# Old root frame (Blender Y-up: x left, y up, z forward) to REP-103 (x forward, y left, z up). USD matrices act on
# row vectors (p' = p * M), so row i is the image of the old axis i: x -> y, y -> z, z -> x.
OLD_ROOT_TO_REP103 = Gf.Matrix4d(0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1)
FIXED_JOINT = f"{ROOT}/base_link/FixedJoint"
# Tolerances of the checks against the source.
POINT_TOLERANCE_M = 1e-5
ANGLE_TOLERANCE_DEG = 1e-3


# Step 1: mass properties.


def apply_masses(stage: Usd.Stage) -> None:
    rigid_bodies = {p.GetPath().pathString for p in rigid_body_prims(stage)}
    configured = {f"{ROOT}/{rel}" for rel in MASSES_KG}
    if rigid_bodies != configured:
        sys.exit(f"Rigid bodies and MASSES_KG differ: {sorted(rigid_bodies ^ configured)}")
    if abs(sum(MASSES_KG.values()) - TOTAL_MASS_KG) > 1e-6:
        sys.exit(f"MASSES_KG sums to {sum(MASSES_KG.values())} kg, expected {TOTAL_MASS_KG} kg")

    for rel, mass in MASSES_KG.items():
        mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(f"{ROOT}/{rel}"))
        mass_api.CreateMassAttr().Set(mass)
    base_link = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{ROOT}/base_link"))
    base_link.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(BASE_LINK_INERTIA_KGM2))
    base_link.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0))
    base_link.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))


# Step 2: structure and frames.


def rigid_body_prims(stage: Usd.Stage) -> list:
    return [p for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def joint_prims(stage: Usd.Stage) -> list:
    return [p for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)) if p.IsA(UsdPhysics.Joint)]


def rigid(matrix: Gf.Matrix4d) -> Gf.Matrix4d:
    """The rotation and translation of `matrix`, without its scale (and without its mirror, if any)."""
    transform = Gf.Transform(matrix)
    out = Gf.Matrix4d(1.0)
    out.SetRotateOnly(transform.GetRotation())
    out.SetTranslateOnly(transform.GetTranslation())
    return out


def translation(position: Gf.Vec3d) -> Gf.Matrix4d:
    out = Gf.Matrix4d(1.0)
    out.SetTranslateOnly(position)
    return out


def rotation_of(matrix: Gf.Matrix4d) -> Gf.Matrix4d:
    out = rigid(matrix)
    out.SetTranslateOnly(Gf.Vec3d(0.0))
    return out


def joint_frame(stage: Usd.Stage, xforms: UsdGeom.XformCache, joint: UsdPhysics.Joint, side: int) -> Gf.Matrix4d:
    """World matrix of the joint frame, seen from body0 (side 0) or body1 (side 1), as PhysX builds it."""
    body = (joint.GetBody0Rel() if side == 0 else joint.GetBody1Rel()).GetTargets()[0]
    position = (joint.GetLocalPos0Attr() if side == 0 else joint.GetLocalPos1Attr()).Get()
    quat = (joint.GetLocalRot0Attr() if side == 0 else joint.GetLocalRot1Attr()).Get()
    body_world = xforms.GetLocalToWorldTransform(stage.GetPrimAtPath(body))
    frame = Gf.Matrix4d(1.0)
    frame.SetRotateOnly(Gf.Rotation(Gf.Quatd(quat.GetReal(), Gf.Vec3d(quat.GetImaginary()))))
    frame = frame * rotation_of(body_world)
    # The local position is in the body frame, with the body scale (PhysX scales it).
    frame.SetTranslateOnly(body_world.Transform(Gf.Vec3d(position)))
    return frame


def set_local_transform(prim: Usd.Prim, local: Gf.Matrix4d | None) -> None:
    """Replaces the xform ops of `prim` by a translate and an orient op, or by no op when `local` is None."""
    for attr in prim.GetAttributes():
        if attr.GetName().startswith("xformOp:") or attr.GetName() == "xformOpOrder":
            prim.RemoveProperty(attr.GetName())
    if local is None:
        return
    xformable = UsdGeom.Xformable(prim)
    transform = Gf.Transform(local)
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(transform.GetTranslation())
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(transform.GetRotation().GetQuat())


def bake_mesh(mesh: UsdGeom.Mesh, to_parent: Gf.Matrix4d) -> bool:
    """Bakes the matrix `to_parent` into the points and normals of `mesh`. Returns True if the mesh was mirrored."""
    matrix = np.array(to_parent)
    linear, offset = matrix[:3, :3], matrix[3, :3]
    points = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((points @ linear + offset).astype(np.float32)))
    normal_matrix = np.linalg.inv(linear).T

    def transform_normals(values):
        normals = np.array(values, dtype=np.float64) @ normal_matrix
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        return Vt.Vec3fArray.FromNumpy(normals.astype(np.float32))

    normals_attr = mesh.GetNormalsAttr()
    if normals_attr.HasAuthoredValue():
        normals_attr.Set(transform_normals(normals_attr.Get()))
    primvars = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    if primvars.HasPrimvar("normals"):
        primvar = primvars.GetPrimvar("normals")
        primvar.Set(transform_normals(primvar.Get()))

    mirrored = np.linalg.det(linear) < 0
    if mirrored:
        # A mirror turns the faces inside out: reverse the vertex order of each face (keeping its first vertex), and
        # the face-varying values in the same way.
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
        order = []
        start = 0
        for count in counts:
            order += [start] + list(range(start + count - 1, start, -1))
            start += count
        order = np.array(order)
        indices = np.array(mesh.GetFaceVertexIndicesAttr().Get())
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(indices[order].astype(np.int32)))
        if normals_attr.HasAuthoredValue() and mesh.GetNormalsInterpolation() == UsdGeom.Tokens.faceVarying:
            values = np.array(normals_attr.Get())
            normals_attr.Set(Vt.Vec3fArray.FromNumpy(values[order].astype(np.float32)))
        for primvar in primvars.GetPrimvars():
            if primvar.GetInterpolation() != UsdGeom.Tokens.faceVarying:
                continue
            if primvar.IsIndexed():
                primvar.SetIndices(Vt.IntArray.FromNumpy(np.array(primvar.GetIndices())[order].astype(np.int32)))
            else:
                values = primvar.Get()
                primvar.Set(type(values)([values[int(i)] for i in order]))
    mesh.GetExtentAttr().Set(UsdGeom.Mesh.ComputeExtent(mesh.GetPointsAttr().Get()))
    return mirrored


def clean_frames(source: Usd.Stage, stage: Usd.Stage) -> dict:
    """
    Step 2 on `stage` (a flattened copy of `source`). Returns the old-to-new world matrix and the new world matrix of
    each Xformable prim, for the checks.
    """
    xforms = UsdGeom.XformCache()
    old_world = {p.GetPath(): xforms.GetLocalToWorldTransform(p) for p in source.Traverse() if p.IsA(UsdGeom.Xformable)}
    # Old asset world -> new asset frame: undo the old root transform, then turn to REP-103.
    to_new = old_world[Sdf.Path(ROOT)].GetInverse() * OLD_ROOT_TO_REP103

    # Joint frames in the new asset frame, from body0 (the side of each joint that PhysX did not move at startup).
    frames = {}
    for prim in joint_prims(source):
        frames[prim.GetPath()] = joint_frame(source, xforms, UsdPhysics.Joint(prim), 0) * to_new
    # Each rigid body that is body1 of a revolute joint gets its origin on that joint.
    origin_joint = {}
    for prim in joint_prims(source):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            origin_joint[UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()[0]] = prim.GetPath()

    base_link = Sdf.Path(f"{ROOT}/base_link")
    # Groups of rigid bodies (Mobility, rocker_bogie) get the root frame, so the rigid body transforms read directly
    # in REP-103.
    groups = {p.GetPath().GetParentPath() for p in rigid_body_prims(stage)}
    new_world = {}
    mirrored = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
        if not prim.IsA(UsdGeom.Xformable):
            continue
        path = prim.GetPath()
        parent_new = new_world.get(path.GetParentPath(), Gf.Matrix4d(1.0))
        target = old_world[path] * to_new
        if path == Sdf.Path(ROOT) or path == base_link:
            # base_link: the REP-103 pose that PhysX gave it at runtime, at its old origin.
            world = translation(target.ExtractTranslation())
        elif path in groups:
            world = Gf.Matrix4d(1.0)
        elif path.GetParentPath() == base_link:
            # The IMU keeps its pose relative to base_link (it moved with base_link at runtime).
            world = (old_world[path] * old_world[base_link].GetInverse()) * parent_new
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            origin = frames[origin_joint[path]].ExtractTranslation() if path in origin_joint else None
            world = translation(origin if origin is not None else target.ExtractTranslation())
        elif prim.IsA(UsdGeom.Mesh):
            world = parent_new
            if bake_mesh(UsdGeom.Mesh(prim), target * parent_new.GetInverse()):
                mirrored.append(path.pathString)
        else:
            world = rigid(target)
        new_world[path] = world
        local = world * parent_new.GetInverse()
        set_local_transform(prim, None if prim.IsA(UsdGeom.Mesh) or local == Gf.Matrix4d(1.0) else local)

    for prim in joint_prims(stage):
        joint = UsdPhysics.Joint(prim)
        if [str(t) for t in joint.GetBody0Rel().GetTargets()] == [f"{ROOT}/Body/body_top"]:
            joint.GetBody0Rel().SetTargets([Sdf.Path(f"{ROOT}/Body")])
        frame = Gf.Matrix4d(1.0) if prim.GetPath() == Sdf.Path(FIXED_JOINT) else frames[prim.GetPath()]
        for side, rel in ((0, joint.GetBody0Rel()), (1, joint.GetBody1Rel())):
            local = frame * new_world[rel.GetTargets()[0]].GetInverse()
            quat = Gf.Quatf(local.ExtractRotationQuat())
            position = Gf.Vec3f(local.ExtractTranslation())
            (joint.GetLocalPos0Attr() if side == 0 else joint.GetLocalPos1Attr()).Set(position)
            (joint.GetLocalRot0Attr() if side == 0 else joint.GetLocalRot1Attr()).Set(quat)
    print(f"Baked {len(mirrored)} mirrored meshes: {', '.join(Sdf.Path(p).name for p in mirrored)}")
    return {"to_new": to_new, "frames": frames}


def apply_material_binding_api(stage: Usd.Stage) -> None:
    for prim in stage.Traverse():
        has_binding = any(r.GetName().startswith("material:binding") for r in prim.GetRelationships())
        if has_binding and not prim.HasAPI(UsdShade.MaterialBindingAPI):
            UsdShade.MaterialBindingAPI.Apply(prim)


# Checks of the result against the source.


def check(output: Usd.Stage, source: Usd.Stage, to_new: Gf.Matrix4d, frames: dict) -> None:
    failures = []
    new_x, old_x = UsdGeom.XformCache(), UsdGeom.XformCache()

    def world_points(stage, xforms, prim, extra):
        matrix = np.array(xforms.GetLocalToWorldTransform(prim) * extra)
        return np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=np.float64) @ matrix[:3, :3] + matrix[3, :3]

    worst = 0.0
    for prim in Usd.PrimRange(output.GetPrimAtPath(ROOT)):
        old = source.GetPrimAtPath(prim.GetPath())
        if prim.IsA(UsdGeom.Xformable):
            scale = Gf.Transform(new_x.GetLocalToWorldTransform(prim)).GetScale()
            if any(abs(s - 1.0) > 1e-6 for s in scale):
                failures.append(f"{prim.GetPath()}: world scale {tuple(scale)}")
        if prim.IsA(UsdGeom.Mesh):
            error = np.abs(
                world_points(output, new_x, prim, Gf.Matrix4d(1.0)) - world_points(source, old_x, old, to_new)
            )
            worst = max(worst, float(error.max()))
        if prim.GetTypeName() == "Camera":
            new_pose = new_x.GetLocalToWorldTransform(prim)
            old_pose = rigid(old_x.GetLocalToWorldTransform(old) * to_new)
            moved = (new_pose.ExtractTranslation() - old_pose.ExtractTranslation()).GetLength()
            turned = (new_pose.ExtractRotation() * old_pose.ExtractRotation().GetInverse()).GetAngle()
            if moved > POINT_TOLERANCE_M or min(turned, 360 - turned) > ANGLE_TOLERANCE_DEG:
                failures.append(f"{prim.GetPath()}: camera moved {moved:.2e} m, turned {turned:.2e} deg")
    if worst > POINT_TOLERANCE_M:
        failures.append(f"mesh points moved up to {worst:.2e} m")
    for prim in joint_prims(output):
        joint = UsdPhysics.Joint(prim)
        frame0, frame1 = joint_frame(output, new_x, joint, 0), joint_frame(output, new_x, joint, 1)
        moved = (frame0.ExtractTranslation() - frame1.ExtractTranslation()).GetLength()
        turned = (frame0.ExtractRotation() * frame1.ExtractRotation().GetInverse()).GetAngle()
        if moved > POINT_TOLERANCE_M or min(turned, 360 - turned) > ANGLE_TOLERANCE_DEG:
            failures.append(f"{prim.GetPath()}: joint frames differ by {moved:.2e} m and {turned:.2e} deg")
        if prim.GetPath() != Sdf.Path(FIXED_JOINT):
            expected = frames[prim.GetPath()]
            moved = (frame0.ExtractTranslation() - expected.ExtractTranslation()).GetLength()
            turned = (frame0.ExtractRotation() * expected.ExtractRotation().GetInverse()).GetAngle()
            if moved > POINT_TOLERANCE_M or min(turned, 360 - turned) > ANGLE_TOLERANCE_DEG:
                failures.append(f"{prim.GetPath()}: joint moved {moved:.2e} m, turned {turned:.2e} deg in the world")
    for rel, mass in MASSES_KG.items():
        if UsdPhysics.MassAPI(output.GetPrimAtPath(f"{ROOT}/{rel}")).GetMassAttr().Get() != np.float32(mass):
            failures.append(f"{rel}: mass differs from MASSES_KG")
    for side in ("left", "right"):
        for position, sign_x in (("front", 1), ("back", -1)):
            wheel = new_x.GetLocalToWorldTransform(output.GetPrimAtPath(f"{ROOT}/Mobility/Wheel_{position}_{side}"))
            x, y, _ = wheel.ExtractTranslation()
            if x * sign_x <= 0 or y * (1 if side == "left" else -1) <= 0:
                failures.append(f"Wheel_{position}_{side} at ({x:.2f}, {y:.2f}): not in REP-103 order")
    if output.GetRootLayer().externalReferences:
        failures.append(f"composition arcs to other files: {sorted(output.GetRootLayer().externalReferences)}")
    for prim in output.Traverse():
        texture = prim.GetAttribute("inputs:file")
        if texture and texture.HasAuthoredValue() and not texture.Get().resolvedPath:
            failures.append(f"{prim.GetPath()}: texture {texture.Get().path} does not resolve")
    print(f"Mesh points match the source within {worst:.1e} m")
    if failures:
        sys.exit("Checks failed:\n  " + "\n  ".join(failures))
    print("Checks passed: unit scales, meshes, cameras, joints, masses, wheel layout, textures")


def main() -> None:
    source = Usd.Stage.Open(str(SOURCE))
    layer = source.Flatten()
    stage = Usd.Stage.Open(layer)
    for path in UNUSED_ROOT_PRIMS:
        stage.RemovePrim(path)
    apply_masses(stage)
    result = clean_frames(source, stage)
    apply_material_binding_api(stage)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    layer.Export(str(OUTPUT))

    output = Usd.Stage.Open(str(OUTPUT))
    total = 0.0
    for prim in rigid_body_prims(output):
        mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        total += mass
        print(f"  {prim.GetPath().pathString:40s} {mass:6.2f} kg")
    print(f"Wrote {OUTPUT.relative_to(REPO_ROOT)}: total mass {total:.2f} kg")
    check(output, source, result["to_new"], result["frames"])


if __name__ == "__main__":
    main()
