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
     - MaterialBindingAPI on each prim with a material binding. The rocker, bogie and motor meshes, whose bindings
       pointed outside the asset, get the silver_metal and black materials of the asset.
  3. Left/right symmetry about the plane y = 0 of the root frame. In the source, the rocker and bogie meshes are mirror
     copies about the plane y = +5 mm, the pivots and axles differ by up to 15 mm between the two sides, the wheels
     have different spin angles and the motors are rotated copies. The rover moves so that the plane of the rocker and
     bogie meshes is y = 0 (base_link stays at the root origin, on the plane). Each pair of pivots and axles gets the
     average of the two sides, mirrored. Each wheel mesh moves with its axle. Each left mesh is replaced by the
     mirrored right mesh.
  4. Real size. The Blender model is about two times too large (1.74 m long, published 0.917 m). Every point and
     position is scaled by SCALE about the root origin (D1: one factor, which matches the published length). The masses
     do not change and PhysX computes the inertia from the smaller colliders. The cooked collision data that the source
     stores (physxCookedData, a cache in the old mesh coordinates) is removed, so PhysX cooks the new meshes.
"""

import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from usd_bake import (
    apply_material_binding_api,
    bake_mesh,
    remove_cooked_data,
    rigid,
    rotation_of,
    set_local_transform,
    translation,
    world_points,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = REPO_ROOT / "assets" / "USD_Assets" / "robots" / "pragyaan"
SOURCE = ASSET_DIR / "nograph_pragyaan_source.usd"
OUTPUT = ASSET_DIR / "nograph_pragyaan.usd"
ROOT = "/Pragyaan"
UNUSED_ROOT_PRIMS = ["/FlatGrid", "/Render"]

# Step 4: uniform scale about the root origin (D1 in docs/todo_pragyaan_physics.md): published length 0.917 m / model
# length 1.74 m.
SCALE = 0.527

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
# Step 3: left body -> right body of each pair, relative to ROOT. The meshes of the wheels move with their axle. The
# rocker and bogie meshes (without the motors, which are rotated copies) define the plane of symmetry.
SYMMETRIC_PAIRS = {
    "rocker_bogie/rocker_left": "rocker_bogie/rocker_right",
    "rocker_bogie/bogie_left": "rocker_bogie/bogie_right",
    "Mobility/Wheel_front_left": "Mobility/Wheel_front_right",
    "Mobility/Wheel_middle_left": "Mobility/Wheel_middle_right",
    "Mobility/Wheel_back_left": "Mobility/Wheel_back_right",
}
MESHES_FOLLOW_AXLE = ("Mobility/Wheel_",)
PLANE_PARTS = ("rocker_bogie/rocker_left", "rocker_bogie/bogie_left")
PLANE_EXCLUDED_MESHES = ("motor_black",)
# Materials of the asset for the rocker_bogie meshes without a bound material (step 2), by mesh name ("*": others).
UNBOUND_MESH_MATERIALS = {"motor_black": "black", "*": "silver_metal"}
MIRROR_Y = Gf.Matrix4d(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
MAX_SYMMETRY_MOVE_M = 0.015  # largest move that step 3 may apply to a joint or a mesh
MAX_SYMMETRY_TURN_DEG = 0.5  # largest angle between the joint axes of a pair
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


def subtree_meshes(stage: Usd.Stage, rel: str) -> list:
    return [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ROOT}/{rel}")) if p.IsA(UsdGeom.Mesh)]


def rigid_body_of(prim: Usd.Prim) -> Usd.Prim | None:
    while prim and prim.GetPath() != Sdf.Path.absoluteRootPath:
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def symmetry_plane_y(source: Usd.Stage, xforms: UsdGeom.XformCache, to_new: Gf.Matrix4d) -> float:
    """y (in the frame of `to_new`) of the plane about which the rocker and bogie meshes are mirror copies."""
    mids = []
    for left in PLANE_PARTS:
        sides = []
        for rel in (left, SYMMETRIC_PAIRS[left]):
            meshes = [m for m in subtree_meshes(source, rel) if m.GetName() not in PLANE_EXCLUDED_MESHES]
            sides.append(np.vstack([world_points(xforms, m, to_new) for m in meshes])[:, 1].mean())
        mids.append((sides[0] + sides[1]) / 2)
    return float(np.mean(mids))


def symmetric_frames(source: Usd.Stage, frames: dict, origin_joint: dict) -> tuple[dict, dict]:
    """
    The joint frames with each left/right pair of pivots or axles at the average of the two sides, mirrored, and with
    the rotation of the right side. The frames of a pair can differ by a rotation about the joint axis (the old wheel
    spin angles): this only sets where the joint angle starts, and both local frames of each joint come from the same
    world frame, so the rest pose stays at angle 0. Also returns the move of each paired rigid body (path -> move).
    """
    frames = dict(frames)
    moves = {}
    flip = np.array([1.0, -1.0, 1.0])
    for left, right in SYMMETRIC_PAIRS.items():
        left_joint, right_joint = origin_joint[Sdf.Path(f"{ROOT}/{left}")], origin_joint[Sdf.Path(f"{ROOT}/{right}")]
        left_frame, right_frame = frames[left_joint], frames[right_joint]
        axis_name = source.GetPrimAtPath(right_joint).GetAttribute("physics:axis").Get()
        axis = Gf.Vec3d(*(1.0 if axis_name == name else 0.0 for name in "XYZ"))
        left_axis, right_axis = left_frame.TransformDir(axis), right_frame.TransformDir(axis)
        turned = np.degrees(np.arccos(np.clip(Gf.Dot(left_axis.GetNormalized(), right_axis.GetNormalized()), -1, 1)))
        if turned > MAX_SYMMETRY_TURN_DEG:
            sys.exit(f"{left} and {right}: joint axes turned {turned:.2f} deg from each other")
        left_position = np.array(left_frame.ExtractTranslation())
        right_position = np.array(right_frame.ExtractTranslation())
        average = (right_position + left_position * flip) / 2
        for rel, joint, old, new in (
            (left, left_joint, left_position, average * flip),
            (right, right_joint, right_position, average),
        ):
            frame = Gf.Matrix4d(right_frame)
            frame.SetTranslateOnly(Gf.Vec3d(*map(float, new)))
            frames[joint] = frame
            moves[f"{ROOT}/{rel}"] = new - old
            if np.linalg.norm(new - old) > MAX_SYMMETRY_MOVE_M:
                sys.exit(f"{rel}: symmetry moves its joint by {np.linalg.norm(new - old) * 1000:.1f} mm")
    return frames, moves


def clean_frames(source: Usd.Stage, stage: Usd.Stage) -> dict:
    """
    Steps 2 and 3 on `stage` (a flattened copy of `source`). Returns what the checks need: the old-to-new world matrix,
    the joint frames, the mesh moves and the mirrored mesh pairs.
    """
    xforms = UsdGeom.XformCache()
    old_world = {p.GetPath(): xforms.GetLocalToWorldTransform(p) for p in source.Traverse() if p.IsA(UsdGeom.Xformable)}
    # Old asset world -> new asset frame: undo the old root transform, then turn to REP-103.
    to_new = old_world[Sdf.Path(ROOT)].GetInverse() * OLD_ROOT_TO_REP103
    # Step 3: move the rover so that its plane of symmetry is y = 0.
    plane_y = symmetry_plane_y(source, xforms, to_new)
    to_new = to_new * translation(Gf.Vec3d(0.0, -plane_y, 0.0))
    print(
        f"Plane of symmetry at y = {plane_y * 1000:+.2f} mm from base_link: the rover moves by {-plane_y * 1000:+.2f} mm"
    )
    # Step 4: real size.
    to_new = to_new * Gf.Matrix4d(1.0).SetScale(SCALE)

    # Joint frames in the new asset frame, from body0 (the side of each joint that PhysX did not move at startup).
    source_frames = {}
    for prim in joint_prims(source):
        source_frames[prim.GetPath()] = rigid(joint_frame(source, xforms, UsdPhysics.Joint(prim), 0) * to_new)
    # Each rigid body that is body1 of a revolute joint gets its origin on that joint.
    origin_joint = {}
    for prim in joint_prims(source):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            origin_joint[UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()[0]] = prim.GetPath()
    frames, moves = symmetric_frames(source, source_frames, origin_joint)
    print(
        "Symmetry moves of the pivots and axles: "
        + ", ".join(f"{Sdf.Path(p).name} {np.linalg.norm(m) * 1000:.1f} mm" for p, m in moves.items())
    )

    base_link = Sdf.Path(f"{ROOT}/base_link")
    # Groups of rigid bodies (root, Mobility, rocker_bogie) get the root frame, so the rigid body transforms read
    # directly in REP-103.
    groups = {p.GetPath().GetParentPath() for p in rigid_body_prims(stage)}
    new_world = {}
    mesh_moves = {}
    mirrored = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
        if not prim.IsA(UsdGeom.Xformable):
            continue
        path = prim.GetPath()
        parent_new = new_world.get(path.GetParentPath(), Gf.Matrix4d(1.0))
        target = old_world[path] * to_new
        if path in groups:
            world = Gf.Matrix4d(1.0)
        elif path.GetParentPath() == base_link:
            # The IMU keeps its pose relative to base_link (it moved with base_link at runtime).
            world = (old_world[path] * old_world[base_link].GetInverse()) * parent_new
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Body and base_link (the REP-103 pose that PhysX gave it at runtime) at the root origin, the other rigid
            # bodies on their joint.
            world = (
                translation(frames[origin_joint[path]].ExtractTranslation())
                if path in origin_joint
                else Gf.Matrix4d(1.0)
            )
        elif prim.IsA(UsdGeom.Mesh):
            world = parent_new
            body = rigid_body_of(source.GetPrimAtPath(path)).GetPath().pathString
            if any(body.startswith(f"{ROOT}/{rel}") for rel in MESHES_FOLLOW_AXLE):
                mesh_moves[path] = moves[body]
                target = target * translation(Gf.Vec3d(*map(float, moves[body])))
            if bake_mesh(UsdGeom.Mesh(prim), target * parent_new.GetInverse()):
                mirrored.append(path.pathString)
        else:
            world = rigid(target)
        new_world[path] = world
        local = world * parent_new.GetInverse()
        set_local_transform(prim, None if prim.IsA(UsdGeom.Mesh) or local == Gf.Matrix4d(1.0) else local)
    print(f"Baked {len(mirrored)} mirrored meshes: {', '.join(Sdf.Path(p).name for p in mirrored)}")

    # Step 3: each left mesh becomes the mirrored right mesh (same faces, normals and texture coordinates).
    new_x = UsdGeom.XformCache()
    mirror_pairs = {}
    for left, right in SYMMETRIC_PAIRS.items():
        rights = subtree_meshes(stage, right)
        for left_mesh in subtree_meshes(stage, left):
            count = len(left_mesh.GetAttribute("points").Get())
            centroid = world_points(new_x, left_mesh).mean(0) * np.array([1.0, -1.0, 1.0])
            candidates = [m for m in rights if len(m.GetAttribute("points").Get()) == count]
            match = min(candidates, key=lambda m: np.linalg.norm(world_points(new_x, m).mean(0) - centroid))
            mirror_pairs[left_mesh.GetPath()] = match.GetPath()
    if len(set(mirror_pairs.values())) != len(mirror_pairs):
        sys.exit(f"Left meshes do not pair one to one with right meshes: {mirror_pairs}")
    layer = stage.GetRootLayer()
    for left_path, right_path in mirror_pairs.items():
        to_left = new_world[right_path.GetParentPath()] * MIRROR_Y * new_world[left_path.GetParentPath()].GetInverse()
        Sdf.CopySpec(layer, right_path, layer, left_path)
        bake_mesh(UsdGeom.Mesh(stage.GetPrimAtPath(left_path)), to_left)
    print(f"Replaced {len(mirror_pairs)} left meshes by their mirrored right mesh")

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
    return {
        "to_new": to_new,
        "frames": frames,
        "joint_moves": moves,
        "mesh_moves": mesh_moves,
        "mirror_pairs": mirror_pairs,
        "origin_joint": origin_joint,
    }


def bind_unbound_meshes(stage: Usd.Stage) -> None:
    """
    The rocker, bogie and motor meshes bound materials of the Blender scene (/World/chandrayaan_3_rover/Looks/silver_metal,
    silver_metal_002 and /scene/Materials/black), outside the referenced files, so USD ignored the bindings and they
    rendered without material. They get the materials of the asset with the same name (silver_metal_002 -> silver_metal).
    """
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{ROOT}/rocker_bogie")):
        if not prim.IsA(UsdGeom.Mesh) or UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]:
            continue
        name = UNBOUND_MESH_MATERIALS.get(prim.GetName(), UNBOUND_MESH_MATERIALS["*"])
        material = UsdShade.Material(stage.GetPrimAtPath(f"{ROOT}/Looks/{name}"))
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


# Checks of the result against the source.


def check(output: Usd.Stage, source: Usd.Stage, result: dict) -> None:
    to_new, frames = result["to_new"], result["frames"]
    mirror_pairs, mesh_moves = result["mirror_pairs"], result["mesh_moves"]
    failures = []
    new_x, old_x = UsdGeom.XformCache(), UsdGeom.XformCache()
    flip = np.array([1.0, -1.0, 1.0])

    def differ(moved, turned, max_move=POINT_TOLERANCE_M):
        return moved > max_move or min(turned, 360 - turned) > ANGLE_TOLERANCE_DEG

    worst = 0.0
    for prim in Usd.PrimRange(output.GetPrimAtPath(ROOT)):
        old = source.GetPrimAtPath(prim.GetPath())
        if prim.IsA(UsdGeom.Xformable):
            scale = Gf.Transform(new_x.GetLocalToWorldTransform(prim)).GetScale()
            if any(abs(s - 1.0) > 1e-6 for s in scale):
                failures.append(f"{prim.GetPath()}: world scale {tuple(scale)}")
        if prim.IsA(UsdGeom.Mesh) and prim.GetPath() not in mirror_pairs:
            expected = world_points(old_x, old, to_new) + mesh_moves.get(prim.GetPath(), np.zeros(3))
            worst = max(worst, float(np.abs(world_points(new_x, prim) - expected).max()))
        if prim.GetTypeName() == "Camera":
            new_pose = new_x.GetLocalToWorldTransform(prim)
            old_pose = rigid(old_x.GetLocalToWorldTransform(old) * to_new)
            moved = (new_pose.ExtractTranslation() - old_pose.ExtractTranslation()).GetLength()
            turned = (new_pose.ExtractRotation() * old_pose.ExtractRotation().GetInverse()).GetAngle()
            if differ(moved, turned):
                failures.append(f"{prim.GetPath()}: camera moved {moved:.2e} m, turned {turned:.2e} deg")
    if worst > POINT_TOLERANCE_M:
        failures.append(f"mesh points moved up to {worst:.2e} m")
    asymmetry, left_moves = 0.0, []
    for left, right in mirror_pairs.items():
        left_points = world_points(new_x, output.GetPrimAtPath(left))
        asymmetry = max(
            asymmetry, float(np.abs(left_points - world_points(new_x, output.GetPrimAtPath(right)) * flip).max())
        )
        old_centroid = world_points(old_x, source.GetPrimAtPath(left), to_new).mean(0)
        left_moves.append(float(np.linalg.norm(left_points.mean(0) - old_centroid)))
    if asymmetry > POINT_TOLERANCE_M:
        failures.append(f"left meshes differ from the mirrored right meshes by up to {asymmetry:.2e} m")
    if max(left_moves) > MAX_SYMMETRY_MOVE_M:
        failures.append(f"a left mesh centroid moved by {max(left_moves) * 1000:.1f} mm")
    for prim in joint_prims(output):
        joint = UsdPhysics.Joint(prim)
        frame0, frame1 = joint_frame(output, new_x, joint, 0), joint_frame(output, new_x, joint, 1)
        moved = (frame0.ExtractTranslation() - frame1.ExtractTranslation()).GetLength()
        turned = (frame0.ExtractRotation() * frame1.ExtractRotation().GetInverse()).GetAngle()
        if differ(moved, turned):
            failures.append(f"{prim.GetPath()}: joint frames differ by {moved:.2e} m and {turned:.2e} deg")
        if prim.GetPath() != Sdf.Path(FIXED_JOINT):
            expected = frames[prim.GetPath()]
            moved = (frame0.ExtractTranslation() - expected.ExtractTranslation()).GetLength()
            turned = (frame0.ExtractRotation() * expected.ExtractRotation().GetInverse()).GetAngle()
            if differ(moved, turned):
                failures.append(f"{prim.GetPath()}: joint moved {moved:.2e} m, turned {turned:.2e} deg in the world")
    origin_joint = result["origin_joint"]
    for left, right in SYMMETRIC_PAIRS.items():
        left_frame = joint_frame(
            output, new_x, UsdPhysics.Joint(output.GetPrimAtPath(origin_joint[Sdf.Path(f"{ROOT}/{left}")])), 0
        )
        right_frame = joint_frame(
            output, new_x, UsdPhysics.Joint(output.GetPrimAtPath(origin_joint[Sdf.Path(f"{ROOT}/{right}")])), 0
        )
        gap = np.abs(
            np.array(left_frame.ExtractTranslation()) - np.array(right_frame.ExtractTranslation()) * flip
        ).max()
        if gap > POINT_TOLERANCE_M:
            failures.append(f"{left} and {right}: joints not mirrored ({gap:.2e} m)")
    for rel, mass in MASSES_KG.items():
        if UsdPhysics.MassAPI(output.GetPrimAtPath(f"{ROOT}/{rel}")).GetMassAttr().Get() != np.float32(mass):
            failures.append(f"{rel}: mass differs from MASSES_KG")
    for side in ("left", "right"):
        for position, sign_x in (("front", 1), ("back", -1)):
            wheel = new_x.GetLocalToWorldTransform(output.GetPrimAtPath(f"{ROOT}/Mobility/Wheel_{position}_{side}"))
            x, y, _ = wheel.ExtractTranslation()
            if x * sign_x <= 0 or y * (1 if side == "left" else -1) <= 0:
                failures.append(f"Wheel_{position}_{side} at ({x:.2f}, {y:.2f}): not in REP-103 order")
    for prim in output.Traverse():
        if any(a.GetName().startswith("physxCookedData") for a in prim.GetAttributes()):
            failures.append(f"{prim.GetPath()}: cooked collision data left")
        old = source.GetPrimAtPath(prim.GetPath())
        if old and old.HasAPI(UsdPhysics.CollisionAPI) != prim.HasAPI(UsdPhysics.CollisionAPI):
            failures.append(f"{prim.GetPath()}: collision API differs from the source")
        if prim.IsA(UsdGeom.Mesh) and not UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]:
            failures.append(f"{prim.GetPath()}: no bound material")
    if output.GetRootLayer().externalReferences:
        failures.append(f"composition arcs to other files: {sorted(output.GetRootLayer().externalReferences)}")
    for prim in output.Traverse():
        texture = prim.GetAttribute("inputs:file")
        if texture and texture.HasAuthoredValue() and not texture.Get().resolvedPath:
            failures.append(f"{prim.GetPath()}: texture {texture.Get().path} does not resolve")
    print(f"Unpaired meshes match the source within {worst:.1e} m")
    print(
        f"Left meshes match the mirrored right meshes within {asymmetry:.1e} m, centroids moved up to "
        f"{max(left_moves) * 1000:.1f} mm"
    )
    if failures:
        sys.exit("Checks failed:\n  " + "\n  ".join(failures))
    print("Checks passed: unit scales, meshes, symmetry, cameras, joints, masses, wheel layout, materials, textures")


def main() -> None:
    source = Usd.Stage.Open(str(SOURCE))
    layer = source.Flatten()
    stage = Usd.Stage.Open(layer)
    for path in UNUSED_ROOT_PRIMS:
        stage.RemovePrim(path)
    apply_masses(stage)
    result = clean_frames(source, stage)
    bind_unbound_meshes(stage)
    apply_material_binding_api(stage)
    print(f"Removed the cooked collision data of {remove_cooked_data(stage)} meshes")
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    layer.Export(str(OUTPUT))

    output = Usd.Stage.Open(str(OUTPUT))
    total = 0.0
    for prim in rigid_body_prims(output):
        mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        total += mass
        print(f"  {prim.GetPath().pathString:40s} {mass:6.2f} kg")
    print(f"Wrote {OUTPUT.relative_to(REPO_ROOT)}: total mass {total:.2f} kg")
    check(output, source, result)


if __name__ == "__main__":
    main()
