"""USD helpers to rebuild an asset with clean transforms, shared by the rebuild scripts of this folder.

The rebuild scripts flatten a source asset, give each Xform and camera a transform without scale, and bake each mesh
transform (scale, mirror) into the mesh points, normals and face order. USD matrices act on row vectors (p' = p * M).
"""

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt


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


def world_points(xforms: UsdGeom.XformCache, prim: Usd.Prim, extra: Gf.Matrix4d | None = None) -> np.ndarray:
    """Points of a mesh in the world (then times `extra`), as an (N, 3) array."""
    matrix = xforms.GetLocalToWorldTransform(prim)
    matrix = np.array(matrix if extra is None else matrix * extra)
    return np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=np.float64) @ matrix[:3, :3] + matrix[3, :3]


def remove_cooked_data(stage: Usd.Stage) -> int:
    """Removes the cached cooked collision data (PhysxCookedDataAPI). Returns the number of meshes cleaned."""
    count = 0
    for prim in stage.Traverse():
        names = [a.GetName() for a in prim.GetAttributes() if a.GetName().startswith("physxCookedData")]
        if not names:
            continue
        for name in names:
            prim.RemoveProperty(name)
        schemas = prim.GetMetadata("apiSchemas")

        def keep(items):
            return [t for t in items if not t.startswith("PhysxCookedDataAPI")]

        if schemas.isExplicit:
            schemas = Sdf.TokenListOp.CreateExplicit(keep(schemas.explicitItems))
        else:
            schemas = Sdf.TokenListOp.Create(
                prependedItems=keep(schemas.prependedItems),
                appendedItems=keep(schemas.appendedItems),
                deletedItems=keep(schemas.deletedItems),
            )
        prim.SetMetadata("apiSchemas", schemas)
        count += 1
    return count


def apply_material_binding_api(stage: Usd.Stage) -> None:
    for prim in stage.Traverse():
        has_binding = any(r.GetName().startswith("material:binding") for r in prim.GetRelationships())
        if has_binding and not prim.HasAPI(UsdShade.MaterialBindingAPI):
            UsdShade.MaterialBindingAPI.Apply(prim)
