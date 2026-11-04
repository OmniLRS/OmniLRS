"""Static terrain boundary colliders, authored with USD without requiring a running simulator."""

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from src.configurations.procedural_terrain_confs import TerrainManagerConf

WALL_THICKNESS = 0.5
WALL_INSET = 0.1
WALL_DEPTH_BELOW_TERRAIN = 1.0
WALL_HEIGHT_ABOVE_TERRAIN = 2.0


def update_boundary_walls(
    stage: Usd.Stage,
    terrain_settings: TerrainManagerConf,
    dem: np.ndarray,
    *,
    enabled: bool = True,
    visible: bool = False,
) -> None:
    """Create or resize the same four walls after terrain loading (meters before mesh scaling)."""
    root_path = terrain_settings.root_path.rstrip("/") + "/BoundaryWalls"
    if not enabled:
        if stage.GetPrimAtPath(root_path):
            stage.RemovePrim(root_path)
        return

    # Match TerrainManager.buildGrid: the final sample is one resolution step short of the configured size.
    resolution = terrain_settings.resolution
    width = (int(terrain_settings.sim_width / resolution) - 1) * resolution
    length = (int(terrain_settings.sim_length / resolution) - 1) * resolution
    if min(width, length) <= 2 * WALL_INSET:
        raise ValueError("Terrain must span more than twice the wall inset on both axes to fit the boundary walls.")

    bottom = float(dem.min()) - WALL_DEPTH_BELOW_TERRAIN
    top = float(dem.max()) + WALL_HEIGHT_ABOVE_TERRAIN
    height = top - bottom
    center_z = (top + bottom) / 2
    # Only the inset lies inside the terrain; the remaining wall thickness extends outside.
    edge_offset = WALL_INSET - WALL_THICKNESS / 2

    group = UsdGeom.Xform.Define(stage, root_path)
    transform = Gf.Transform()
    transform.SetTranslation(Gf.Vec3d(*terrain_settings.mesh_position))
    qx, qy, qz, qw = terrain_settings.mesh_orientation
    transform.SetRotation(Gf.Rotation(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz))))
    transform.SetScale(Gf.Vec3d(*terrain_settings.mesh_scale))
    group.MakeMatrixXform().Set(transform.GetMatrix())

    # Full edge lengths make adjacent walls overlap at every corner.
    walls = (
        ("West", (edge_offset, length / 2, center_z), (WALL_THICKNESS, length, height)),
        ("East", (width - edge_offset, length / 2, center_z), (WALL_THICKNESS, length, height)),
        ("South", (width / 2, edge_offset, center_z), (width, WALL_THICKNESS, height)),
        ("North", (width / 2, length - edge_offset, center_z), (width, WALL_THICKNESS, height)),
    )
    for name, position, dimensions in walls:
        wall = UsdGeom.Cube.Define(stage, root_path + "/" + name)
        wall.CreateSizeAttr(1.0)
        wall.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        matrix = Gf.Matrix4d().SetScale(Gf.Vec3d(*dimensions))
        matrix.SetTranslateOnly(Gf.Vec3d(*position))
        wall.MakeMatrixXform().Set(matrix)
        wall.CreateVisibilityAttr(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)
        # A collider with no RigidBodyAPI remains static even when hidden from rendering.
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim()).CreateCollisionEnabledAttr(True)
