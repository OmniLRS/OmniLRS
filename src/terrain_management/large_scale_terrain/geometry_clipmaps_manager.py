__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.terrain_management.large_scale_terrain.geometry_clipmaps import (
    GeometryClipmapConf,
    GeoClipmap,
)
from dataclasses import dataclass, field
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
from pxr import UsdGeom, Sdf, Usd, Gf
from typing import Tuple
import numpy as np
import warp as wp
import omni


@dataclass
class GeoClipmapManagerConf:
    """
    Args:
        root_path (str): path to the root of the clipmap.
        geo_clipmap_specs (GeometryClipmapConf): specifications for the clipmap.
        mesh_position (tuple): position of the mesh.
        mesh_orientation (tuple): orientation of the mesh.
        mesh_scale (tuple): scale of the mesh.
    """

    root_path: str = "/World"
    geo_clipmap_specs: GeometryClipmapConf = field(default_factory=dict)
    mesh_position: tuple = (0.0, 0.0, 0.0)
    mesh_orientation: tuple = (0.0, 0.0, 0.0, 1.0)
    mesh_scale: tuple = (1.0, 1.0, 1.0)


class GeoClipmapManager:
    """
    Class to manage the geometry clipmap.
    """

    def __init__(
        self,
        cfg: GeoClipmapManagerConf,
        interpolation_method: str = "bilinear",
        acceleration_mode: str = "hybrid",
        name_prefix: str = "",
        profiling: bool = False,
        stage: Usd.Stage = None,
    ) -> None:
        """
        Args:
            cfg (GeoClipmapManagerConf): configuration for the clipmap.
            interpolation_method (str): method to use for interpolation.
            acceleration_mode (str): mode to use for acceleration.
            name_prefix (str): prefix to add to the mesh name.
        """

        self._geo_clipmap = GeoClipmap(
            cfg.geo_clipmap_specs,
            interpolation_method=interpolation_method,
            acceleration_mode=acceleration_mode,
            profiling=profiling,
        )
        self._root_path = cfg.root_path
        self.profiling = profiling
        self._stage = stage

        self._mesh_pos = cfg.mesh_position
        self._mesh_rot = cfg.mesh_orientation
        self._mesh_scale = cfg.mesh_scale

        self._og_mesh_path = self._root_path + "/Terrain/terrain_mesh" + name_prefix
        self._mesh_path = self._root_path + "/Terrain/terrain_mesh" + name_prefix

        self.create_Xforms()
        self.update_topology = True

    def build(self, dem: np.ndarray, dem_shape: Tuple[int, int], dem_center: Tuple[float, float] = None) -> None:
        """
        Builds the clipmap with the given dem.

        Args:
            dem (np.ndarray): dem to use for the clipmap.
            dem_shape (Tuple[int, int]): shape of the dem.
            dem_center (Tuple[float, float]): center of the dem in meters. Note that this is not necessarily the center
                of the true center but rather where the [0,0], or the top left of the center block is.
        """

        self._geo_clipmap.build(dem, dem_shape, dem_center=dem_center)

    def update_DEM_buffer(self) -> None:
        """
        Updates the DEM buffer of the clipmap.
        """

        self._geo_clipmap.update_DEM_buffer()

    def update_geoclipmap(self, position: np.ndarray, mesh_position: np.ndarray) -> None:
        """
        Updates the clipmap with the given position and mesh position.

        Args:
            position (np.ndarray): position to use for the update (in meters).
            mesh_position (np.ndarray): position of the mesh (in meters).
        """
        with wp.ScopedTimer("complete update loop", active=self.profiling):
            self._geo_clipmap.update_elevation(position)
            with wp.ScopedTimer("mesh update", active=self.profiling):
                self.render_mesh(
                    self._geo_clipmap.points,
                    self._geo_clipmap.indices,
                    self._geo_clipmap.uvs,
                    update_topology=self.update_topology,
                )
        self.move_mesh(mesh_position)
        self.update_topology = False

    def move_mesh(self, position: np.ndarray) -> None:
        """
        Moves the mesh to the given position.

        Args:
            position (np.ndarray): position to move the mesh to (in meters).
        """

        self._mesh_pos = position
        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        if mesh:
            set_xform_ops(mesh, Gf.Vec3d(self._mesh_pos[0], self._mesh_pos[1], self._mesh_pos[2]))

    def create_Xforms(self) -> None:
        """
        Creates the xforms for the clipmap.
        """

        if not self._stage.GetPrimAtPath(self._root_path):
            self._stage.DefinePrim(self._root_path, "Xform")
        if not self._stage.GetPrimAtPath(self._root_path + "/Terrain"):
            self._stage.DefinePrim(self._root_path + "/Terrain", "Xform")
        self._stage.DefinePrim(self._mesh_path)

    def render_mesh(
        self,
        points: np.ndarray,
        indices: np.ndarray,
        uvs: np.ndarray,
        update_topology: bool = False,
    ) -> None:
        """
        Creates or updates a mesh prim with the given points and indices.

        Args:
            points (np.ndarray): array of points to set as the mesh vertices.
            indices (np.ndarray): array of indices to set as the mesh indices.
            uvs (np.ndarray): array of uvs to set as the mesh uvs.
            colors (np.ndarray): array of colors to set as the mesh colors.
            update_topology (bool): whether to update the mesh topology.
        """

        mesh = UsdGeom.Mesh.Get(self._stage, self._mesh_path)
        self._mesh_path = self._og_mesh_path
        if not mesh:
            mesh = UsdGeom.Mesh.Define(self._stage, self._mesh_path)
            set_xform_ops(mesh, Gf.Vec3d(0.0, 0.0, 0.0), Gf.Quatd(1.0, (0.0, 0.0, 0.0)), Gf.Vec3d(1.0, 1.0, 1.0))

            # force topology update on first update
            update_topology = True

        mesh.GetPointsAttr().Set(points)

        if update_topology:
            idxs = np.array(indices).reshape(-1, 3)
            mesh.GetFaceVertexIndicesAttr().Set(idxs)
            mesh.GetFaceVertexCountsAttr().Set([3] * len(idxs))
            pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.Float2Array)
            pv.Set(uvs)
            pv.SetInterpolation("faceVarying")

        set_xform_ops(
            mesh,
            Gf.Vec3d(self._mesh_pos[0], self._mesh_pos[1], self._mesh_pos[2]),
            Gf.Quatd(self._mesh_rot[-1], (self._mesh_rot[0], self._mesh_rot[1], self._mesh_rot[2])),
            Gf.Vec3d(self._mesh_scale[0], self._mesh_scale[1], self._mesh_scale[2]),
        )

    def get_height_and_random_orientation(
        self,
        x: np.ndarray,
        y: np.ndarray,
        map_coordinates: Tuple[float, float],
        seed: int = 0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Gets the height and random orientation at the given x and y.

        Args:
            x (np.ndarray): x value.
            y (np.ndarray): y value.
            map_coordinates (Tuple[float, float]): coordinates of the map.
            seed (int): seed to use for the random orientation.

        Returns:
            Tuple[np.ndarray, np.ndarray]: height and random orientation.
        """

        return self._geo_clipmap.get_height_and_random_orientation(x, y, map_coordinates, seed=seed)
