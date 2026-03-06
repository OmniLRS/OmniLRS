__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import Tuple
import numpy as np
import dataclasses
import os

from pxr import UsdGeom, Gf, Usd

from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops, add_collider, remove_collider
from src.terrain_management.large_scale_terrain.utils import ScopedTimer


@dataclasses.dataclass
class ColliderBuilderConf:
    """
    Args:
        resolution (float): resolution of the collider (meter per pixel/texel).
        block_size (int): size of the block (meters).
        collider_path (str): path to the collider.
        collider_mode (str): mode of the collider. Should use meshSimplification.
        visible (bool): visibility of the collider. True means visible.
        profiling (bool): enable profiling.
    """

    resolution: float = dataclasses.field(default_factory=float)
    block_size: int = dataclasses.field(default_factory=int)
    collider_path: str = dataclasses.field(default_factory=str)
    collider_mode: str = dataclasses.field(default_factory=str)
    visible: bool = dataclasses.field(default_factory=bool)
    profiling: bool = dataclasses.field(default_factory=bool)


class ColliderBuilder:
    """
    Class to build a collider mesh for the terrain.
    """

    def __init__(self, settings: ColliderBuilderConf, stage: Usd.Stage) -> None:
        """
        Args:
            settings (ColliderBuilderConf): settings of the collider.
            stage (Usd.Stage): stage to create the collider
        """

        self.settings = settings
        self.stage = stage
        self.indices = []
        self.count = 0

    @staticmethod
    def grid_index(x: int, y: int, stride: int) -> int:
        """
        Returns the index of the grid point at (x, y) in a grid of width stride.

        Args:
            x (int): x coordinate of the grid point.
            y (int): y coordinate of the grid point.
            stride (int): width of the grid.

        Returns:
            int: index of the grid point at (x, y).
        """

        return y * stride + x

    def build_base_grid(self) -> None:
        """
        Builds the grid of vertices and indices for the terrain mesh.
        Also generates the UVs coordinates for the terrain mesh.
        """

        with ScopedTimer("build_base_grid", active=self.settings.profiling):
            block_size_pixels = int(self.settings.block_size / self.settings.resolution) + 1

            vertices = []
            for y in range(
                0,
                block_size_pixels,
            ):
                for x in range(
                    0,
                    block_size_pixels,
                ):
                    pos = (
                        float(x) * self.settings.resolution,
                        float(y) * self.settings.resolution,
                        float(0),
                    )

                    # Generates vertices indices for the terrain mesh.
                    vertices.append(pos)
                    if x > 0 and y > 0:  # Two triangles per pixel.
                        # First triangle.
                        self.indices.append(self.grid_index(x, y - 1, block_size_pixels))
                        self.indices.append(self.grid_index(x, y, block_size_pixels))
                        self.indices.append(self.grid_index(x - 1, y - 1, block_size_pixels))
                        # Second triangle.
                        self.indices.append(self.grid_index(x, y, block_size_pixels))
                        self.indices.append(self.grid_index(x - 1, y, block_size_pixels))
                        self.indices.append(self.grid_index(x - 1, y - 1, block_size_pixels))

            self.sim_verts = np.array(vertices, dtype=np.float32)
            self.indices = np.array(self.indices).reshape(-1, 3)
            self.num_indices = [3] * len(self.indices)

    def create_collider(self, position: Tuple[float, float], map: np.ndarray, name: str) -> str:
        """
        Creates a mesh with a collider at the given position.
        It uses the data inside map to assign the mesh vertices heights.

        Args:
            position (Tuple[float, float]): position of the mesh.
            map (np.ndarray): map to use for the mesh vertices heights.
            name (str): name of the mesh.

        Returns:
            str: path to the mesh.
        """

        with ScopedTimer("create_collider_mesh", active=self.settings.profiling):
            self.sim_verts[:, -1] = map.flatten()
            mesh_path = os.path.join(self.settings.collider_path, name)

            mesh = UsdGeom.Mesh.Define(self.stage, mesh_path)
            mesh.GetPointsAttr().Set(self.sim_verts)
            mesh.GetFaceVertexIndicesAttr().Set(self.indices)
            mesh.GetFaceVertexCountsAttr().Set(self.num_indices)
            if not self.settings.visible:
                mesh.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

            set_xform_ops(mesh, translate=Gf.Vec3d(position[0], position[1], 0))
            add_collider(self.stage, mesh.GetPath(), mode=self.settings.collider_mode)
        return mesh_path

    def remove_collider(self, path: str) -> None:
        """
        Removes the mesh at the given path.

        Args:
            path (str): path to the mesh to remove.
        """

        with ScopedTimer("remove_collider", active=self.settings.profiling):
            self.stage.RemovePrim(path)
