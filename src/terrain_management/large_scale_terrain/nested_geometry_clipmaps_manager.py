__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import Tuple
import dataclasses
import numpy as np
import warnings
import math

from semantics.schema.editor import PrimSemanticData
import omni

from src.terrain_management.large_scale_terrain.geometry_clipmaps_manager import (
    GeoClipmapManagerConf,
    GeoClipmapManager,
)
from src.terrain_management.large_scale_terrain.geometry_clipmaps import GeometryClipmapConf
from src.terrain_management.large_scale_terrain.pxr_utils import bind_material, load_material


@dataclasses.dataclass
class NestedGeometryClipmapManagerConf:
    """
    Args:
        num_texels_per_level (int): The number of texels per level.
        target_res (float): The target resolution. (in meters)
        fine_interpolation_method (str): The interpolation method for the fine clipmap. (bicubic or bilinear)
        coarse_interpolation_method (str): The interpolation method for the coarse clipmap. (bicubic or bilinear)
        fine_acceleration_mode (str): The acceleration mode for the fine clipmap. (hybrid or gpu)
        coarse_acceleration_mode (str): The acceleration mode for the coarse clipmap. (hybrid or gpu)
        profiling (bool): Whether to profile the clipmap manager.
        semantic_label (str): The semantic label of the terrain. (None if no label is to be used)
        texture_name (str): The name of the texture. (None if no texture is to be used)
        texture_path (str): The path to the texture. (None if no texture is to be used)
    """

    num_texels_per_level: int = dataclasses.field(default_factory=int)
    target_res: float = dataclasses.field(default_factory=float)
    fine_interpolation_method: str = dataclasses.field(default_factory=str)
    coarse_interpolation_method: str = dataclasses.field(default_factory=str)
    fine_acceleration_mode: str = dataclasses.field(default_factory=str)
    coarse_acceleration_mode: str = dataclasses.field(default_factory=str)
    profiling: bool = dataclasses.field(default_factory=bool)
    semantic_label: str = dataclasses.field(default_factory=str)
    texture_name: str = dataclasses.field(default_factory=str)
    texture_path: str = dataclasses.field(default_factory=str)

    def __post_init__(self) -> None:
        """
        Post initialization checks.
        """

        assert self.fine_interpolation_method in ["bicubic", "bilinear"]
        assert self.coarse_interpolation_method in ["bicubic", "bilinear"]
        assert self.fine_acceleration_mode in ["hybrid", "gpu"]
        assert self.coarse_acceleration_mode in ["hybrid", "gpu"]

        if self.semantic_label == "":
            self.semantic_label = None
        if self.texture_name == "":
            self.texture_name = None
        if self.texture_path == "":
            self.texture_path = None


class NestedGeometryClipmapManager:
    """
    The nested geometry clipmap manager.
    It manages the fine and coarse clipmaps and makes sure that they are updated correctly.
    """

    def __init__(self, settings: NestedGeometryClipmapManagerConf) -> None:
        """
        Args:
            settings (NestedGeometryClipMapManagerConf): The settings for the nested geometry clipmap manager.
        """

        self.settings = settings
        self.stage = omni.usd.get_context().get_stage()

    def generate_geometry_clip_maps_configs(
        self,
        hr_dem_shape: Tuple[float, float],
        lr_dem_shape: Tuple[float, float],
        hr_dem_res: float,
        lr_dem_res: float,
    ) -> None:
        """
        Generate the geometry clipmap configurations using the shape and resolution of the DEMs.
        This code will ensure that the clipmaps are using the full extent of the DEMs.

        Args:
            hr_dem_shape (tuple): The shape of the high resolution DEM. (pixels)
            lr_dem_shape (tuple): The shape of the low resolution DEM. (pixels)
            hr_dem_res (float): The resolution of the high resolution DEM. (meters per pixel)
            lr_dem_res (float): The resolution of the low resolution DEM. (meters per pixel)
        """

        hr_dem_width = max(hr_dem_shape[0], hr_dem_shape[1]) * hr_dem_res
        lr_dem_width = max(lr_dem_shape[0], lr_dem_shape[1]) * lr_dem_res

        fine_clipmap_levels = int(
            math.log(hr_dem_width / (self.settings.num_texels_per_level * self.settings.target_res)) / math.log(2)
        )
        coarse_clipmap_res = self.settings.target_res * 2 ** (fine_clipmap_levels - 1)
        coarse_clipmap_levels = int(
            math.log(lr_dem_width / (self.settings.num_texels_per_level * coarse_clipmap_res)) / math.log(2)
        )

        self.fine_clipmap_specs = GeometryClipmapConf(
            startingLODLevel=0,
            numMeshLODLevels=fine_clipmap_levels,
            meshBaseLODExtentHeightfieldTexels=self.settings.num_texels_per_level,
            meshBackBonePath="mesh_backbone_fine.npz",
            source_resolution=hr_dem_res,
            minimum_target_resolution=self.settings.target_res,
        )
        self.coarse_clipmap_specs = GeometryClipmapConf(
            startingLODLevel=1,
            numMeshLODLevels=coarse_clipmap_levels + 1,
            meshBaseLODExtentHeightfieldTexels=self.settings.num_texels_per_level,
            meshBackBonePath="mesh_backbone_coarse.npz",
            source_resolution=lr_dem_res,
            minimum_target_resolution=coarse_clipmap_res,
        )
        self.fine_clipmap_manager_cfg = GeoClipmapManagerConf(
            root_path="/World",
            geo_clipmap_specs=self.fine_clipmap_specs,
            mesh_position=np.array([0.0, 0.0, 0.0]),
            mesh_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            mesh_scale=np.array([1.0, 1.0, 1.0]),
        )
        self.coarse_clipmap_manager_cfg = GeoClipmapManagerConf(
            root_path="/World",
            geo_clipmap_specs=self.coarse_clipmap_specs,
            mesh_position=np.array([0.0, 0.0, 0.0]),
            mesh_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            mesh_scale=np.array([1.0, 1.0, 1.0]),
        )

    def build(
        self,
        hr_dem: np.ndarray,
        lr_dem: np.ndarray,
        hr_dem_shape: Tuple[float, float],
        lr_dem_shape: Tuple[float, float],
        hr_dem_res: float,
        lr_dem_res: float,
        hr_dem_center: Tuple[float, float],
        lr_dem_center: Tuple[float, float],
    ) -> None:
        """
        Build the nested geometry clipmap manager.

        Args:
            hr_dem (np.ndarray): The high resolution DEM.
            lr_dem (np.ndarray): The low resolution DEM.
            hr_dem_shape (tuple): The shape of the high resolution DEM. (pixels)
            lr_dem_shape (tuple): The shape of the low resolution DEM. (pixels)
            hr_dem_res (float): The resolution of the high resolution DEM. (meters per pixel)
            lr_dem_res (float): The resolution of the low resolution DEM. (meters per pixel)
            hr_dem_center (tuple): The center of the high resolution DEM. (meters) Note that this is not necessarily the
                true center of the map.
            lr_dem_center (tuple): The center of the low resolution DEM. (meters) Note that this is not necessarily the
                true center of the map.
        """

        self.generate_geometry_clip_maps_configs(hr_dem_shape, lr_dem_shape, hr_dem_res, lr_dem_res)

        self.fine_clipmap_manager = GeoClipmapManager(
            self.fine_clipmap_manager_cfg,
            interpolation_method=self.settings.fine_interpolation_method,
            acceleration_mode=self.settings.fine_acceleration_mode,
            name_prefix="_fine",
            profiling=self.settings.profiling,
            stage=self.stage,
        )
        self.coarse_clipmap_manager = GeoClipmapManager(
            self.coarse_clipmap_manager_cfg,
            interpolation_method=self.settings.coarse_interpolation_method,
            acceleration_mode=self.settings.coarse_acceleration_mode,
            name_prefix="_coarse",
            profiling=self.settings.profiling,
            stage=self.stage,
        )
        self.fine_clipmap_manager.build(hr_dem, hr_dem_shape, dem_center=hr_dem_center)
        self.coarse_clipmap_manager.build(lr_dem, lr_dem_shape, dem_center=lr_dem_center)
        self.load_and_apply_material()
        self.add_semantic_label()

    def update_clipmaps(
        self,
        position_fine: Tuple[float, float],
        position_coarse: Tuple[float, float],
        mesh_position: Tuple[float, float],
    ) -> None:
        """
        Update the clipmaps geometry based on the given positions.

        Args:
            position_fine (Tuple[float, float]): The coordinates of the robot in the fine clipmap frame. (meters)
            position_coarse (Tuple[float, float]): The coordinates of the robot in the coarse clipmap frame. (meters)
            mesh_position (Tuple[float, float]): The position of the geometry clipmaps' meshes in the world frame. (meters)
        """

        position_fine = np.array(position_fine)
        position_coarse = np.array(position_coarse)
        mesh_position = np.array([mesh_position[0], mesh_position[1], 0])
        self.fine_clipmap_manager.update_geoclipmap(position_fine, mesh_position)
        self.coarse_clipmap_manager.update_geoclipmap(position_coarse, mesh_position)

    def get_height_and_random_scale(
        self,
        x: np.ndarray,
        y: np.ndarray,
        map_coordinates: Tuple[float, float],
        seed: int = 0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the height and random scale at the given coordinates.

        Args:
            x (np.ndarray): The x coordinates.
            y (np.ndarray): The y coordinates.
            map_coordinates (Tuple[float, float]): The coordinates in the map.
            seed (int): The seed for the random scale.
        """

        return self.fine_clipmap_manager.get_height_and_random_orientation(x, y, map_coordinates, seed=seed)

    def load_and_apply_material(self):
        """
        Load and apply the material to the clipmaps.

        Warnings:
            - If the texture name is not provided, a warning is raised.
            - If the texture path is not provided, a warning is raised.
        """

        if (self.settings.texture_name is not None) and (self.settings.texture_path is not None):
            material_path = load_material(self.settings.texture_name, self.settings.texture_path)
            bind_material(self.fine_clipmap_manager._stage, material_path, self.fine_clipmap_manager._mesh_path)
            bind_material(self.coarse_clipmap_manager._stage, material_path, self.coarse_clipmap_manager._mesh_path)
        elif (self.settings.texture_name is not None) and (self.settings.texture_path is None):
            warnings.warn("No texture path provided. Material will not be applied.")
        elif (self.settings.texture_name is None) and (self.settings.texture_path is not None):
            warnings.warn("No texture name provided. Material will not be applied.")

    def add_semantic_label(self):
        """
        Add the semantic label to the clipmaps.
        """
        if self.settings.semantic_label is not None:
            prim_sd = PrimSemanticData(self.stage.GetPrimAtPath(self.fine_clipmap_manager._mesh_path))
            prim_sd.add_entry("class", self.settings.semantic_label)
            prim_sd = PrimSemanticData(self.stage.GetPrimAtPath(self.coarse_clipmap_manager._mesh_path))
            prim_sd.add_entry("class", self.settings.semantic_label)
