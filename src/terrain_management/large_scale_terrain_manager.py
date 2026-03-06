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
import math
import copy

from src.configurations.environments import LargeScaleTerrainConf

from src.terrain_management.large_scale_terrain.nested_geometry_clipmaps_manager import (
    NestedGeometryClipmapManagerConf,
    NestedGeometryClipmapManager,
)
from src.terrain_management.large_scale_terrain.collider_manager import (
    ColliderManagerConf,
    ColliderManager,
)
from src.terrain_management.large_scale_terrain.rock_manager import (
    RockManagerConf,
    RockManager,
)
from src.terrain_management.large_scale_terrain.map_manager import (
    MapManagerConf,
    MapManager,
)


def is_map_done():
    return True


class LargeScaleTerrainManager:
    def __init__(
        self,
        settings: LargeScaleTerrainConf,
        is_simulation_alive: callable = lambda: True,
        close_simulation: callable = lambda: None,
    ):
        self.settings = settings
        self.is_simulation_alive = is_simulation_alive
        self.close_simulation = close_simulation
        self.last_update_coordinates = None

    def build_configs(self):
        self.nested_geometric_clipmap_manager_cfg = NestedGeometryClipmapManagerConf(**self.settings.NGCMMConf_D)
        self.mapmanager_cfg = MapManagerConf(**self.settings.MMConf_D)
        self.rock_manager_cfg = RockManagerConf(**self.settings.RMConf_D)
        self.collider_manager_cfg = ColliderManagerConf(**self.settings.CMConf_D)

    def build_managers(self):
        self.build_map_manager()
        self.build_geometry_clipmap_manager()
        self.build_collider_manager()
        self.build_rock_manager()

    def build_map_manager(self):
        self.map_manager = MapManager(
            self.mapmanager_cfg, is_simulation_alive=self.is_simulation_alive, close_simulation=self.close_simulation
        )
        self.map_manager.load_lr_dem_by_name(self.settings.lr_dem_name)
        self.map_manager.initialize_hr_dem(self.settings.starting_position)

    def build_collider_manager(self):
        self.collider_manager = ColliderManager(
            self.collider_manager_cfg,
            self.map_manager.get_hr_dem(),
            self.map_manager.get_hr_dem_shape(),
            self.map_manager.get_hr_dem_center_top_left(),
            self.map_manager.get_hr_dem_res(),
        )
        self.collider_manager.build()

    def build_geometry_clipmap_manager(self):
        self.nested_clipmap_manager = NestedGeometryClipmapManager(self.nested_geometric_clipmap_manager_cfg)
        self.nested_clipmap_manager.build(
            self.map_manager.get_hr_dem(),
            self.map_manager.get_lr_dem(),
            self.map_manager.get_hr_dem_shape(),
            self.map_manager.get_lr_dem_shape(),
            self.map_manager.get_hr_dem_res(),
            self.map_manager.get_lr_dem_res(),
            self.map_manager.get_hr_dem_center_top_left(),
            self.map_manager.get_lr_dem_center_top_left(),
        )

    def build_rock_manager(self):
        self.rock_manager = RockManager(
            self.rock_manager_cfg,
            self.nested_clipmap_manager.get_height_and_random_scale,
            is_map_done,
        )
        self.rock_manager.build()

    def get_height_local(self, coordinates: Tuple[float, float]) -> float:
        """
        Get the height at the given coordinates in the local map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the local map.

        Returns:
            float: The height at the given coordinates.
        """

        global_coordinates = (
            coordinates[0] + self.settings.starting_position[0],
            coordinates[1] + self.settings.starting_position[1],
        )
        return self.map_manager.get_height(global_coordinates)

    def get_height_global(self, coordinates: Tuple[float, float]) -> float:
        """
        Get the height at the given coordinates in the global map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the global map.

        Returns:
            float: The height at the given coordinates.
        """

        return self.map_manager.get_height(coordinates)

    def get_normal_local(self, coordinates: Tuple[float, float]) -> np.ndarray:
        """
        Get the normal at the given coordinates in the local map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the local map.

        Returns:
            np.ndarray: The normal to the DEM surface at the given coordinates.
        """

        global_coordinates = (
            coordinates[0] + self.settings.starting_position[0],
            coordinates[1] + self.settings.starting_position[1],
        )
        return self.map_manager.get_normal(global_coordinates)

    def get_normal_global(self, coordinates: Tuple[float, float]) -> np.ndarray:
        """
        Get the normal at the given coordinates in the global map.

        Args:
            coordinates (Tuple[float, float]): The coordinates in the global map.

        Returns:
            np.ndarray: The normal to the DEM surface at the given coordinates.
        """

        return self.map_manager.get_normal(coordinates)

    def get_lat_lon(self) -> Tuple[float, float]:
        """
        Get the latitude and longitude of the center of the map.

        Returns:
            Tuple[float, float]: The latitude and longitude of the center of the map.
        """

        return self.map_manager.get_lat_lon()

    def build(self):
        self.build_configs()
        self.build_managers()
        self.mesh_position = (0, 0)
        self.update_visual_mesh((0, 0))

    def update_visual_mesh(self, local_coordinates):
        # print("local_coordinates", local_coordinates)
        # print("last_update_coordinates", self.last_update_coordinates)
        if self.last_update_coordinates is None:
            self.last_update_coordinates = copy.copy(local_coordinates)
            delta = (0.0, 0.0)
            dist = self.settings.update_every_n_meters * 2
        else:
            # local coordinates are in meters
            delta = (
                local_coordinates[0] - self.last_update_coordinates[0],
                local_coordinates[1] - self.last_update_coordinates[1],
            )
            dist = math.sqrt(delta[0] ** 2 + delta[1] ** 2)
        if dist > self.settings.update_every_n_meters:
            # cast the coordinates so that they are a multiple of the threshold.
            x = (local_coordinates[0] // self.settings.update_every_n_meters) * self.settings.update_every_n_meters
            y = (local_coordinates[1] // self.settings.update_every_n_meters) * self.settings.update_every_n_meters
            corrected_coordinates = (x, y)
            # Update the visual mesh
            self.last_update_coordinates = local_coordinates

            # Get the global coordinates. The mesh initial position is in (0,0).
            # Thus, we need to add the initial coordinates to the local coordinates.
            global_corrected_coordinates = (
                corrected_coordinates[0] + self.settings.starting_position[0],
                corrected_coordinates[1] + self.settings.starting_position[1],
            )
            global_coordinates = (
                local_coordinates[0] + self.settings.starting_position[0],
                local_coordinates[1] + self.settings.starting_position[1],
            )
            # print("initial_coordinates", self.initial_coordinates)
            # print("global_coordinates", global_coordinates)

            # Update the high resolution DEM
            # hr_dem_updated = self.map_manager.update_hr_dem(global_coordinates)
            self.map_manager.hr_dem_gen.update_terrain_data_blocking(global_corrected_coordinates)
            # if the DEM was updated, the high DEM inside the warp buffer of the nested clipmap manager
            # needs to be updated as well.

            # self.mesh_position = (
            #    self.mesh_position[0] + delta[0],
            #    self.mesh_position[1] + delta[1],
            # )
            print("terrain updated")
            fine_position = self.map_manager.get_hr_coordinates(global_corrected_coordinates)
            coarse_position = self.map_manager.get_lr_coordinates(global_corrected_coordinates)
            print("coordinates aquired")
            # print("coarse_position", coarse_position)
            # print("min value lr dem", self.map_manager.get_lr_dem().min())
            # print("max value lr dem", self.map_manager.get_lr_dem().max())
            # print("min value hr dem", self.map_manager.get_hr_dem().min())
            # print("max value hr dem", self.map_manager.get_hr_dem().max())

            self.nested_clipmap_manager.update_clipmaps(fine_position, coarse_position, corrected_coordinates)
            print("clipmaps updated")
            self.rock_manager.sample(local_coordinates)
            print("rock manager sampled")
            self.collider_manager.update_shifting_map(local_coordinates)
            print("collider manager updated")
            update = True
        else:
            corrected_coordinates = (0, 0)
            update = False

        return update, corrected_coordinates
