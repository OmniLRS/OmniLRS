__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import Tuple, List
import dataclasses
import numpy as np
import cv2

import omni


from src.terrain_management.large_scale_terrain.collider_builder import ColliderBuilder, ColliderBuilderConf
from src.terrain_management.large_scale_terrain.utils import ScopedTimer


@dataclasses.dataclass
class ColliderManagerConf:
    """
    Args:
        collider_resolution (float): resolution of the collider (meter per pixel).
        block_size (int): size of the block (meters).
        collider_path (str): path to the collider.
        cache_size (int): size of the collider cache. Must be larger or equal to 4.
        build_colliders_n_meters_ahead (int): number of meters ahead to build colliders.
        collider_builder_conf (dict): configuration for the collider builder.
    """

    collider_resolution: float = dataclasses.field(default_factory=float)
    block_size: int = dataclasses.field(default_factory=int)
    collider_path: str = dataclasses.field(default_factory=str)
    cache_size: int = dataclasses.field(default_factory=int)
    build_colliders_n_meters_ahead: int = dataclasses.field(default_factory=int)
    collider_builder_conf: ColliderBuilderConf = dataclasses.field(default_factory=dict)
    profiling: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        assert self.build_colliders_n_meters_ahead > 0, "build_colliders_n_meters_ahead must be greater than 0"
        assert (
            self.build_colliders_n_meters_ahead < self.block_size
        ), "build_colliders_n_meters_ahead must be smaller than block_size"
        assert self.cache_size >= 4, "collider_cache_size must be greater or equal to 4"

        self.collider_builder_conf = ColliderBuilderConf(**self.collider_builder_conf)


class ColliderManager:
    """
    Class to manage the colliders for the terrain.
    Because building colliders is expensive, we only build them when needed.
    Also, we only remove them once we are sure we don't need them anymore.
    This means we have a cache of active colliders.
    Building colliders is a "locking" action: it will block the main simulation thread,
    hence we can generate colliders at the very last moment.
    """

    def __init__(
        self,
        settings: ColliderManagerConf,
        hr_dem: np.ndarray,
        hr_dem_shape: Tuple[int, int],
        dem_center: Tuple[float, float],
        source_resolution: float,
    ) -> None:
        """
        Args:
            settings (ColliderManagerConf): configuration for the collider manager.
            hr_dem (np.ndarray): high resolution DEM.
            hr_dem_shape (Tuple[int, int]): shape of the high resolution DEM.
            dem_center (Tuple[float, float]): center of the DEM.
        """

        self.settings = settings
        self.hr_dem = hr_dem
        self.hr_dem_shape = hr_dem_shape
        self.dem_center = dem_center
        self.stage = omni.usd.get_context().get_stage()
        self.cache = {}
        self.current_coordinates = (0, 0)
        self.source_resolution = source_resolution
        self.block_hw = int(self.settings.block_size / self.settings.collider_resolution) + 1

    def cast_coordinates_to_block_space(self, coordinates: Tuple[float, float]) -> Tuple[int, int]:
        """
        Cast the coordinates to the block space.

        Args:
            coordinates (Tuple[float, float]): coordinates to cast.

        Returns:
            Tuple[int, int]: coordinates in the block space.
        """

        x = int(coordinates[0] // self.settings.block_size) * self.settings.block_size
        y = int(coordinates[1] // self.settings.block_size) * self.settings.block_size
        return (x, y)

    def build(self) -> None:
        """
        Builds the collider builder and applies it to generate the colliders for the terrain.
        """

        with ScopedTimer("build_collider_manager", active=self.settings.profiling):
            self.collider_builder = ColliderBuilder(self.settings.collider_builder_conf, self.stage)
            self.collider_builder.build_base_grid()

    def update(self, global_coordinates: Tuple[float, float]) -> None:
        """
        Updates the collider manager given the coordinates.
        Adds the colliders that are needed and removes the ones that are not needed anymore.

        Args:
            global_coordinates (Tuple[float, float]): coordinates to update the collider in meters.
            map_coordinates (Tuple[float, float]): coordinates to update the collider in meters.

        """

        with ScopedTimer("update_collider_manager", active=self.settings.profiling):
            with ScopedTimer("get_blocks_to_build", active=self.settings.profiling, unit="us"):
                self.current_coordinates = self.cast_coordinates_to_block_space(global_coordinates)
                blocks_to_build = self.get_blocks_to_build(global_coordinates)
            with ScopedTimer("build_colliders", active=self.settings.profiling):
                for block_coord in blocks_to_build:
                    if block_coord not in self.cache.keys():
                        self.cache[block_coord] = self.collider_builder.create_collider(
                            block_coord, self.get_terrain_block(block_coord), self.get_name(block_coord)
                        )
            with ScopedTimer("prune_blocks", active=self.settings.profiling, unit="us"):
                self.prune_blocks()

    def update_shifting_map(self, global_coordinates: Tuple[float, float]) -> None:
        """
        Updates the collider manager given the coordinates in a shifting map. That is a map that moves. The main
        difference with the other method is that since the map moves all the time, we can't use a fixed indexing to
        know where the blocks are. Instead we need to get some information regarding our relative position in the
        map frame.

        Adds the colliders that are needed and removes the ones that are not needed anymore.

        Args:
            global_coordinates (Tuple[float, float]): coordinates to update the collider in meters.

        """

        with ScopedTimer("update_collider_manager", active=self.settings.profiling):
            with ScopedTimer("get_blocks_to_build", active=self.settings.profiling, unit="us"):
                self.current_coordinates = self.cast_coordinates_to_block_space(global_coordinates)
                blocks_to_build = self.get_blocks_to_build(global_coordinates)
            with ScopedTimer("build_colliders", active=self.settings.profiling):
                for block_coord in blocks_to_build:
                    if block_coord not in self.cache.keys():
                        map_coord = (
                            block_coord[1] - self.current_coordinates[1],
                            block_coord[0] - self.current_coordinates[0],
                        )
                        self.cache[block_coord] = self.collider_builder.create_collider(
                            block_coord, self.get_terrain_block(map_coord).T, self.get_name(block_coord)
                        )
            with ScopedTimer("prune_blocks", active=self.settings.profiling, unit="us"):
                self.prune_blocks()

    def get_blocks_to_build(self, global_coordinates: Tuple[float, float]) -> List[Tuple[int, int]]:
        """
        Returns the blocks to build given the coordinates.

        Args:
            global_coordinates (Tuple[float, float]): coordinates in meters.

        Returns:
            List[Tuple[int, int]]: blocks to build.
        """

        blocks_to_build = set()
        for i in [-1, 0, 1]:
            x = global_coordinates[0] + i * self.settings.build_colliders_n_meters_ahead
            for j in [-1, 0, 1]:
                y = global_coordinates[1] + j * self.settings.build_colliders_n_meters_ahead
                block_coord = self.cast_coordinates_to_block_space((x, y))
                blocks_to_build.add(block_coord)

        return list(blocks_to_build)

    def prune_blocks(self):
        """
        Prunes the blocks that are not needed anymore. The farthest blocks are removed until the cache is of the right size.
        """

        dists = {}
        if len(self.cache) > self.settings.cache_size:
            num_to_remove = len(self.cache) - self.settings.cache_size
            dists = {
                cache_coord: (cache_coord[0] - self.current_coordinates[0])
                * (cache_coord[0] - self.current_coordinates[0])
                + (cache_coord[1] - self.current_coordinates[1]) * (cache_coord[1] - self.current_coordinates[1])
                for cache_coord in self.cache.keys()
            }
            sorted_dists = sorted(dists.items(), key=lambda x: x[1])
            to_remove = [x[0] for x in sorted_dists[-num_to_remove:]]
            for coord in to_remove:
                self.collider_builder.remove_collider(self.cache[coord])
                self.cache.pop(coord)

    @staticmethod
    def get_name(coord: Tuple[int, int]) -> str:
        """
        Returns the name of the collider given the coordinates.

        Args:
            coord (Tuple[int, int]): coordinates of the collider.

        Returns:
            str: name of the collider.
        """

        if coord[0] > 0:
            x = str(coord[0])
        else:
            x = "m" + str(coord[0] * -1)
        if coord[1] > 0:
            y = str(coord[1])
        else:
            y = "m" + str(coord[1] * -1)
        return f"collider_{x}_{y}"

    def get_terrain_block(self, coords: Tuple[float, float]) -> None:
        """
        Returns a terrain block given the coordinates. Note the +1 in the slicing to include the last pixel
        so that the colliders joint correctly.

        Args:
            coords (Tuple[float, float]): coordinates of the block.
        """

        x_min = (self.dem_center[0] + coords[0]) / self.source_resolution
        x_max = (self.dem_center[0] + coords[0] + self.settings.block_size) / self.source_resolution
        y_min = (self.dem_center[1] + coords[1]) / self.source_resolution
        y_max = (self.dem_center[1] + coords[1] + self.settings.block_size) / self.source_resolution
        source_data_block = self.hr_dem[int(y_min) : int(y_max) + 1, int(x_min) : int(x_max) + 1]
        return cv2.resize(source_data_block, (self.block_hw, self.block_hw), interpolation=cv2.INTER_LINEAR)
