__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from matplotlib import pyplot as plt
from typing import List, Tuple
import dataclasses
import numpy as np
import colorsys

from src.terrain_management.large_scale_terrain.utils import (
    BoundingBox,
    RockBlockData,
    ScopedTimer,
)
from src.terrain_management.large_scale_terrain.rock_database import RockDB

# TODO (antoine.richard / JAOPS): The current implementation can be extremely costly in terms of memory.
# For small rocks, whose density can be very high, the memory footprint will be very large.
# Sampling them on the fly could be a better solution, but the required compute, and added complexity
# may not be worth it. It may be more relevant to sample the rocks in a narrow region around the robot
# and call it a day. Alternatively, we could switch to an SQLite database to reduce memory usage, but
# this would then impact the disk lifespan. The data is already compressed with best in class algorithm (ZFP
# from the Lawrence Livermore National Laboratory), so there is not much we can do on that side.


def mock_call(
    x: np.ndarray, y: np.ndarray, coordinates: Tuple[float, float], seed: int
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Mock function to generate the z and quaternion values.
    Quaternions values are given as [x, y, z, w].

    Args:
        x (np.ndarray): x coordinates.
        y (np.ndarray): y coordinates.
        seed (int): seed for the random number generator.

    Returns:
        Tuple[np.ndarray, np.ndarray]: z coordinates, quaternion values.
    """

    z = np.zeros_like(x)
    quat = np.zeros((x.shape[0], 4))
    quat[:, -1] = 1
    return z, quat


@dataclasses.dataclass
class BaseDistribution:
    """
    Base class for distributions.

    Args:
        name (str): name of the distribution.
    """

    name: str = dataclasses.field(default_factory=str)
    seed: int = None

    def sample(self, **kwargs) -> np.ndarray:
        raise NotImplementedError

    def set_seed(self, seed: int) -> None:
        self.seed = int(seed)
        self.rng = np.random.default_rng(seed)

    def __call__(self, **kwargs) -> np.ndarray:
        return self.sample(**kwargs)


@dataclasses.dataclass
class Poisson(BaseDistribution):
    """
    Sample from a Poisson distribution.

    Args:
        name (str): name of the distribution.
        density (float): density of the distribution.
        seed (int): seed for the random number generator.
    """

    name: str = "poisson"
    density: float = dataclasses.field(default_factory=float)
    seed: int = None

    def __post_init__(self) -> None:
        assert self.density > 0, "density must be larger than 0."
        assert type(self.density == float), "density must be a float."

        self.density = float(self.density)
        if self.seed is not None:
            self.seed = int(self.seed)
            self.rng = np.random.default_rng(self.seed)

    def get_num_points(self, region: BoundingBox) -> int:
        """
        Get the number of points to sample from a Poisson distribution.

        Args:
            region (BoundingBox): region to sample the points from.

        Returns:
            int: number of points.
        """

        area = (region.x_max - region.x_min) * (region.y_max - region.y_min)
        return self.rng.poisson(area * self.density)

    def sample(self, region: BoundingBox = BoundingBox(), density: int = None, **kwargs) -> Tuple[np.ndarray, int]:
        """
        Sample from a Poisson distribution.

        Args:
            region (BoundingBox): region to sample the points from.
            density (int): density of the distribution.
            **kwargs: additional arguments.

        Returns:
            Tuple[np.ndarray, int]: coordinates of the points, number of points.
        """

        num_points = self.get_num_points(region)
        if density is not None:
            self.density = density
        x_coords = self.rng.uniform(region.x_min, region.x_max, num_points)
        y_coords = self.rng.uniform(region.y_min, region.y_max, num_points)
        return np.stack([x_coords, y_coords]).T, num_points


@dataclasses.dataclass
class ThomasPointProcess(BaseDistribution):
    """
    Sample from a Thomas point process.

    Args:
        name (str): name of the distribution.
        parent_density (float): density of the parent process.
        child_density (float): density of the child process.
        sigma (float): standard deviation of the normal distribution.
        seed (int): seed for the random number generator.
    """

    name: str = "thomas_point_process"
    parent_density: float = dataclasses.field(default_factory=float)
    child_density: float = dataclasses.field(default_factory=float)
    sigma: float = dataclasses.field(default_factory=float)
    seed: int = None

    def __post_init__(self):
        assert type(self.parent_density == float), "parent_density must be a float."
        assert self.parent_density > 0, "parent_density must be larger than 0."
        assert type(self.child_density == float), "child_density must be a float."
        assert self.child_density > 0, "child_density must be larger than 0."
        assert type(self.sigma == float), "sigma must be a float."
        assert self.sigma > 0, "sigma must be larger than 0."

        self.parent = Poisson(name="poisson", density=self.parent_density, seed=self.seed)
        self.normal = Normal(name="normal", mean=0.0, std=self.sigma, seed=self.seed)
        self.extension = 7 * self.sigma
        if self.seed is not None:
            self.set_seed(self.seed)

    def set_seed(self, seed: int) -> None:
        self.rng = np.random.default_rng(seed)
        if self.parent.seed is None:
            self.parent.set_seed(seed)
        if self.normal.seed is None:
            self.normal.set_seed(seed)

    def sample_parents(self, region: BoundingBox) -> Tuple[np.ndarray, int]:
        """
        Sample parents of the thomas point process.

        Args:
            region (BoundingBox): region to sample the parents from.

        Returns:
            Tuple[np.ndarray, int]: coordinates of the parents, number of parents.
        """

        region_ext = BoundingBox(region.x_min, region.x_max, region.y_min, region.y_max)
        region_ext.x_min = region.x_min - self.extension
        region_ext.x_max = region.x_max + self.extension
        region_ext.y_min = region.y_min - self.extension
        region_ext.y_max = region.y_max + self.extension

        area_region = region.get_area()
        area_region_ext = region_ext.get_area()

        ratio = area_region / area_region_ext

        adjusted_density = self.parent_density * ratio

        parent_coords, num_parents = self.parent.sample(region, density=adjusted_density)
        return parent_coords, num_parents

    def sample_children(self, parent_coords: np.ndarray, num_parents: int) -> Tuple[np.ndarray, int]:
        """
        Sample children of the thomas point process.

        Args:
            parent_coords (np.ndarray): coordinates of the parents.
            num_parents (int): number of parents.
        """

        num_child = self.rng.poisson(self.child_density, num_parents)
        num_points = np.sum(num_child)
        children_coords = self.normal.sample(num_points, 2)
        children_coords = children_coords + np.repeat(parent_coords, num_child, axis=0)
        return children_coords, num_points

    def sample(self, region: BoundingBox = BoundingBox(), **kwargs) -> Tuple[np.ndarray, int]:
        """
        Sample from a Thomas point process.
        It first samples the number of parents in the region using a Poisson distribution.
        Then, it samples the coordinates of the parents in the 2D space defined by the region.
        Using the parent coordinates, it samples the number of children for each parent using a
        Poisson distribution. Finally, it samples the children coordinates using a normal distribution
        centered around the parent coordinates.

        Args:
            region (BoundingBox): region to sample the points from.

        Returns:
            Tuple[np.ndarray, int]: coordinates of the points, number of points.
        """

        parent_coords, num_parents = self.sample_parents(region)
        children_coords, num_children = self.sample_children(parent_coords, num_parents)
        return children_coords, num_children


@dataclasses.dataclass
class Uniform(BaseDistribution):
    """
    Sample from a uniform distribution.

    Args:
        name (str): name of the distribution.
        min (float): minimum value.
        max (float): maximum value.
        seed (int): seed for the random number generator.
    """

    name: str = "uniform"
    min: float = dataclasses.field(default_factory=float)
    max: float = dataclasses.field(default_factory=float)
    seed: int = None

    def __post_init__(self):
        assert type(self.min) == float, "min must be a float"
        assert type(self.max) == float, "max must be a float"
        assert self.min < self.max, "min must be smaller than max"
        if self.seed is not None:
            self.set_seed(self.seed)
        self.min = float(self.min)
        self.max = float(self.max)

    def sample(self, num_points: int = 1, dim: int = 1, **kwargs):
        """
        Sample from a uniform distribution.
        dim is used to specify the dimension of the output.
        That is, if dim is 1: the output will be [num_points], if dim is 2: the output will be [num_points, 2],
        if dim is 3: the output will be [num_points, 3], etc.

        Args:
            num_points (int): number of points to sample.
            dim (int): dimension of the generated output.
            **kwargs: additional arguments.
        """
        return self.rng.uniform(self.min, self.max, (num_points, dim))


@dataclasses.dataclass
class Normal(BaseDistribution):
    """
    Sample from a normal distribution.

    Args:
        name (str): name of the distribution.
        mean (float): mean of the distribution.
        std (float): standard deviation of the distribution.
        seed (int): seed for the random number generator.
    """

    name: str = "normal"
    mean: float = dataclasses.field(default_factory=float)
    std: float = dataclasses.field(default_factory=float)
    seed: int = None

    def __post_init__(self):
        assert type(self.mean) == float, "mean must be a float"
        assert type(self.std) == float, "std must be a float"
        assert self.std > 0, "std must be larger than 0"
        self.mean = float(self.mean)
        self.std = float(self.std)
        if self.seed is not None:
            self.set_seed(self.seed)

    def sample(self, num_points: int = 1, dim: int = 1, **kwargs):
        """
        Sample from a normal distribution.
        dim is used to specify the dimension of the output.
        That is, if dim is 1: the output will be [num_points], if dim is 2: the output will be [num_points, 2],
        if dim is 3: the output will be [num_points, 3], etc.

        Args:
            num_points (int): number of points to sample.
            dim (int): dimension of the generated output.
            **kwargs: additional arguments.
        """

        return self.rng.normal(self.mean, self.std, (num_points, dim))


@dataclasses.dataclass
class Integer(BaseDistribution):
    """
    Sample integer uniformly between min and max.

    Args:
        name (str): name of the distribution.
        min (int): minimum value.
        max (int): maximum value.
        seed (int): seed for the random number generator.
    """

    name: str = "integer"
    min: int = dataclasses.field(default_factory=int)
    max: int = dataclasses.field(default_factory=int)
    seed: int = None

    def __post_init__(self):
        assert self.min < self.max, "min must be smaller than max"
        assert type(self.min) == int, "min must be an integer"
        assert type(self.max) == int, "max must be an integer"
        self.min = int(self.min)
        self.max = int(self.max)
        if self.seed is not None:
            self.set_seed(self.seed)

    def sample(self, num_points: int = 1, **kwargs):
        """
        Sample integer uniformly between min and max.

        Args:
            num_points (int): number of points to sample.
            **kwargs: additional arguments.
        """

        return self.rng.integers(self.min, self.max, num_points)


class Factory:
    """
    Factory class to create objects based on configs.
    """

    def __init__(self) -> None:
        """
        Args:
            cfg (dict): configuration for the distribution.
        """

        self.objs = {}

    def add(self, obj, name) -> None:
        """
        Creates the distribution object.
        """

        self.objs[name] = obj

    def create(self, cfg) -> object:
        """
        Creates the distribution object.
        """

        if cfg["name"] not in self.objs:
            raise ValueError(f"Unknown distribution: {cfg['name']}")

        return self.objs[cfg["name"]](**cfg)


distribution_factory = Factory()
distribution_factory.add(Poisson, "poisson")
distribution_factory.add(ThomasPointProcess, "thomas_point_process")
distribution_factory.add(Uniform, "uniform")
distribution_factory.add(Normal, "normal")
distribution_factory.add(Integer, "integer")


@dataclasses.dataclass
class RockDynamicDistributionConf:
    """
    Args:
        position_distribution (dict): configuration for the position distribution.
        scale_distribution (dict): configuration for the scale distribution.
        seed (int): seed for the random number generator.
        num_rock_id (int): number of rock ids.
    """

    position_distribution: BaseDistribution = dataclasses.field(default_factory=dict)
    scale_distribution: BaseDistribution = dataclasses.field(default_factory=dict)
    seed: int = None

    def __post_init__(self):
        # Reseed the random number generator
        if self.seed is not None:
            assert type(self.seed) == int, "seed must be an integer"
            self.seed = int(self.seed)


class DynamicDistribute:
    """
    Distributes rocks on a DEM using the distribution given in the configuration.
    """

    def __init__(
        self,
        settings: RockDynamicDistributionConf,
        sampling_func: callable = mock_call,
        num_objects: int = 1,
    ) -> None:
        """
        Args:
            settings (RockDynamicDistributionConf): settings for the rock distribution.
            sampling_func (function): function to sample the z and quaternion values.
            num_objects (int): number of objects to sample.
        """

        self.settings = settings
        self.num_objects = num_objects
        self._rng = np.random.default_rng(self.settings.seed)
        self.sampling_func = sampling_func

    def build_samplers(self) -> None:
        """
        Builds the samplers for the position and scale distributions.
        Note that the orientation is note sampled from here. For the rocks,
        we assume their z axis is aligned with the normal to the terrain
        surface, and apply a random rotation around this axis.

        This process is handed over to the sampling function. (self.sampling_func)
        """

        id_sampler_cfg = {
            "name": "integer",
            "min": 0,
            "max": self.num_objects,
            "seed": self.settings.seed + 1,
        }

        self.position_sampler: BaseDistribution = distribution_factory.create(self.settings.position_distribution)
        self.scale_sampler: BaseDistribution = distribution_factory.create(self.settings.scale_distribution)
        self.id_sampler: Integer = distribution_factory.create(id_sampler_cfg)

        if self.position_sampler.seed is None:
            self.position_sampler.set_seed(self.settings.seed + 2)
        if self.scale_sampler.seed is None:
            self.scale_sampler.set_seed(self.settings.seed + 3)

    def run(self, region: BoundingBox, map_coordinates: Tuple[float, float]) -> Tuple[RockBlockData, bool]:
        """
        Runs the rock distribution.

        Args:
            region (BoundingBox): region to sample the rocks from.
            map_coordinates (Tuple[float, float]): coordinates of the map.

        Returns:
            Tuple[RockBlockData, bool]: block of rocks, not empty. Returns False if the block is empty.
        """

        xy_position, num_points = self.position_sampler(region=region)
        scale = self.scale_sampler(num_points=num_points, dim=3)
        ids = self.id_sampler(num_points=num_points)
        z_position, quat = self.sampling_func(xy_position[:, 0], xy_position[:, 1], map_coordinates, self.settings.seed)
        xyz_position = np.stack([xy_position[:, 0], xy_position[:, 1], z_position]).T
        block = RockBlockData(xyz_position, quat, scale, ids)
        return block, num_points > 0

    def get_xy_coordinates_from_block(self, block: RockBlockData) -> np.ndarray:
        """
        Extracts the xy coordinates from the block.

        Args:
            block (RockBlockData): block of rocks.

        Returns:
            np.ndarray: xy coordinates.
        """

        return block.coordinates[:, :2]


@dataclasses.dataclass
class RockSamplerConf:
    """
    Args:
        block_size (int): size of the blocks.
        rock_dist_cfg (RockDynamicDistributionConf): configuration for the rock distribution
    """

    block_size: int = dataclasses.field(default_factory=int)
    seed: int = None
    rock_dist_cfg: RockDynamicDistributionConf = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        self.rock_dist_cfg = RockDynamicDistributionConf(**self.rock_dist_cfg)


class RockSampler:
    """
    Class to scatter rocks on a DEM. It aggregates the rock distribution, the rock
    generation and the rock database to generate rocks on the fly.
    """

    def __init__(
        self,
        rock_sampler_cfg: RockSamplerConf,
        db: RockDB,
        map_sampling_func: callable = mock_call,
        num_objects: int = 1,
        profiling: bool = False,
    ) -> None:
        """
        Args:
            rock_sampler_cfg (RockSamplerConf): configuration for the rock sampler.
            db (RockDB): database to store the rocks.
            map_sampling_func (function): function to sample the z and quaternion values.
            num_objects (int): number of objects to sample.
            profiling (bool): flag to enable profiling.
        """

        self.settings = rock_sampler_cfg
        self.num_objects = num_objects
        self.rock_db = db
        self.profiling = profiling
        self.build_samplers(map_sampling_func, num_objects)

    def build_samplers(self, map_sampling_func: callable = mock_call, num_objects: int = 1) -> None:
        self.rock_dist_gen = DynamicDistribute(
            self.settings.rock_dist_cfg, sampling_func=map_sampling_func, num_objects=num_objects
        )
        if self.rock_dist_gen.settings.seed is None:
            self.rock_dist_gen.settings.seed = self.settings.seed * 4
        self.rock_dist_gen.build_samplers()

    @staticmethod
    def compute_largest_rectangle(matrix: np.ndarray) -> Tuple[int, int]:
        """
        Computes the largest rectangle in a binary matrix.
        This is used to find the largest region without rocks in the matrix.

        Args:
            matrix (np.ndarray): binary matrix.

        Returns:
            Tuple[float, Tuple[int,int]]: the area and coordinates of the largest rectangle.
        """

        m, n = matrix.shape
        left = np.zeros((n), dtype=int)
        right = np.ones((n), dtype=int) * n
        height = np.zeros((n), dtype=int)

        max_area = 0
        s_i = -1
        s_height = -1
        s_left = -1
        s_right = -1

        for i in range(m):
            cur_left, cur_right = 0, n

            for j in range(n - 1, -1, -1):
                if matrix[i, j] == 0:
                    right[j] = min(right[j], cur_right)
                else:
                    right[j] = n
                    cur_right = j

            for j in range(n):
                if matrix[i, j] == 0:
                    height[j] += 1
                    left[j] = max(left[j], cur_left)
                else:
                    height[j] = 0
                    left[j] = 0
                    cur_left = j + 1

                v = max(max_area, (right[j] - left[j]) * height[j])
                if v > max_area:
                    max_area = v
                    s_i = i
                    s_left = left[j]
                    s_right = right[j]
                    s_height = height[j]

        coords = ((s_i - s_height + 1, s_i + 1), (s_left, s_right))
        return max_area, coords

    def sample_rocks_by_block(self, block_coordinates: Tuple[int, int], map_coordinates: Tuple[float, float]) -> None:
        """
        Samples rocks by block. This method is used to sample rocks in a block
        that does not contain any rocks.

        Args:
            block_coordinates (Tuple[int,int]): coordinates of the block.
            map_coordinates (Tuple[float, float]): coordinates of the map.
        """

        with ScopedTimer("Sampling rocks block", active=self.profiling):
            # Checks if the block is valid
            self.rock_db.is_valid(block_coordinates)

            # Checks if the block already contains rocks
            if not self.rock_db.check_block_exists(block_coordinates):
                bb = BoundingBox(
                    block_coordinates[0],
                    block_coordinates[0] + self.settings.block_size,
                    block_coordinates[1],
                    block_coordinates[1] + self.settings.block_size,
                )
                block, not_empty = self.rock_dist_gen.run(bb, map_coordinates)
                if not_empty:
                    self.rock_db.add_block_data(block, block_coordinates)

    def dissect_region_blocks(
        self, block: RockBlockData, region: BoundingBox
    ) -> Tuple[List[RockBlockData], List[Tuple[float, float]]]:
        """
        Dissects the region into blocks and returns the rocks in each block.

        Args:
            blocks RockBlockData: the data block containing the rocks.
            region (BoundingBox): region to dissect.

        Returns:
            Tuple[List[RockBlockData], List[Tuple[float, float]]]: list of
            rock data for each block,  coordinates of the blocks.
        """

        block_list = []
        coordinate_list = []

        xy = self.rock_dist_gen.get_xy_coordinates_from_block(block)
        for i, x in enumerate(range(region.x_min, region.x_max, self.settings.block_size)):
            for j, y in enumerate(range(region.y_min, region.y_max, self.settings.block_size)):
                # If rocks are generated outside the regin boundaries, we store them
                # in the boundary blocks. We do this to prevent the db from believing
                # data has been generated in the neighboring blocks. Ideally, we would
                # like to avoid this, but this would require extra work on the db side
                # to flag partially generated blocks.
                # The issue will be that we will need to be careful when querying the
                # DEM to avoid requesting data outside its boundaries.
                # TODO (antoine.richard / JAOPS): handle this case properly

                if i == 0:
                    c1 = np.ones_like(xy[:, 0], dtype=bool)
                else:
                    c1 = xy[:, 0] >= x
                if i == (region.x_max - region.x_min - 1) // self.settings.block_size:
                    c2 = np.ones_like(xy[:, 0], dtype=bool)
                else:
                    c2 = xy[:, 0] < x + self.settings.block_size
                if j == 0:
                    c3 = np.ones_like(xy[:, 1], dtype=bool)
                else:
                    c3 = xy[:, 1] >= y
                if j == (region.y_max - region.y_min - 1) // self.settings.block_size:
                    c4 = np.ones_like(xy[:, 1], dtype=bool)
                else:
                    c4 = xy[:, 1] < y + self.settings.block_size

                cond = c1 & c2 & c3 & c4

                idx = np.where(cond)
                if len(idx[0]) > 0:
                    block_list.append(
                        RockBlockData(
                            block.coordinates[idx],
                            block.quaternion[idx],
                            block.scale[idx],
                            block.ids[idx],
                        )
                    )
                    coordinate_list.append((x, y))
        return block_list, coordinate_list

    def sample_rocks_by_region(self, region: BoundingBox, map_coordinates: Tuple[float, float]) -> None:
        """
        Samples rocks by region. This method is used to sample rocks in a region.
        It will sample rocks only in the blocks that do not contain any rocks.
        To optimize the process, it finds the largest rectangle in the region that does not
        contain any rocks and samples rocks in this region. This process is repeated
        until the largest rectangle is smaller than a certain threshold. After that,
        the method samples rocks in the remaining blocks that do not contain any rocks.

        Args:
            region (BoundingBox): region to sample rocks from.
            map_coordinates (Tuple[float, float]): coordinates of the map.
        """

        # Process by regions until the largest rectangle is smaller or equal to 1 block
        process_by_regions = True
        while process_by_regions:
            with ScopedTimer("Getting blocks within region", active=self.profiling):
                matrix = self.rock_db.get_occupancy_matrix_within_region(region)
            # Computes the largest rectangle in the region that does not contain any rocks
            with ScopedTimer("Computing largest rectangle", active=self.profiling):
                area, coords = self.compute_largest_rectangle(matrix)
            # If the area is smaller or equal to 1 block, we stop the process
            # and fallback to the block sampling method
            if area <= 1:
                process_by_regions = False
                break

            # Region to sample rocks from
            new_region = BoundingBox(
                region.x_min + coords[0][0] * self.settings.block_size,
                region.x_min + coords[0][1] * self.settings.block_size,
                region.y_min + coords[1][0] * self.settings.block_size,
                region.y_min + coords[1][1] * self.settings.block_size,
            )

            # Samples rocks in the region
            with ScopedTimer("Sampling rocks in region", active=self.profiling):
                new_block, not_empty = self.rock_dist_gen.run(new_region, map_coordinates)

            if not_empty:
                with ScopedTimer("Getting xy coordinates from block", active=self.profiling):
                    coords = self.rock_dist_gen.get_xy_coordinates_from_block(new_block)

                # Dissects the region into blocks and adds the rocks to the database
                with ScopedTimer("Dissecting region into blocks", active=self.profiling):
                    new_blocks_list, block_coordinates_list = self.dissect_region_blocks(new_block, new_region)

                with ScopedTimer("Adding block data to the database", active=self.profiling):
                    for block_data, block_coordinates in zip(new_blocks_list, block_coordinates_list):
                        self.rock_db.add_block_data(block_data, block_coordinates)

        # If the largest rectangle is smaller or equal to 1 block, we sample rocks
        # on a per block basis.
        with ScopedTimer("Sampling rocks by block", active=self.profiling):
            with ScopedTimer("Getting missing blocks", active=self.profiling):
                coordinates = self.rock_db.get_missing_blocks(region)

            for coord in coordinates:
                self.sample_rocks_by_block(coord, map_coordinates)

    def display_block(self, coordinates: Tuple[float, float]):
        """
        Displays the rocks in a block.

        Args:
            coordinates (Tuple[float,float]): coordinates of the block.
        """

        fig = plt.figure(figsize=(10, 10), dpi=300)
        if self.rock_db.check_block_exists(coordinates):
            block = self.rock_db.get_block_data(coordinates)
            coordinates = self.rock_dist_gen.get_xy_coordinates_from_block(block)
            plt.scatter(coordinates[:, 0], coordinates[:, 1])
            plt.axis("equal")

    def display_block_set(self, coords: List[Tuple[float, float]]):
        """
        Displays a set of blocks. Each block is represented by a different color.

        Args:
            coords (List[Tuple[float,float]]): list of coordinates of the blocks.
        """

        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks = [self.rock_db.get_block_data(coord) for coord in coords]

        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i, block in enumerate(blocks):
            coordinates = self.rock_dist_gen.get_xy_coordinates_from_block(block)
            plt.scatter(coordinates[:, 0], coordinates[:, 1], c=[colors[i]])
        plt.axis("equal")

    def display_region(self, region: BoundingBox):
        """
        Displays the rocks in a region. Each block is represented by a different color.

        Args:
            region (BoundingBox): region to display.
        """

        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks, _, _ = self.rock_db.get_blocks_within_region(region)

        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i in range(len(blocks)):
            coordinates = self.rock_dist_gen.get_xy_coordinates_from_block(blocks[i])
            plt.scatter(coordinates[:, 0], coordinates[:, 1], c=[colors[i]])
        plt.axis("equal")
