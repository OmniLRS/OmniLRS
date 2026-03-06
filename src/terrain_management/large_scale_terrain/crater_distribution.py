__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from scipy.interpolate import CubicSpline
from matplotlib import pyplot as plt
from typing import List, Tuple
import dataclasses
import numpy as np
import colorsys
import logging
import pickle

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

from src.terrain_management.large_scale_terrain.utils import BoundingBox, CraterMetadata
from src.terrain_management.large_scale_terrain.crater_database import CraterDB


@dataclasses.dataclass
class CraterDynamicDistributionConf:
    """
    Args:
        densities (List[float]): list of densities for the craters.
        radius (List[float]): list of radii for the craters.
        num_repeat (int): number of times to repeat the hardcore rejection.
        seed (int): seed for the random number generator.
    """

    densities: List[float] = dataclasses.field(default_factory=list)
    radius: List[float] = dataclasses.field(default_factory=list)
    num_repeat: int = 1
    seed: int = 42

    def __post_init__(self):
        assert len(self.densities) == len(self.radius), "The ..."
        self.num_repeat = int(self.num_repeat)
        assert self.num_repeat > 0, "Num ..."


class DynamicDistribute:
    """
    Distributes craters on a DEM using a Poisson process with hardcore rejection.
    """

    def __init__(
        self,
        settings: CraterDynamicDistributionConf,
    ) -> None:
        """
        Args:
            settings (CraterDynamicDistributionConf): settings for the crater distribution.
        """

        self.settings = settings
        self._rng = np.random.default_rng(self.settings.seed)

    def sample_from_poisson(
        self,
        region: BoundingBox,
        l: float,
        r_minmax: Tuple[float, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Samples from a Poisson process.

        Args:
            region (BoundingBox): region to sample from.
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (Tuple[float,float]): minimum and maximum radius of the craters (in meters).

        Returns:
            Tuple[np.ndarray, np.ndarray]: coordinates and radius of the craters.
        """

        area = (region.x_max - region.x_min) * (region.y_max - region.y_min)
        num_points = self._rng.poisson(area * l)
        radius = self._rng.uniform(r_minmax[0], r_minmax[1], num_points)
        x_coords = self._rng.uniform(region.x_min, region.x_max, num_points)
        y_coords = self._rng.uniform(region.y_min, region.y_max, num_points)
        return np.stack([x_coords, y_coords]).T, radius

    def hardcore_rejection(self, coords: np.ndarray, radius: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Performs hardcore rejection on the craters. This operation is
        expensive and should be used with caution.

        Args:
            coords (np.ndarray): coordinates of the craters (in meters).
            radius (np.ndarray): radii of the craters (in meters).

        Returns:
            Tuple[np.ndarray, np.ndarray]: coordinates of the craters (in meters).
        """

        mark_age = self._rng.uniform(0, 1, coords.shape[0])
        boole_keep = np.zeros(mark_age.shape[0], dtype=bool)
        for i in range(mark_age.shape[0]):
            dist_tmp = np.linalg.norm(coords[i] - coords, axis=-1)
            in_disk = (dist_tmp < radius[i]) & (dist_tmp > 0)
            if len(mark_age[in_disk]) == 0:
                boole_keep[i] = True
            else:
                boole_keep[i] = all(mark_age[i] < mark_age[in_disk])
        return coords[boole_keep], radius[boole_keep]

    def check_previous(
        self, new_coords: np.ndarray, radius: np.ndarray, prev_coords: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Checks if the new craters are not contained in the previous craters.

        Args:
            new_coords (np.ndarray): coordinates of the new craters.
            radius (np.ndarray): radii of the new craters.
            prev_coords (np.ndarray): coordinates of the previous craters.

        Returns:
            Tuple[np.ndarray, np.ndarray]: the coordinates of the new craters, the radii of the new craters.
        """

        boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        if prev_coords is None:
            boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        else:
            for i in range(prev_coords[0].shape[0]):
                dist_tmp = np.linalg.norm(prev_coords[0][i] - new_coords, axis=-1)
                in_disk = (dist_tmp < prev_coords[1][i]) & (dist_tmp > 0)
                boole_keep[in_disk] = False
        return new_coords[boole_keep], radius[boole_keep]

    def simulate_HC_poisson_process(
        self,
        region: BoundingBox,
        l: float,
        r_minmax: Tuple[float],
        prev_coords: np.ndarray = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates a hardcore Poisson process.

        Args:
            region (BoundingBox): region to sample from.
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).
            prev_coords (np.ndarray): coordinates of the previous craters (in meters).

        Returns:
            Tuple[np.ndarray, np.ndarray]: coordinates of the craters, radii of the craters.
        """

        coords, radius = self.sample_from_poisson(region, l, r_minmax)
        for _ in range(self.settings.num_repeat):
            coords, radius = self.hardcore_rejection(coords, radius)
            new_coords, new_radius = self.sample_from_poisson(region, l, r_minmax)
            coords = np.concatenate([coords, new_coords])
            radius = np.concatenate([radius, new_radius])
            self.check_previous(coords, radius, prev_coords)
        coords, radius = self.hardcore_rejection(coords, radius)
        coords, radius = self.check_previous(coords, radius, prev_coords)
        return coords, radius

    def simulate_poisson_process(
        self,
        region: BoundingBox,
        l: float,
        r_minmax: Tuple[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates a Poisson process.

        Args:
            region (BoundingBox): region to sample from.
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).
            prev_coords (np.ndarray): coordinates of the previous craters (in meters).

        Returns:
            tuple: coordinates of the craters, radii of the craters.
        """

        coords, radius = self.sample_from_poisson(region, l, r_minmax)
        return coords, radius

    def run_HC(
        self, region: BoundingBox, prev_coords: Tuple[np.ndarray, np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the hardcore Poisson process for all the densities and radius in order.
        A hardcore Poisson process is a Poisson process with a minimum distance between the points.
        Points that are too close to each other are rejected.

        Args:
            region (BoundingBox): region to sample from.
            prev_coords (tuple): coordinates of the previous craters.

        Returns:
            tuple: coordinates of the craters, radii of the craters.
        """

        if prev_coords[1].shape[0] == 0:
            prev_coords = None

        coords_to_save = []
        rads_to_save = []
        for d, r_minmax in zip(self.settings.densities, self.settings.radius):
            new_coords, new_radius = self.simulate_HC_poisson_process(region, d, r_minmax, prev_coords)
            coords_to_save.append(new_coords)
            rads_to_save.append(new_radius)
            if prev_coords is not None:
                prev_coords = (
                    np.concatenate([prev_coords[0], new_coords], axis=0),
                    np.concatenate([prev_coords[1], new_radius], axis=0),
                )
            else:
                prev_coords = (new_coords, new_radius)

        to_save = (
            np.concatenate(coords_to_save, axis=0),
            np.concatenate(rads_to_save, axis=0),
        )
        return to_save

    def run_NHC(
        self, region: BoundingBox, prev_coords: Tuple[np.ndarray, np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the Poisson process for all the densities and radius in order.
        Unlike the hardcore rejection, this method does not reject craters that are too close
        to each other.

        Args:
            region (BoundingBox): region to sample from.
            prev_coords (tuple): coordinates of the previous craters.

        Returns:
            tuple: coordinates of the craters, radii of the craters.
        """

        if prev_coords[1].shape[0] == 0:
            prev_coords = None

        coords_to_save = []
        rads_to_save = []
        for d, r_minmax in zip(self.settings.densities, self.settings.radius):
            new_coords, new_radius = self.simulate_poisson_process(region, d, r_minmax)
            coords_to_save.append(new_coords)
            rads_to_save.append(new_radius)
            if prev_coords is not None:
                prev_coords = (
                    np.concatenate([prev_coords[0], new_coords], axis=0),
                    np.concatenate([prev_coords[1], new_radius], axis=0),
                )
            else:
                prev_coords = (new_coords, new_radius)

        to_save = (
            np.concatenate(coords_to_save, axis=0),
            np.concatenate(rads_to_save, axis=0),
        )
        return to_save

    def run(
        self,
        region: BoundingBox,
        prev_coords: Tuple[np.ndarray, np.ndarray] = None,
        use_hc: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the Poisson process for all the densities and radius in order.

        Args:
            region (BoundingBox): region to sample from.
            prev_coords (Tuple[np.ndarray, np.ndarray]): coordinates of the previous craters.
            use_hc (bool): whether to use hardcore rejection or not.

        Returns:
            Tuple[np.ndarray, np.ndarray]: coordinates of the craters, radii of the craters.
        """

        if use_hc:
            return self.run_HC(region, prev_coords)
        else:
            return self.run_NHC(region, prev_coords)


@dataclasses.dataclass
class CraterGeneratorConf:
    """
    Args:
        profiles_path (str): path to the profiles.
        min_xy_ratio (float): minimum ratio between x and y.
        max_xy_ratio (float): maximum ratio between x and y.
        random_rotation (bool): whether to apply random rotation to the craters.
        seed (int): seed for the random number generator.
        num_unique_profiles (int): number of unique profiles to generate.
    """

    profiles_path: str = dataclasses.field(default_factory=str)
    min_xy_ratio: float = dataclasses.field(default_factory=float)
    max_xy_ratio: float = dataclasses.field(default_factory=float)
    random_rotation: bool = dataclasses.field(default_factory=bool)
    seed: int = dataclasses.field(default_factory=int)
    num_unique_profiles: int = dataclasses.field(default_factory=int)

    def __post_init__(self) -> None:
        assert type(self.profiles_path) is str, "profile_path must be a string"
        assert type(self.min_xy_ratio) is float, "min_xy_ratio must be a float"
        assert type(self.max_xy_ratio) is float, "max_xy_ratio must be a float"
        assert type(self.random_rotation) is bool, "random_rotation must be a boolean"
        assert type(self.seed) is int, "seed must be an integer"

        assert self.min_xy_ratio <= self.max_xy_ratio, "min_xy_ratio must be smaller than max_xy_ratio"
        assert self.min_xy_ratio > 0, "min_xy_ratio must be greater than 0"
        assert self.max_xy_ratio > 0, "max_xy_ratio must be greater than 0"
        assert self.min_xy_ratio <= 1, "min_xy_ratio must be smaller than 1"
        assert self.max_xy_ratio <= 1, "max_xy_ratio must be smaller than 1"


class CraterMetadataGenerator:
    """
    Generates metadata for craters allowing to cache a lightweight representation
    of the craters in the database. This allows to generate the craters on the fly
    without having to store the entire crater DEM in memory.
    """

    def __init__(self, settings: CraterGeneratorConf) -> None:
        """
        Args:
            settings (CraterGeneratorConf): settings for the crater generation.
        """

        self.settings = settings
        self.deformation_profiles = []
        self.marking_profiles = []
        self._rng = np.random.default_rng(self.settings.seed)
        self.build()

    def build(self) -> None:
        """
        Builds the crater generator by pre-generating the deformation and marking profiles.
        It also loads the half crater profiles from a pickle file. This is done so that the
        spline profiles are generated only once and cached inside the database.
        """

        logger.debug("Warming up crater generation...")
        self.generate_deformation_profiles()
        self.generate_marking_profiles()
        self.load_profiles()

    def get_crater_profiles(self) -> List[CubicSpline]:
        """
        Gets the half crater profiles. This is used by the database to store the profiles.

        Returns:
            List[CubicSpline]: list of half crater profiles.
        """

        return self.crater_profiles

    def get_deformation_profiles(self) -> List[CubicSpline]:
        """
        Gets the deformation profiles. This is used by the database to store the profiles.

        Returns:
            List[CubicSpline]: list of deformation profiles.
        """

        return self.deformation_profiles

    def get_marking_profiles(self) -> List[CubicSpline]:
        """
        Gets the marking profiles. This is used by the database to store the profiles.

        Returns:
            List[CubicSpline]: list of marking profiles.
        """

        return self.marking_profiles

    def generate_deformation_profiles(self) -> None:
        """
        Generates the deformation profiles for the craters.
        """

        logger.debug("Pre-generating crater deformation profiles")
        for i in range(self.settings.num_unique_profiles):
            deformation_profile = self._rng.uniform(0.95, 1, 9)
            deformation_profile = np.concatenate([deformation_profile, [deformation_profile[0]]], axis=0)
            tmp_x = np.linspace(0, 1, deformation_profile.shape[0])
            self.deformation_profiles.append(CubicSpline(tmp_x, deformation_profile, bc_type=((1, 0.0), (1, 0.0))))

    def generate_marking_profiles(self) -> None:
        """
        Generates the marking profiles for the craters.
        """

        logger.debug("Pre-generating crater marking profiles")
        for i in range(self.settings.num_unique_profiles):
            # Generates a profile to add marks that converges toward the center of the crater
            marks_profile = self._rng.uniform(0.0, 0.01, 45)
            marks_profile = np.concatenate([marks_profile, [marks_profile[0]]], axis=0)
            tmp_x = np.linspace(0, 1, marks_profile.shape[0])
            self.marking_profiles.append(CubicSpline(tmp_x, marks_profile, bc_type=((1, 0.0), (1, 0.0))))

    def load_profiles(self) -> None:
        """
        Loads the half crater spline profiles from a pickle file.
        """

        logger.debug("Loading crater profiles")
        with open(self.settings.profiles_path, "rb") as handle:
            self.crater_profiles = pickle.load(handle)

    def randomize_crater_parameters(self, coordinates: Tuple[float, float], radius: float) -> CraterMetadata:
        """
        Randomizes the parameters of a crater.

        Args:
            coordinates (Tuple[float, float]): coordinates of the crater.
            radius (float): radius of the crater.

        Returns:
            CraterData: data regarding the crater generation.
        """

        # Generates the metadata for a random crater
        crater_data = CraterMetadata()

        # Makes sure the matrix size is odd
        crater_data.radius = radius
        crater_data.coordinates = coordinates
        crater_data.deformation_spline_id = self._rng.integers(0, self.settings.num_unique_profiles, 1)[0]
        crater_data.marks_spline_id = self._rng.integers(0, self.settings.num_unique_profiles, 1)[0]
        crater_data.marks_intensity = self._rng.uniform(0, 1)
        crater_data.crater_profile_id = self._rng.integers(0, len(self.crater_profiles), 1)[0]
        crater_data.xy_deformation_factor = (
            self._rng.uniform(self.settings.min_xy_ratio, self.settings.max_xy_ratio),
            1.0,
        )
        crater_data.rotation = int(self._rng.uniform(0, 360))
        return crater_data

    def run(self, coordinates: np.ndarray, radius: np.ndarray) -> List[CraterMetadata]:
        """
        Generates the metadata for a set of craters whose coordinates and radius are known.

        Args:
            coordinates (np.ndarray): coordinates of the craters.

        Returns:
            List[CraterMetadata]: list of metadata for the craters.
        """

        metadatas = []
        for i in range(coordinates.shape[0]):
            metadatas.append(self.randomize_crater_parameters(coordinates[i], radius[i]))
        return metadatas

    def castMetadata(self, metadatas=List[CraterMetadata]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Casts the metadata to a numpy array. It only exctracts the coordinates and the radius.

        Args:
            metadatas (List[CraterMetadata]): list of metadata for the craters.

        Returns:
            Tuple[np.ndarray, np.ndarray]: coordinates of the craters, radii of the craters.
        """

        coordinates = np.array([metadata.coordinates for metadata in metadatas])
        radius = np.array([metadata.radius for metadata in metadatas])

        return coordinates, radius


@dataclasses.dataclass
class CraterSamplerConf:
    """
    Args:
        block_size (int): size of the blocks.
        crater_gen_cfg (CraterGeneratorConf): configuration for the crater generator.
        crater_dist_cfg (CraterDynamicDistributionConf): configuration for the crater distribution
    """

    block_size: int = 50
    crater_gen_cfg: CraterGeneratorConf = dataclasses.field(default_factory=dict)
    crater_dist_cfg: CraterDynamicDistributionConf = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        assert self.crater_gen_cfg is not None, "Crater generator configuration must be provided."
        assert self.crater_dist_cfg is not None, "Crater distribution configuration must be provided."

        self.crater_gen_cfg = CraterGeneratorConf(**self.crater_gen_cfg)
        self.crater_dist_cfg = CraterDynamicDistributionConf(**self.crater_dist_cfg)


class CraterSampler:
    """
    Class to sample craters on a DEM. It aggregates the crater distribution, the crater
    generation and the crater database to generate craters' metadata on the fly.
    """

    def __init__(self, crater_sampler_cfg: CraterSamplerConf, db: CraterDB) -> None:
        """
        Args:
            crater_sampler_cfg (CraterSamplerConf): configuration for the crater sampler.
            db (CraterDB): database to store the craters.
        """

        self.settings = crater_sampler_cfg
        self.crater_dist_gen = DynamicDistribute(self.settings.crater_dist_cfg)
        self.crater_metadata_gen = CraterMetadataGenerator(self.settings.crater_gen_cfg)
        self.crater_db = db
        self.add_profiles_to_db()

    def add_profiles_to_db(self):
        """
        Adds the profiles to the database.
        """

        self.crater_db.add_deformation_profiles(self.crater_metadata_gen.get_deformation_profiles())
        self.crater_db.add_marks_profiles(self.crater_metadata_gen.get_marking_profiles())
        self.crater_db.add_crater_profiles(self.crater_metadata_gen.get_crater_profiles())

    @staticmethod
    def compute_largest_rectangle(matrix: np.ndarray) -> Tuple[int, int]:
        """
        Computes the largest rectangle in a binary matrix.
        This is used to find the largest region without craters in the matrix.

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

    def sample_craters_by_block(self, block_coordinates: Tuple[int, int]) -> None:
        """
        Samples craters by block. This method is used to sample craters in a block
        that does not contain any craters.

        Args:
            block_coordinates (Tuple[int,int]): coordinates of the block.
        """

        # Checks if the block is valid
        self.crater_db.is_valid(block_coordinates)

        # Checks if the block already contains craters
        if self.crater_db.check_block_exists(block_coordinates):
            block = self.crater_db.get_block_data(block_coordinates)
        else:
            # If the block does not contain any craters, we collect the neighbors
            # this is done for the hardcore rejection process.
            block = self.crater_db.get_block_data_with_neighbors(block_coordinates)
            # Converts the metadatas into numpy arrays
            prev_coords = self.crater_metadata_gen.castMetadata(block)
            bb = BoundingBox(
                block_coordinates[0],
                block_coordinates[0] + self.settings.block_size,
                block_coordinates[1],
                block_coordinates[1] + self.settings.block_size,
            )
            coord, rad = self.crater_dist_gen.run(bb, prev_coords=prev_coords)
            block = self.crater_metadata_gen.run(coord, rad)
            self.crater_db.add_block_data(block, block_coordinates)

    def dissect_region_blocks(
        self, blocks: Tuple[np.ndarray, np.ndarray], region: BoundingBox
    ) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], List[Tuple[float, float]]]:
        """
        Dissects the region into blocks and returns the craters in each block.

        Args:
            blocks (Tuple[np.ndarray, np.ndarray]): coordinates and radii of the craters.
            region (BoundingBox): region to dissect.

        Returns:
            Tuple[List[Tuple[np.ndarray, np.ndarray]], List[Tuple[float, float]]]: list of
                craters in each block, coordinates of the blocks.
        """

        block_list = []
        coordinate_list = []
        for x in range(region.x_min, region.x_max, self.settings.block_size):
            for y in range(region.y_min, region.y_max, self.settings.block_size):
                idx = np.where(
                    (blocks[0][:, 0] >= x)
                    & (blocks[0][:, 0] < x + self.settings.block_size)
                    & (blocks[0][:, 1] >= y)
                    & (blocks[0][:, 1] < y + self.settings.block_size)
                )
                if len(idx[0]) > 0:
                    block_list.append((blocks[0][idx], blocks[1][idx]))
                    coordinate_list.append((x, y))

        return block_list, coordinate_list

    def sample_craters_by_region(self, region: BoundingBox) -> None:
        """
        Samples craters by region. This method is used to sample craters in a region.
        It will sample craters only in the blocks that do not contain any craters.
        To optimize the process, it finds the largest rectangle in the region that does not
        contain any craters and samples craters in this region. This process is repeated
        until the largest rectangle is smaller than a certain threshold. After that,
        the method samples craters in the remaining blocks that do not contain any craters.

        Args:
            region (BoundingBox): region to sample craters from.
        """

        # Process by regions until the largest rectangle is smaller or equal to 1 block
        process_by_regions = True
        while process_by_regions:
            matrix = self.crater_db.get_occupancy_matrix_within_region(region)
            # Computes the largest rectangle in the region that does not contain any craters
            area, coords = self.compute_largest_rectangle(matrix)
            # If the area is smaller or equal to 1 block, we stop the process
            # and fallback to the block sampling method
            if area <= 1:
                process_by_regions = False
                break

            # Region to sample craters from
            new_region = BoundingBox(
                region.x_min + coords[0][0] * self.settings.block_size,
                region.x_min + coords[0][1] * self.settings.block_size,
                region.y_min + coords[1][0] * self.settings.block_size,
                region.y_min + coords[1][1] * self.settings.block_size,
            )

            # Get all the existing craters in the region (should be empty except
            # for the neighbors). This is done for the hardcore rejection.
            blocks, _, _ = self.crater_db.get_blocks_within_region_with_neighbors(new_region)

            # Converts the metadatas into numpy arrays
            prev_coords = self.crater_metadata_gen.castMetadata(blocks)

            # Samples craters in the region
            new_blocks = self.crater_dist_gen.run(new_region, prev_coords=prev_coords)

            # Dissects the region into blocks and adds the craters to the database
            new_blocks_list, block_coordinates_list = self.dissect_region_blocks(new_blocks, region)
            for (coordinates, radius), block_coordinates in zip(new_blocks_list, block_coordinates_list):
                metadata = self.crater_metadata_gen.run(coordinates, radius)
                self.crater_db.add_block_data(metadata, block_coordinates)

        # If the largest rectangle is smaller or equal to 1 block, we sample craters
        # on a per block basis.
        coordinates = self.crater_db.get_missing_blocks(region)
        for coord in coordinates:
            self.sample_craters_by_block(coord)

    def display_block(self, coordinates: Tuple[float, float]):
        """
        Displays the craters in a block.

        Args:
            coordinates (Tuple[float,float]): coordinates of the block.
        """
        fig = plt.figure(figsize=(10, 10), dpi=300)
        if self.crater_db.check_block_exists(coordinates):
            block = self.crater_db.get_block_data(coordinates)
            coordinates, radius = self.crater_metadata_gen.castMetadata(block)
            ppm = 3000 / self.settings.block_size / 5
            plt.scatter(coordinates[:, 0], coordinates[:, 1], s=radius * ppm)
            plt.axis("equal")

    def display_block_set(self, coords: List[Tuple[float, float]]):
        """
        Displays a set of blocks. Each block is represented by a different color.

        Args:
            coords (List[Tuple[float,float]]): list of coordinates of the blocks.
        """
        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks = [self.crater_db.get_block_data(coord) for coord in coords]

        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i, block in enumerate(blocks):
            coordinates, radius = self.crater_metadata_gen.castMetadata(block)
            ppm = 3000 / self.settings.block_size / 5
            plt.scatter(coordinates[:, 0], coordinates[:, 1], s=radius * ppm, c=[colors[i]])

        plt.axis("equal")

    def display_region(self, region: BoundingBox):
        """
        Displays the craters in a region. Each block is represented by a different color.

        Args:
            region (BoundingBox): region to display.
        """

        fig = plt.figure(figsize=(10, 10), dpi=300)
        blocks, _, _ = self.crater_db.get_blocks_within_region(region)
        coordinates, radius = self.crater_metadata_gen.castMetadata(blocks)
        blocks, block_coordinates = self.dissect_region_blocks((coordinates, radius), region)
        color_interp = np.linspace(0, 1, len(blocks), endpoint=False)
        colors = [colorsys.hsv_to_rgb(i, 1, 1) for i in color_interp]
        for i in range(len(blocks)):
            coordinates, radius = blocks[i]
            ppm = 3000 / self.settings.block_size / 5
            plt.scatter(coordinates[:, 0], coordinates[:, 1], s=radius * ppm, c=[colors[i]])
        plt.axis("equal")
