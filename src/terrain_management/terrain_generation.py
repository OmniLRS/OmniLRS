__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from scipy.interpolate import CubicSpline
from matplotlib import pyplot as plt
from scipy.ndimage import rotate
from typing import List, Tuple
import dataclasses
import numpy as np
import datetime
import pickle
import cv2

from src.configurations.procedural_terrain_confs import (
    CraterGeneratorConf,
    CraterDistributionConf,
    BaseTerrainGeneratorConf,
    DeformationEngineConf,
    MoonYardConf,
)

from src.terrain_management.deformation_engine import DeformationEngine


@dataclasses.dataclass
class CraterData:
    deformation_spline: CubicSpline = None
    marks_spline: CubicSpline = None
    marks_intensity: float = 0
    size: int = 0
    crater_profile_id: int = 0
    xy_deformation_factor: Tuple[float, float] = (0, 0)
    rotation: float = 0
    coord: Tuple[int, int] = (0, 0)


class CraterGenerator:
    """
    Generates craters DEM from a set of spline profiles and randomizes their 2D appearance
    """

    def __init__(
        self,
        cfg: CraterGeneratorConf,
    ) -> None:
        """
        Args:
            profiles_path (str): path to the pickle file containing the spline profiles.
            seed (int, optional): random seed. Defaults to 42.
            min_xy_ratio (float, optional): minimum xy ratio of the crater. Defaults to 0.85.
            max_xy_ratio (float, optional): maximum xy ratio of the crater. Defaults to 1.
            resolution (float, optional): resolution of the DEM (in meters per pixel). Defaults to 0.01.
            pad_size (int, optional): size of the padding to add to the DEM. Defaults to 500.
            random_rotation (bool, optional): whether to randomly rotate the craters. Defaults to True.
            z_scale (float, optional): scale of the craters. Defaults to 1."""

        self._profiles_path = cfg.profiles_path
        self._resolution = cfg.resolution
        self._min_xy_ratio = cfg.min_xy_ratio
        self._max_xy_ratio = cfg.max_xy_ratio
        self._z_scale = cfg.z_scale
        self._random_rotation = cfg.random_rotation
        self._pad_size = cfg.pad_size
        self._profiles = None
        self._rng = np.random.default_rng(cfg.seed)

        self.loadProfiles()

    def loadProfiles(self) -> None:
        """
        Loads the half crater spline profiles from a pickle file."""

        with open(self._profiles_path, "rb") as handle:
            self._profiles = pickle.load(handle)

    def sat_gaussian(self, x: np.ndarray, mu1: float, mu2: float, std: float) -> np.ndarray:
        """
        Saturates a gaussian function to its maximum between mu1 and mu2 with a standard deviation of std.

        Args:
            x (np.ndarray): input array.
            mu1 (float): gaussian mu lower bound.
            mu2 (float): gaussian mu upper bound.
            std (float): standard deviation.

        Returns:
            np.ndarray: saturated gaussian."""

        shape = x.shape
        x = x.flatten()
        x[x < mu1] = np.exp(-0.5 * ((x[x < mu1] - mu1) / std) ** 2)
        x[x > mu2] = np.exp(-0.5 * ((x[x > mu2] - mu2) / std) ** 2)
        x[(x >= mu1) & (x <= mu2)] = 1.0
        x = x / (std * np.sqrt(2 * np.pi))
        x = x.reshape(shape)
        return x

    def centeredDistanceMatrix(self, crater_data: CraterData) -> np.ndarray:
        """
        Generates a distance matrix centered at the center of the matrix.

        Args:
            n (int): size of the matrix

        Returns:
            np.ndarry: distance matrix."""

        # Generates the deformation matrix
        m = np.zeros([crater_data.size, crater_data.size])
        x, y = np.meshgrid(np.linspace(-1, 1, crater_data.size), np.linspace(-1, 1, crater_data.size))
        theta = np.arctan2(y, x)
        fac = crater_data.deformation_spline(theta / (2 * np.pi) + 0.5)

        # Generates the marks matrix
        marks = crater_data.marks_spline(theta / (2 * np.pi) + 0.5) * crater_data.size / 2 * crater_data.marks_intensity

        # Generates the distance matrix
        x, y = np.meshgrid(range(crater_data.size), range(crater_data.size))
        m = np.sqrt(
            ((x - (crater_data.size / 2) + 1) * 1 / crater_data.xy_deformation_factor[0]) ** 2
            + ((y - (crater_data.size / 2) + 1) * 1 / crater_data.xy_deformation_factor[1]) ** 2
        )

        # Deforms the distance matrix
        m = m * fac

        # Smoothes the marks such that they are not present on the outer edge of the crater
        sat = self.sat_gaussian(
            m,
            0.15 * crater_data.size / 2,
            0.45 * crater_data.size / 2,
            0.05 * crater_data.size / 2,
        )
        sat = (sat - sat.min()) / (sat.max() - sat.min())

        # Adds the marks to the distance matrix
        m = m + marks * sat

        # Rotate the matrix
        m = rotate(m, crater_data.rotation, reshape=False, cval=crater_data.size / 2)

        # Saturate the matrix such that the distance is not greater than the radius of the crater
        m[m > crater_data.size / 2] = crater_data.size / 2

        return m

    def applyProfile(self, distance: np.ndarray, crater_data: CraterData) -> np.ndarray:
        """
        Applies a profile to the distance matrix.

        Args:
            index (int): index of the profile to apply.
            distance (np.ndarray): distance matrix.
            size (int): size of the matrix.

        Returns:
            np.ndarray: crater DEM."""

        crater = self._profiles[crater_data.crater_profile_id](2 * distance / crater_data.size)
        return crater

    def randomizeCraterParameters(self, index: int, size: int) -> CraterData:
        """
        Randomizes the parameters of a crater.

        Args:
            index (int): index of the profile to use.
            size (int): size of the crater.

        Returns:
            CraterData: data regarding the crater generation."""

        # Generates the metadata for a random crater
        crater_data = CraterData()

        # Makes sure the matrix size is odd
        size = size + ((size % 2) == 0)
        crater_data.size = size

        # Generates a profile to deform the crater
        deformation_profile = self._rng.uniform(0.95, 1, 9)
        deformation_profile = np.concatenate([deformation_profile, [deformation_profile[0]]], axis=0)
        tmp_x = np.linspace(0, 1, deformation_profile.shape[0])
        crater_data.deformation_spline = CubicSpline(tmp_x, deformation_profile, bc_type=((1, 0.0), (1, 0.0)))

        # Generates a profile to add marks that converges toward the center of the crater
        marks_profile = self._rng.uniform(0.0, 0.01, 45)
        marks_profile = np.concatenate([marks_profile, [marks_profile[0]]], axis=0)
        tmp_x = np.linspace(0, 1, marks_profile.shape[0])
        crater_data.marks_spline = CubicSpline(tmp_x, marks_profile, bc_type=((1, 0.0), (1, 0.0)))
        crater_data.marks_intensity = self._rng.uniform(0, 1)

        # XY deformation factor
        sx = self._rng.uniform(self._min_xy_ratio, self._max_xy_ratio)
        sy = 1.0
        crater_data.xy_deformation_factor = (sx, sy)

        # Random rotation
        crater_data.rotation = int(self._rng.uniform(0, 360))

        # Index of the profile
        if index == -1:
            index = self._rng.integers(0, len(self._profiles), 1)[0]
            pass
        elif index < len(self._profiles):
            pass
        else:
            raise ValueError("Unknown profile")
        crater_data.crater_profile_id = index
        return crater_data

    def generateCrater(
        self, size: int = None, index: int = -1, crater_data: CraterData = None
    ) -> Tuple[np.ndarray, CraterData]:
        """
        Generates a crater DEM.

        Args:
            size (float): size of the crater.
            index (int): index of the profile to use.

        Returns:
            tuple: crater DEM, the whole the of the data regarding the crater generation.
        """

        if crater_data is None:
            crater_data = self.randomizeCraterParameters(index, size)

        distance = self.centeredDistanceMatrix(crater_data)

        crater = self.applyProfile(distance, crater_data) * crater_data.size / 2.0 * self._z_scale * self._resolution
        return crater, crater_data

    def generateCraters(
        self,
        DEM: np.ndarray,
        coords: np.ndarray = None,
        radius: np.ndarray = None,
        craters_data: List[CraterData] = None,
    ) -> Tuple[np.ndarray, np.ndarray, List[CraterData]]:
        """
        Generates a DEM with craters.

        Args:
            DEM (np.ndarray): DEM to add craters to.
            coords (np.ndarray): coordinates of the craters (in meters).

        Returns:
            np.ndarray: unpadded DEM with craters, and matching mask."""

        # Creates a padded DEM and mask
        DEM_padded = np.zeros((self._pad_size * 2 + DEM.shape[0], self._pad_size * 2 + DEM.shape[1]))
        mask_padded = np.ones_like(DEM_padded)
        DEM_padded[self._pad_size : -self._pad_size, self._pad_size : -self._pad_size] = DEM

        # Creates a cache to store craters data
        if craters_data is None:
            craters_data = []

        # Adds the craters to the DEM
        if len(craters_data) == 0:
            for coord, rad in zip(coords, radius):
                rad = int(rad * 2 / self._resolution)
                coord_s = coord / self._resolution
                c, crater_data = self.generateCrater(int(rad))
                crater_data.coord = (coord[0], coord[1])
                coord2 = (coord_s + self._pad_size).astype(np.int64)
                coord = (coord_s - crater_data.size / 2 + self._pad_size).astype(np.int64)
                DEM_padded[
                    coord[0] : coord[0] + crater_data.size,
                    coord[1] : coord[1] + crater_data.size,
                ] += c
                mask_padded = cv2.circle(
                    mask_padded,
                    (coord2[1], coord2[0]),
                    int(crater_data.size / 4),
                    0,
                    -1,
                )
                craters_data.append(crater_data)
        else:
            for crater_data in craters_data:
                rad = int(crater_data.size * 2 / self._resolution)
                coord = coord / self._resolution
                c, crater_data = self.generateCrater(crater_data)
                coord2 = (coord + self._pad_size).astype(np.int64)
                coord = (coord - crater_data.size / 2 + self._pad_size).astype(np.int64)
                DEM_padded[
                    coord[0] : coord[0] + crater_data.size,
                    coord[1] : coord[1] + crater_data.size,
                ] += c
                mask_padded = cv2.circle(
                    mask_padded,
                    (coord2[1], coord2[0]),
                    int(crater_data.size / 4),
                    0,
                    -1,
                )

        mask_padded[: self._pad_size + 1, :] = 0
        mask_padded[:, : self._pad_size + 1] = 0
        mask_padded[-self._pad_size - 1 :, :] = 0
        mask_padded[:, -self._pad_size - 1 :] = 0
        return (
            DEM_padded[self._pad_size : -self._pad_size, self._pad_size : -self._pad_size],
            mask_padded[self._pad_size : -self._pad_size, self._pad_size : -self._pad_size],
            craters_data,
        )


class Distribute:
    """
    Distributes craters on a DEM using a Poisson process with hardcore rejection."""

    def __init__(
        self,
        cfg: CraterDistributionConf,
    ) -> None:
        """
        Args:
            x_size (float, optional): size of the DEM in the x direction (in meters). Defaults to 10.
            y_size (float, optional): size of the DEM in the y direction (in meters). Defaults to 10.
            densities (float, optional): densities of the craters (in units per square meters). Defaults to [0.25,1.5,5].
            radius (list, optional): min and max radii of the craters (in meters). Defaults to [(1.5,2.5),(0.75,1.5),(0.25,0.5)].
            num_repeat (int, optional): number of times to repeat the hardcore rejection. Defaults to 0.
            seed (int, optional): random seed. Defaults to 42."""

        self._x_max = cfg.x_size
        self._y_max = cfg.y_size
        self._densities = cfg.densities
        self._radius = cfg.radius
        self._area = self._x_max * self._y_max
        self._num_repeat = cfg.num_repeat
        self._rng = np.random.default_rng(cfg.seed)

    def sampleFromPoisson(self, l: float, r_minmax: Tuple[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Samples from a Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).

        Returns:
            tuple: coordinates and radius of the craters"""

        num_points = self._rng.poisson(self._area * l)
        radius = self._rng.uniform(r_minmax[0], r_minmax[1], num_points)
        x_coords = self._rng.uniform(0, self._x_max, num_points)
        y_coords = self._rng.uniform(0, self._y_max, num_points)
        return np.stack([x_coords, y_coords]).T, radius

    def hardcoreRejection(self, coords: np.ndarray, radius: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Performs hardcore rejection on the craters.

        Args:
            coords (np.ndarray): coordinates of the craters (in meters).
            radius (np.ndarray): radii of the craters (in meters).

        Returns:
            tuple: coordinates of the craters (in meters)."""

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

    def checkPrevious(
        self, new_coords: np.ndarray, radius: np.ndarray, prev_coords: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Checks if the new craters are not in the previous craters.

        Args:
            new_coords (np.ndarray): coordinates of the new craters.
            radius (np.ndarray): radii of the new craters.
            prev_coords (np.ndarray): coordinates of the previous craters.

        Returns:
            tuple: the coordinates of the new craters, the radii of the new craters."""

        boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        if prev_coords is None:
            boole_keep = np.ones(new_coords.shape[0], dtype=bool)
        else:
            for i in range(prev_coords[0].shape[0]):
                dist_tmp = np.linalg.norm(prev_coords[0][i] - new_coords, axis=-1)
                in_disk = (dist_tmp < prev_coords[1][i]) & (dist_tmp > 0)
                boole_keep[in_disk] = False
        return new_coords[boole_keep], radius[boole_keep]

    def simulateHCPoissonProcess(
        self, l: float, r_minmax: Tuple[float], prev_coords: np.ndarray = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Simulates a hardcore Poisson process.

        Args:
            l (float): density of the Poisson process (in units per square meters).
            r_minmax (tuple): minimum and maximum radius of the craters (in meters).
            prev_coords (np.ndarray): coordinates of the previous craters (in meters).

        Returns:
            tuple: coordinates of the craters, radii of the craters."""

        coords, radius = self.sampleFromPoisson(l, r_minmax)
        for _ in range(self._num_repeat):
            coords, radius = self.hardcoreRejection(coords, radius)
            new_coords, new_radius = self.sampleFromPoisson(l, r_minmax)
            coords = np.concatenate([coords, new_coords])
            radius = np.concatenate([radius, new_radius])
            self.checkPrevious(coords, radius, prev_coords)
        coords, radius = self.hardcoreRejection(coords, radius)
        coords, radius = self.checkPrevious(coords, radius, prev_coords)
        return coords, radius

    def run(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Runs the hardcore Poisson process for all the densities and radius in order.

        Returns:
            tuple: coordinates of the craters, radii of the craters."""

        prev_coords = None
        for d, r_minmax in zip(self._densities, self._radius):
            new_coords, new_radius = self.simulateHCPoissonProcess(d, r_minmax, prev_coords)
            if prev_coords is not None:
                prev_coords = (
                    np.concatenate([prev_coords[0], new_coords], axis=0),
                    np.concatenate([prev_coords[1], new_radius], axis=0),
                )
            else:
                prev_coords = (new_coords, new_radius)
        return prev_coords


class BaseTerrainGenerator:
    """
    Generates a random terrain DEM."""

    def __init__(
        self,
        cfg: BaseTerrainGeneratorConf,
    ) -> None:
        """
        Args:
            x_size (float, optional): size of the DEM in the x direction (in meters). Defaults to 10.
            y_size (float, optional): size of the DEM in the y direction (in meters). Defaults to 10.
            resolution (float, optional): resolution of the DEM (in meters per pixel). Defaults to 0.01.
            max_elevation (float, optional): maximum elevation of the DEM (in meters). Defaults to 0.5.
            min_elevation (float, optional): minimum elevation of the DEM (in meters). Defaults to -0.25.
            seed (int, optional): random seed. Defaults to 42.
            z_scale (float, optional): scale of the DEM. Defaults to 50."""

        self._min_elevation = cfg.min_elevation
        self._max_elevation = cfg.max_elevation
        self._x_size = int(cfg.x_size / cfg.resolution)
        self._y_size = int(cfg.y_size / cfg.resolution)
        self._DEM = np.zeros((self._x_size, self._y_size), dtype=np.float32)
        self._rng = np.random.default_rng(cfg.seed)
        self._z_scale = cfg.z_scale

    def generateRandomTerrain(self, is_lab: bool = False, is_yard: bool = False) -> np.ndarray:
        """
        Generates a random terrain DEM. With some of its borders at 0 height
        to align with the catawalks in the lab.

        Args:
            is_lab (bool): whether the DEM is in a lab or not.
            is_yard (bool): whether the DEM is in a yard or not.

        Returns:
            DEM (np.ndarray): random terrain DEM."""

        # Generate low frequency noise to simulate large scale terrain features.
        if is_lab:
            lr_noise = np.zeros((4, 4))
            lr_noise[:-1, 1:] = self._rng.uniform(self._min_elevation, self._max_elevation, [3, 3])
        elif is_yard:
            lr_noise = np.zeros((7, 7))
            lr_noise[1:-1, 1:-1] = self._rng.uniform(self._min_elevation, self._max_elevation, [5, 5])
        else:
            lr_noise = self._rng.uniform(self._min_elevation, self._max_elevation, [4, 4])
        hr_noise = cv2.resize(lr_noise, (self._y_size, self._x_size), interpolation=cv2.INTER_CUBIC)
        self._DEM += hr_noise
        # Generate high frequency noise to simulate small scale terrain features.
        lr_noise = self._rng.uniform(self._min_elevation * 0.01, self._max_elevation * 0.01, [100, 100])
        hr_noise = cv2.resize(lr_noise, (self._y_size, self._x_size), interpolation=cv2.INTER_CUBIC)
        self._DEM += hr_noise
        # Normalize the DEM between 0 and 1 and scale it to the desired elevation range.
        # self._DEM = (self._DEM - self._DEM.min()) / (self._DEM.max() - self._DEM.min())
        # Scale the DEM to the desired elevation range.
        # self._DEM = self._DEM * (self._max_elevation - self._min_elevation) + self._min_elevation
        return self._DEM * self._z_scale


class GenerateProceduralMoonYard:
    """
    Generates a random terrain DEM with craters."""

    def __init__(
        self,
        moon_yard: MoonYardConf,
    ) -> None:
        """
        Args:
            crater_profiles_path (str): path to the pickle file containing the spline profiles.
            x_size (float, optional): size of the DEM in the x direction (in meters). Defaults to 10.
            y_size (float, optional): size of the DEM in the y direction (in meters). Defaults to 6.5.
            resolution (float, optional): resolution of the DEM (in meters per pixel). Defaults to 0.01.
            max_elevation (float, optional): maximum elevation of the DEM (in meters). Defaults to 0.25.
            min_elevation (float, optional): minimum elevation of the DEM (in meters). Defaults to -0.025.
            z_scale (float, optional): scale of the DEM. Defaults to 1.
            pad (int, optional): size of the padding to add to the DEM. Defaults to 500.
            num_repeat (float, optional): number of times to repeat the hardcore rejection. Defaults to 0.
            densities (float, optional): densities of the craters (in units per square meters). Defaults to [0.025,0.05,0.5].
            radius (list, optional): min and max radii of the craters (in meters). Defaults to [(1.5,2.5),(0.75,1.5),(0.25,0.5)].
            is_lab (bool, optional): whether the DEM is in a lab or not. Defaults to False.
            is_yard (bool, optional): whether the DEM is in a yard or not. Defaults to False.
            seed (int, optional): random seed. Defaults to 42."""

        self.T = BaseTerrainGenerator(moon_yard.base_terrain_generator)
        self.D = Distribute(moon_yard.crater_distribution)
        self.G = CraterGenerator(moon_yard.crater_generator)
        self.is_lab = moon_yard.is_lab
        self.is_yard = moon_yard.is_yard

        self.DE = DeformationEngine(moon_yard.deformation_engine)
        self._dem_init = None
        self._mask = None
        self._dem_delta = None
        self._num_pass = None
        self._depth_memory = None
        self._trace_memory = None

    def randomize(self) -> np.ndarray:
        """
        Generates a random terrain DEM with craters.

        Returns:
            tuple: random terrain DEM with craters, mask of the craters, and the data regarding the craters generation.
        """

        DEM = self.T.generateRandomTerrain(is_lab=self.is_lab, is_yard=self.is_yard)
        coords, radius = self.D.run()
        DEM, mask, craters_data = self.G.generateCraters(DEM, coords, radius)
        self._dem_init = DEM
        self._dem_delta = np.zeros_like(DEM)
        self._mask = mask
        self._num_pass = np.zeros_like(mask)
        self._depth_memory = np.zeros_like(mask)
        self._trace_memory = np.zeros_like(mask)
        return DEM, mask, craters_data

    def augment(self, DEM: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        Generates a random terrain DEM with craters.

        Returns:
            tuple: random terrain DEM with craters, mask of the craters, and the data regarding the craters generation.
        """
        coords, radius = self.D.run()
        DEM, mask_, craters_data = self.G.generateCraters(DEM, coords, radius)
        mask = mask * mask_
        self._dem_init = DEM
        self._dem_delta = np.zeros_like(DEM)
        self._mask = mask
        self._num_pass = np.zeros_like(mask)
        self._depth_memory = np.zeros_like(mask)
        self._trace_memory = np.zeros_like(mask)
        return DEM, mask, craters_data

    def register_terrain(self, DEM: np.ndarray, mask: np.ndarray):
        """
        Register dem and mask to instance variables.
        """
        self._dem_init = DEM
        self._dem_delta = np.zeros_like(DEM)
        self._mask = mask
        self._num_pass = np.zeros_like(mask)
        self._depth_memory = np.zeros_like(mask)
        self._trace_memory = np.zeros_like(mask)

    def deform(
        self, world_positions: np.ndarray, world_orientations: np.ndarray, linear_velocities: np.ndarray, angular_velocities: np.ndarray, contact_forces: np.ndarray
    ) -> np.ndarray:
        """
        Add vertical deformation to terrain DEM.
        Args:
            world_positions (numpy.ndarray): world positions of robot links (N, 3)
            world_orientations (numpy.ndarray): world orientations of robot links (N, 4)
            contact_forces(numpy.ndarray): contact forces on robot links (N, 3)
        """
        self._dem_delta, self._num_pass, self._depth_memory, self._trace_memory = self.DE.deform(
            self._dem_delta, self._num_pass, self._depth_memory, self._trace_memory, world_positions, world_orientations, linear_velocities, angular_velocities,contact_forces[:, 2]
        )
        return self._dem_init + self._dem_delta, self._mask


if __name__ == "__main__":
    craters_profiles_path = "crater_spline_profiles.pkl"
    start = datetime.datetime.now()
    G = GenerateProceduralMoonYard(craters_profiles_path)
    DEMs, mask, craters_data = [G.randomize() for i in range(4)]
    end = datetime.datetime.now()

    for DEM, mask in DEMs:
        plt.figure()
        plt.imshow(DEM, cmap="jet")
        plt.figure()
        plt.imshow(mask, cmap="jet")
    plt.show()
