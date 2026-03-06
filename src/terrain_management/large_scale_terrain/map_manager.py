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
import logging
import math
import yaml
import time
import os

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import (
    HighResDEMGen,
)
from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import (
    HighResDEMGenConf,
)


@dataclasses.dataclass
class DemInfo:
    """
    Args:
        size (tuple): size of the DEM in pixels
        pixel_size (tuple): size of the pixel in meters
        center_coordinates: (tuple): center coordinates of the DEM in meters
    """

    size: tuple = dataclasses.field(default_factory=tuple)
    pixel_size: tuple = dataclasses.field(default_factory=tuple)
    center_coordinates: tuple = dataclasses.field(default_factory=tuple)


@dataclasses.dataclass
class MapManagerConf:
    """
    Args:
        folder_path (str): path to the folder containing the DEMs.
        hrdem_settings (HighResDEMGenConf): settings for the high resolution DEM generation.
    """

    folder_path: str = dataclasses.field(default_factory=str)
    hrdem_settings: HighResDEMGenConf = dataclasses.field(default_factory=dict)

    def __post_init__(self):
        assert type(self.folder_path) is str, "folder_path must be a string"

        self.hrdem_settings = HighResDEMGenConf(**self.hrdem_settings)


class MapManager:
    """
    Class to manage the DEMs. It is reponsible for loading the DEMs and generating
    the high resolution DEM. It provides an interface for other classes to
    interract with the DEMs.
    """

    def __init__(
        self,
        map_manager_settings: MapManagerConf,
        is_simulation_alive: callable = lambda: True,
        close_simulation: callable = lambda: None,
    ) -> None:
        """
        Args:
            map_manager_settings (MapManagerConf): settings for the map manager.
            is_simulation_alive (callable): function to check if the simulation is alive.
            close_simulation (callable): function to close the simulation.
        """

        self.hr_dem_settings = map_manager_settings.hrdem_settings
        self.is_simulation_alive = is_simulation_alive
        self.close_simulation = close_simulation
        self.settings = map_manager_settings
        self.lr_dem = None

        self.fetch_pregenerated_lr_dems()

    def load(self, path: str) -> np.ndarray:
        map = np.load(path)
        return np.flip(map.T, axis=1)

    def fetch_pregenerated_lr_dems(self) -> None:
        """
        Fetches the pregenerated DEMs in the folder path.
        The folder path should contain a folder for each DEM with the DEM
        and the DEM info. It should look like this:
        folder_path
        ├── DEM1
        │   ├── dem.npy
        │   └── dem.yaml
        ├── DEM2
        │   ├── dem.npy
        │   └── dem.yaml
        ├── ...

        This method will load the paths to the DEMs and the DEM infos, not the DEMs
        themselves.

        dem.yaml should contain the following:
        size: Tuple[int, int]
        pixel_size: Tuple[float, float]
        center_coordinates: Tuple[float, float]
        """

        assert os.path.isdir(self.settings.folder_path), "Folder path does not exist"
        self.dem_paths = {}
        self.dem_infos = {}
        for folder in os.listdir(self.settings.folder_path):
            if os.path.isdir(os.path.join(self.settings.folder_path, folder)):
                dem_path = os.path.join(self.settings.folder_path, folder, "dem.npy")
                dem_info_path = os.path.join(self.settings.folder_path, folder, "dem.yaml")
                if os.path.exists(dem_path):
                    self.dem_paths[folder] = dem_path
                else:
                    logger.warn(
                        f"DEM {dem_path} does not exist. Expected to find dem.npy in the folder but could not find it."
                    )
                if os.path.exists(dem_info_path):
                    with open(dem_info_path, "r") as file:
                        self.dem_infos[folder] = DemInfo(**yaml.safe_load(file))
                else:
                    logger.warn(
                        f"DEM info {dem_info_path} does not exist. Expected to find dem.yaml in the folder but could not find it."
                    )
            else:
                logger.warn(f"Folder {folder} is not a directory.")

    def load_lr_dem_by_name(self, name: str) -> None:
        """
        Loads the low resolution DEM by name, and initializes the high resolution DEM generator.

        Args:
            name (str): name of the DEM to load.
        """

        if name in self.dem_paths:
            self.lr_dem = self.load(self.dem_paths[name])
            if name in self.dem_infos:
                self.lr_dem_info = self.dem_infos[name]
            else:
                raise ValueError(f"DEM info {name} does not exist in the folder path {self.settings.folder_path}")
        else:
            logger.warn(f"DEM {name} does not exist in the folder path {self.settings.folder_path}")

        # Override the source resolution
        lr_dem_res = self.lr_dem_info.pixel_size[0]
        self.hr_dem_settings.high_res_dem_cfg.source_resolution = lr_dem_res
        self.hr_dem_settings.interpolator_cfg.source_resolution = lr_dem_res
        self.hr_dem_gen = HighResDEMGen(
            self.lr_dem,
            self.hr_dem_settings,
            is_simulation_alive=self.is_simulation_alive,
            close_simulation=self.close_simulation,
        )

    def load_lr_dem_by_path(self, path: str) -> None:
        """
        Loads the low resolution DEM by path, and initializes the high resolution DEM generator.
        This method is used to load a DEM from a specific path outside of the folder path.

        Args:
            path (str): path to the DEM to load.
        """

        if os.path.exists(path):
            dem_path = os.path.join(path, "dem.npy")
            dem_info_path = os.path.join(path, "dem.yaml")
            if os.path.exists(dem_path) and os.path.exists(dem_info_path):
                self.lr_dem = self.load(dem_path)
                self.lr_dem_info = DemInfo(**yaml.load(open(dem_info_path, "r")))
            else:
                raise ValueError(f"DEM {dem_path} or DEM info {dem_info_path} does not exist in the folder path {path}")
        lr_dem_res = self.lr_dem_info.pixel_size[0]
        self.hr_dem_settings.high_res_dem_cfg.source_resolution = lr_dem_res
        self.hr_dem_settings.interpolator_cfg.source_resolution = lr_dem_res
        self.hr_dem_gen = HighResDEMGen(
            self.lr_dem,
            self.hr_dem_settings,
            is_simulation_alive=self.is_simulation_alive,
            close_simulation=self.close_simulation,
        )

    def load_lr_dem_by_id(self, id: int) -> None:
        """
        Loads the low resolution DEM by id, and initializes the high resolution DEM generator.
        This method is a legacy method and should not be used.

        Args:
            id (int): id of the DEM to load.
        """

        logger.warn("load_lr_dem_by_id is a legacy method and should not be used. Use load_lr_dem_by_name instead.")
        if id in list(range(len(self.dem_paths.keys()))):
            key = list(self.dem_paths.keys())[id]
            self.lr_dem = self.load(self.dem_paths[key])
            self.lr_dem_info = self.dem_infos[key]
        elif id == -1:
            self.generate_procedural_lr_dem()
        else:
            raise ValueError(
                f"id {id} is not in the range of the number of DEMs in the folder path {self.settings.folder_path}"
            )
        lr_dem_res = self.lr_dem_info.pixel_size[0]
        self.hr_dem_settings.high_res_dem_cfg.source_resolution = lr_dem_res
        self.hr_dem_settings.interpolator_cfg.source_resolution = lr_dem_res
        self.hr_dem_gen = HighResDEMGen(
            self.lr_dem,
            self.hr_dem_settings,
            is_simulation_alive=self.is_simulation_alive,
            close_simulation=self.close_simulation,
        )

    def generate_procedural_lr_dem(self):
        """
        Generates a procedural low resolution DEM.
        """

        raise NotImplementedError

    def get_lr_dem(self) -> np.ndarray:
        """
        Returns the low resolution DEM.

        Returns:
            np.ndarray: low resolution DEM.
        """

        return self.lr_dem

    def get_lr_dem_shape(self) -> Tuple[int, int]:
        """
        Returns the shape of the low resolution DEM.

        Returns:
            tuple: shape of the low resolution DEM.
        """
        return self.lr_dem.shape

    def get_lr_dem_res(self) -> float:
        """
        Returns the resolution of the low resolution DEM.

        Returns:
            float: resolution of the low resolution DEM.
        """

        return math.fabs(self.lr_dem_info.pixel_size[0])

    def get_lr_coordinates(self, coordinates: Tuple[float, float]) -> Tuple[float, float]:
        """
        Converts the global coordinates in meters to the low resolution DEM coordinates in meters.

        Args:
            coordinates (Tuple[float, floay]): coordinates in meters.

        Returns:
            Tuple[float, float]: coordinates in meters in the LR dem space.
        """

        x = coordinates[0] + self.get_lr_dem_res() * self.get_lr_dem_shape()[0] // 2 - self.get_lr_dem_res() / 2
        y = coordinates[1] + self.get_lr_dem_res() * self.get_lr_dem_shape()[1] // 2 - self.get_lr_dem_res() / 2
        return (x, y)

    def get_hr_dem(self) -> np.ndarray:
        """
        Returns the high resolution DEM.

        Returns:
            np.ndarray: high resolution DEM.
        """

        return self.hr_dem_gen.high_res_dem

    def get_hr_dem_shape(self) -> Tuple[int, int]:
        """
        Returns the shape of the high resolution DEM.

        Returns:
            tuple: shape of the high resolution DEM.
        """

        return self.hr_dem_gen.high_res_dem.shape

    def get_hr_dem_res(self) -> float:
        """
        Returns the resolution of the high resolution DEM.

        Returns:
            float: resolution of the high resolution DEM.
        """

        return self.hr_dem_gen.settings.resolution

    def get_hr_coordinates(self, coordinates: Tuple[float, float]) -> Tuple[float, float]:
        """
        Converts the global coordinates in meters to the high resolution DEM coordinates in meters.

        Args:
            coordinates (Tuple[float,float]): coordinates in meters.

        Returns:
            Tuple[float,float]: coordinates in meters in the hr dem space.
        """

        return self.hr_dem_gen.get_coordinates(coordinates)

    def get_hr_dem_mask(self) -> np.ndarray:
        """
        Returns the high resolution DEM mask.

        Returns:
            np.ndarray: high resolution DEM mask.
        """

        raise NotImplementedError

    def get_hr_dem_center_top_left(self) -> Tuple[float, float]:
        """
        Returns the coordinates of the top left corner of the center block of the high resolution DEM.

        Returns:
            Tuple[float, float]: coordinates of the high resolution DEM center's block top left corner.
        """

        return self.hr_dem_gen.get_center_top_left()

    def get_lr_dem_center_top_left(self) -> Tuple[float, float]:
        """
        Returns the coordinates of the top left corner of the center block of the high resolution DEM.

        Returns:
            Tuple[float, float]: coordinates of the high resolution DEM center's block top left corner.
        """

        height = self.lr_dem_info.size[0] * self.lr_dem_info.pixel_size[0]
        width = self.lr_dem_info.size[1] * abs(self.lr_dem_info.pixel_size[1])

        top = height / 2 - self.settings.hrdem_settings.high_res_dem_cfg.block_size / 2
        left = width / 2 - self.settings.hrdem_settings.high_res_dem_cfg.block_size / 2

        return self.hr_dem_gen.get_center_top_left()

    def update_hr_dem(self, coordinates: Tuple[float, float]) -> bool:
        """
        Updates the high resolution DEM around the coordinates.
        If conditions are met, this will trigger an update of the high resolution DEM.

        Args:
            coordinates (Tuple[float,float]): coordinates in meters.

        Returns:
            bool: True if the high resolution DEM was updated, False otherwise.
        """

        return self.hr_dem_gen.update_high_res_dem(coordinates)

    def get_height(self, coordinates: Tuple[float, float]) -> float:
        """
        Returns the height at the coordinates in meters.

        Args:
            coordinates (Tuple[float,float]): coordinates in meters.

        Returns:
            float: height at the coordinates in meters.
        """

        return self.hr_dem_gen.get_height(coordinates)

    def get_normal(self, coordinates: Tuple[float, float]) -> np.ndarray:
        """
        Returns the normal at the coordinates.

        Args:
            coordinates (Tuple[float,float]): coordinates in meters.

        Returns:
            np.ndarray: normal vector to the DEM surface at the given coordinates.
        """

        return self.hr_dem_gen.get_normal(coordinates)

    def initialize_hr_dem(self, coordinates: Tuple[float, float]) -> None:
        """
        Initializes the high resolution DEM around the coordinates.
        Calling this method will block until the high resolution DEM is updated.

        Args:
            coordinates (Tuple[float,float]): coordinates in meters.
        """

        logger.info("Initializing HR DEM")
        start = time.time()
        self.update_hr_dem(coordinates)
        while not self.is_hr_dem_updated():
            time.sleep(0.1)
        end = time.time()
        logger.info(f"HR DEM initialized in {end - start} seconds")

    def is_hr_dem_updated(self) -> bool:
        """
        Returns True if the high resolution DEM is updated, False otherwise.

        Returns:
            bool: True if the high resolution DEM is updated, False otherwise.
        """

        return self.hr_dem_gen.is_map_done()

    def get_hr_map_current_block_coordinates(self):
        return self.hr_dem_gen.get_current_block_coordinates()

    def get_lat_lon(self) -> Tuple[float, float]:
        return (self.lr_dem_info.center_coordinates[1], self.lr_dem_info.center_coordinates[0])


if __name__ == "__main__":
    HRDEMCfg_D = {
        "num_blocks": 4,
        "block_size": 50,
        "pad_size": 10.0,
        "max_blocks": int(1e7),
        "seed": 42,
        "resolution": 0.05,
        "z_scale": 1.0,
        "source_resolution": 5.0,
        "resolution": 0.05,
        "interpolation_padding": 2,
        "generate_craters": True,
    }
    CWMCfg_D = {
        "num_workers": 8,
        "input_queue_size": 400,
        "output_queue_size": 16,
        "worker_queue_size": 2,
    }
    IWMCfg_D = {
        "num_workers": 1,
        "input_queue_size": 400,
        "output_queue_size": 30,
        "worker_queue_size": 200,
    }
    CraterDBCfg_D = {
        "block_size": 50,
        "max_blocks": 7,
        "save_to_disk": False,
        "write_to_disk_interval": 100,
    }
    CGCfg_D = {
        "profiles_path": "assets/Terrains/crater_spline_profiles.pkl",
        "min_xy_ratio": 0.85,
        "max_xy_ratio": 1.0,
        "random_rotation": True,
        "seed": 42,
        "num_unique_profiles": 10000,
    }
    CDDCfg_D = {
        "densities": [0.025, 0.05, 0.5],
        "radius": [[1.5, 2.5], [0.75, 1.5], [0.25, 0.5]],
        "num_repeat": 1,
        "seed": 42,
    }
    CraterSamplerCfg_D = {
        "block_size": 50,
        "crater_gen_cfg": CGCfg_D,
        "crater_dist_cfg": CDDCfg_D,
    }
    CraterBuilderCfg_D = {
        "block_size": 50,
        "pad_size": 10.0,
        "resolution": 0.05,
        "z_scale": 1.0,
    }
    ICfg = {
        "source_resolution": 5.0,
        "target_resolution": 0.05,
        "source_padding": 2,
        "method": "bicubic",
    }
    HRDEMGenCfg_D = {
        "high_res_dem_cfg": HRDEMCfg_D,
        "crater_db_cfg": CraterDBCfg_D,
        "crater_sampler_cfg": CraterSamplerCfg_D,
        "crater_builder_cfg": CraterBuilderCfg_D,
        "interpolator_cfg": ICfg,
        "crater_worker_manager_cfg": CWMCfg_D,
        "interpolator_worker_manager_cfg": IWMCfg_D,
    }

    hrdem_settings = HighResDEMGenConf(**HRDEMGenCfg_D)

    MMCfg_D = {
        "folder_path": "assets/Terrains/SouthPole",
        "lr_dem_name": "crater",
    }

    mm_settings = MapManagerConf(**MMCfg_D)
    from matplotlib import pyplot as plt

    MM = MapManager(hrdem_settings, mm_settings)
    MM.load_lr_dem_by_name("NPD_final_adj_5mpp_surf")
    MM.initialize_hr_dem((0, 0))
    plt.figure()
    plt.imshow(MM.hr_dem_gen.high_res_dem, cmap="jet")
    plt.figure()
    plt.imshow(MM.lr_dem, cmap="jet")
    plt.figure()
    plt.imshow(MM.lr_dem[1950:2100, 1950:2100], cmap="jet")
    plt.show()
    MM.hr_dem_gen.shutdown()
