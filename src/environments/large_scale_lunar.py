__author__ = "Antoine Richard, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from scipy.spatial.transform import Rotation as SSTR
from typing import Dict, Tuple
import numpy as np

from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.static_assets_manager import StaticAssetsManager
from src.environments.stellar_engine_env_mixin import StellarEngineEnvMixin
from src.terrain_management.large_scale_terrain_manager import LargeScaleTerrainManager
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.environments import LargeScaleTerrainConf
from src.environments.base_env import BaseEnv
from src.robots.robot import RobotManager


class LargeScaleController(BaseEnv, StellarEngineEnvMixin):
    """
    This class is used to control the environment's interactive elements.
    """

    def __init__(
        self,
        mode:SimulatorMode = SimulatorMode.ROS2,
        large_scale_terrain: LargeScaleTerrainConf = None,
        stellar_engine_settings: StellarEngineConf = None,
        sun_settings: SunConf = None,
        is_simulation_alive: callable = lambda: True,
        static_assets_settings: Dict = None,
        monitoring_cameras_settings: Dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the env controller. This class is used to control the env interactive elements.
        Including:
            - Sun position, intensity, radius, color.

        Args:
            env_settings (LargeScaleTerrainConf): The settings of the lab.
            stellar_engine_settings (StellarEngineConf): The settings of the stellar engine.
            sun_settings (SunConf): The settings of the sun.
            is_simulation_alive (callable): function to check if the simulation is alive.
            **kwargs: Arbitrary keyword arguments.
        """

        super().__init__(mode, **kwargs)
        self.scene_name = "/LargeScaleLunar"
        self.stage_settings = large_scale_terrain
        self.is_simulation_alive = is_simulation_alive

        if static_assets_settings:
            self.SAM = StaticAssetsManager(static_assets_settings)

        if monitoring_cameras_settings and monitoring_cameras_settings["enabled"]:
            self.MCM = MonitoringCamerasManager(self._mode, monitoring_cameras_settings)

        self._sun_settings = sun_settings
        self.init_stellar_engine(stage_settings=self.stage_settings, 
                                  stellar_engine_settings=stellar_engine_settings)

    def build_scene(self) -> None:
        """
        Builds the scene.
        """

        # Creates an empty xform with the name lunaryard
        large_scale = self.stage.DefinePrim(self.scene_name, "Xform")

        self.create_sun(self._sun_settings)
        self.create_earth()

    def update(self) -> None:
        """
        Updates the environment.
        """

        # print position of prim to track
        p, _ = self.pose_tracker()
        self.LSTM.update_visual_mesh((p[0], p[1]))

    def monitor_thread_is_alive(self) -> None:
        return self.LSTM.map_manager.hr_dem_gen.monitor_thread.thread.is_alive()

    def get_wait_for_threads(self) -> None:
        return [self.LSTM.map_manager.hr_dem_gen.monitor_thread.thread.join]

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        self.build_scene()
        # Instantiates the terrain manager
        self.LSTM = LargeScaleTerrainManager(self.stage_settings, is_simulation_alive=self.is_simulation_alive)
        self.LSTM.build()
        # Sets the sun using the stellar engine if enabled
        if self.enable_stellar_engine:
            self.SE.set_lat_lon(*self.LSTM.get_lat_lon())

        if self.SAM:
            self.SAM.spawn(get_height_func=self.LSTM.get_height_local)

        if self.MCM:
            self.MCM.spawn()

    def add_robot_manager(self, robotManager: RobotManager) -> None:
        """
        Adds the robot manager to the environment.

        Args:
            robotManager (RobotManager): The robot manager to be added.
        """

        super().add_robot_manager(robotManager)
        self.pose_tracker = self.robotManager.robot.get_pose     # is later called as a function, thus now does not have ()

    # ==============================================================================
    # Terrain info
    # ==============================================================================

    def get_height_and_normal(
        self, position: Tuple[float, float, float]
    ) -> Tuple[float, Tuple[float, float, float, float]]:
        """
        Gets the height and normal of the terrain at a given position.

        Args:
            position (Tuple[float,float,float]): The position in meters.

        Returns:
            Tuple[float, Tuple[float,float,float,float]]: The height and normal of the terrain at the given position.
                                                          (height, (x,y,z,w))
        """

        normal_vector = self.LSTM.get_normal_local(position)
        heading_vector = np.array([1.0, 0.0, 0.0])
        heading_vector = heading_vector / np.linalg.norm(heading_vector)
        heading_vector = np.cross(normal_vector, heading_vector)
        heading_vector = heading_vector / np.linalg.norm(heading_vector)
        heading_vector_2 = np.cross(normal_vector, heading_vector)
        RNorm = np.array([heading_vector, heading_vector_2, normal_vector]).T
        RM = SSTR.from_matrix(RNorm)

        return (self.LSTM.get_height_local(position), RM.as_quat())
