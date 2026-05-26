__author__ = "Antoine Richard, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from scipy.spatial.transform import Rotation as SSTR
from typing import Dict, List, Tuple
import numpy as np
import math
import os

from isaacsim.core.utils.stage import add_reference_to_stage
import omni

from pxr import UsdLux, Gf, Usd

from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.static_assets_manager import StaticAssetsManager
from src.environments.stellar_engine_env_extension import StellarEngineEnvExtension
from src.terrain_management.large_scale_terrain_manager import LargeScaleTerrainManager
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops, set_texture_path
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.environments import LargeScaleTerrainConf
from src.environments.base_env import BaseEnv
from src.robots.robot import RobotManager
from assets import get_assets_path


class LargeScaleController(BaseEnv, StellarEngineEnvExtension):
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
        self.stage_settings = large_scale_terrain
        self.sun_settings = sun_settings

        self.is_simulation_alive = is_simulation_alive

        self.init_stellar_engine(earth_scale=self.stage_settings.earth_scale, 
                                  sun_path=self.stage_settings.sun_path, 
                                  stellar_engine_settings=stellar_engine_settings)

        self.dem = None
        self.mask = None
        self.scene_name = "/LargeScaleLunar"

        self.SAM = None
        self.MCM = None

        if static_assets_settings:
            self.SAM = StaticAssetsManager(static_assets_settings)

        if monitoring_cameras_settings and monitoring_cameras_settings["enabled"]:
            self.MCM = MonitoringCamerasManager(self._mode, monitoring_cameras_settings)

    def build_scene(self) -> None:
        """
        Builds the scene.
        """

        # Creates an empty xform with the name lunaryard
        large_scale = self.stage.DefinePrim(self.scene_name, "Xform")

        # Creates the sun
        sun = self.stage.DefinePrim(self.stage_settings.sun_path, "Xform")
        self._sun_prim = sun.GetPrim()
        self._sun_lux: UsdLux.DistantLight = UsdLux.DistantLight.Define(
            self.stage, os.path.join(self.stage_settings.sun_path, "sun")
        )
        self._sun_lux.CreateIntensityAttr(self.sun_settings.intensity)
        self._sun_lux.CreateAngleAttr(self.sun_settings.angle)
        self._sun_lux.CreateDiffuseAttr(self.sun_settings.diffuse_multiplier)
        self._sun_lux.CreateSpecularAttr(self.sun_settings.specular_multiplier)
        self._sun_lux.CreateColorAttr(
            Gf.Vec3f(self.sun_settings.color[0], self.sun_settings.color[1], self.sun_settings.color[2])
        )
        self._sun_lux.CreateColorTemperatureAttr(self.sun_settings.temperature)
        x, y, z, w = SSTR.from_euler(
            "xyz", [0, self.sun_settings.elevation, self.sun_settings.azimuth - 90], degrees=True
        ).as_quat()
        set_xform_ops(
            self._sun_lux.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(0.5, Gf.Vec3d(0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1)
        )
        set_xform_ops(self._sun_prim.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(w, Gf.Vec3d(x, y, z)), Gf.Vec3d(1, 1, 1))

        # Creates the earth
        self._earth_prim = self.stage.DefinePrim(os.path.join(self.scene_name, "Earth"), "Xform")
        self._earth_prim.GetReferences().AddReference(self.stage_settings.earth_usd_path)
        earth_texture_path = os.path.abspath("assets/Textures/Earth/earth_color_with_clouds.tif")
        material_path = f"{self.stage_settings.earth_path}/Looks/OmniPBR"
        set_texture_path(self.stage, material_path, "Shader", earth_texture_path)
        dist = self.stage_settings.earth_distance * self.stage_settings.earth_scale
        px = math.cos(math.radians(self.stage_settings.earth_azimuth)) * dist
        py = math.sin(math.radians(self.stage_settings.earth_azimuth)) * dist
        pz = math.sin(math.radians(self.stage_settings.earth_elevation)) * dist
        set_xform_ops(self._earth_prim, Gf.Vec3d(px, py, pz), Gf.Quatd(0, 0, 0, 1))

    def instantiate_scene(self) -> None:
        """
        Instantiates the scene. Applies any operations that need to be done after the scene is built and
        the renderer has been stepped.
        """

        pass

    def reset(self) -> None:
        """
        Resets the environment. Implement the logic to reset the environment.
        """

        pass

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

    def deform_derrain(self) -> None:
        """
        Deforms the terrain.
        Args:
            world_poses (np.ndarray): The world poses of the contact points.
            contact_forces (np.ndarray): The contact forces in local frame reported by rigidprimview.
        """
        pass

    def apply_terramechanics(self) -> None:
        pass
