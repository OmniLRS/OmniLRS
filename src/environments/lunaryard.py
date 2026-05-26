__author__ = "Antoine Richard, Junnosuke Kamohara, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from scipy.spatial.transform import Rotation as SSTR
from typing import List, Tuple, Union, Dict
import numpy as np
import math
import os

from isaacsim.core.utils.stage import add_reference_to_stage
import omni

from pxr import UsdGeom, UsdLux, Gf, Usd

from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.environments.static_assets_manager import StaticAssetsManager
from src.environments.stellar_engine_env_extension import StellarEngineEnvExtension
from src.environments.terrain_control_extension import TerrainControlExtension
from src.terrain_management.large_scale_terrain.pxr_utils import load_material
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.configurations.environments import LunaryardConf
from src.environments.base_env import BaseEnv
from src.configurations.simulator_mode_enum import SimulatorMode


class LunaryardController(BaseEnv, StellarEngineEnvExtension, TerrainControlExtension):
    """
    This class is used to control the lab interactive elements.
    """

    def __init__(
        self,
        mode:SimulatorMode = SimulatorMode.ROS2,
        lunaryard_settings: LunaryardConf = None,
        rocks_settings: Dict = None,
        terrain_manager: TerrainManagerConf = None,
        stellar_engine_settings: StellarEngineConf = None,
        sun_settings: SunConf = None,
        static_assets_settings: Dict = None,
        monitoring_cameras_settings: Dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.
        Including:
            - Sun position, intensity, radius, color.
            - Terrains randomization or using premade DEMs.
            - Rocks random placements.

        Args:
            lunaryard_settings (LunaryardConf): The settings of the lab.
            rocks_settings (Dict): The settings of the rocks.
            terrain_manager (TerrainManagerConf): The settings of the terrain manager.
            stellar_engine_settings (StellarEngineConf): The settings of the stellar engine.
            **kwargs: Arbitrary keyword arguments.
        """

        super().__init__(mode, **kwargs)
        self.scene_name = "/Lunaryard"
        self.stage_settings = lunaryard_settings

        if static_assets_settings:
            self.SAM = StaticAssetsManager(static_assets_settings)

        if monitoring_cameras_settings and monitoring_cameras_settings["enabled"]:
            self.MCM = MonitoringCamerasManager(self._mode, monitoring_cameras_settings)

        self._sun_settings = sun_settings
        self.init_stellar_engine(stage_settings=self.stage_settings, 
                                  stellar_engine_settings=stellar_engine_settings)

        self.init_terrain_control(terrain_manager=terrain_manager, rocks_settings=rocks_settings)

    def build_scene(self) -> None:
        """
        Builds the scene.
        """

        # Creates an empty xform with the name lunaryard
        lunaryard = self.stage.DefinePrim(self.scene_name, "Xform")

        self.create_sun(self._sun_settings)
        self.create_earth()

        # Load default textures
        looks_path = os.path.join(self.scene_name, "Looks")
        self.stage.DefinePrim(looks_path, "Scope")
        load_material("Basalt", "assets/Textures/GravelStones.mdl", looks_path)
        load_material("Sand", "assets/Textures/Sand.mdl", looks_path)
        load_material("LunarRegolith8k", "assets/Textures/LunarRegolith8k.mdl", looks_path)

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        # Builds the scene
        self.build_scene()

        # Generates the instancer for the rocks
        self.RM.build(self.dem, self.mask)
        # Loads the DEM and the mask
        self.switch_terrain(self.stage_settings.terrain_id)
        if self.enable_stellar_engine:
            self.SE.set_lat_lon(self.stage_settings.coordinates.latitude, self.stage_settings.coordinates.longitude)
            self.update_stellar_engine()

        if self.SAM:
            self.SAM.spawn()

        if self.MCM:
            self.MCM.spawn()
