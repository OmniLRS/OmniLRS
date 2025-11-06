__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
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
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops, load_material, set_texture_path
from src.physics.terramechanics_parameters import RobotParameter, TerrainMechanicalParameter
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.physics.terramechanics_solver import TerramechanicsSolver
from src.terrain_management.terrain_manager import TerrainManager
from src.configurations.environments import LunaryardConf
from src.environments.rock_manager import RockManager
from src.stellar.stellar_engine import StellarEngine
from src.environments.base_env import BaseEnv
from src.robots.robot import RobotManager


class LunaryardController(BaseEnv):
    """
    This class is used to control the lab interactive elements.
    """

    def __init__(
        self,
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

        super().__init__(**kwargs)
        self.stage_settings = lunaryard_settings
        self.sun_settings = sun_settings

        if stellar_engine_settings is not None:
            self.SE = StellarEngine(stellar_engine_settings)
            self.enable_stellar_engine = True
        else:
            self.enable_stellar_engine = False

        self.T = TerrainManager(terrain_manager)
        self.RM = RockManager(**rocks_settings)
        self.TS = TerramechanicsSolver(
            robot_param=RobotParameter(),
            terrain_param=TerrainMechanicalParameter(),
        )
        self.SAM = StaticAssetsManager(static_assets_settings)
        self.MCM = MonitoringCamerasManager(monitoring_cameras_settings)
        self.dem = None
        self.mask = None
        self.scene_name = "/Lunaryard"
        self.deformation_conf = terrain_manager.moon_yard.deformation_engine

    def build_scene(self) -> None:
        """
        Builds the scene.
        """

        # Creates an empty xform with the name lunaryard
        lunaryard = self.stage.DefinePrim(self.scene_name, "Xform")
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
        self._earth_prim = self.stage.DefinePrim(self.stage_settings.earth_path, "Xform")
        self._earth_prim.GetReferences().AddReference(self.stage_settings.earth_usd_path)
        earth_texture_path = os.path.abspath("assets/Textures/Earth/earth_color_with_clouds.tif")
        material_path = f"{self.stage_settings.earth_path}/Looks/OmniPBR"
        set_texture_path(self.stage, material_path, "Shader", earth_texture_path)
        dist = self.stage_settings.earth_distance * self.stage_settings.earth_scale
        px = math.cos(math.radians(self.stage_settings.earth_azimuth)) * dist
        py = math.sin(math.radians(self.stage_settings.earth_azimuth)) * dist
        pz = math.sin(math.radians(self.stage_settings.earth_elevation)) * dist
        set_xform_ops(self._earth_prim, Gf.Vec3d(px, py, pz), Gf.Quatd(0, 0, 0, 1))

        # Load default textures
        looks_path = os.path.join(self.scene_name, "Looks")
        self.stage.DefinePrim(looks_path, "Scope")
        load_material("Basalt", "assets/Textures/GravelStones.mdl", looks_path)
        load_material("Sand", "assets/Textures/Sand.mdl", looks_path)
        load_material("LunarRegolith8k", "assets/Textures/LunarRegolith8k.mdl", looks_path)

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

        pass

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
        self.SAM.spawn_from_config()
        self.MCM.spawn_from_config()

    def add_robot_manager(self, robotManager: RobotManager) -> None:
        self.robotManager = robotManager

    def load_DEM(self) -> None:
        """
        Loads the DEM and the mask from the TerrainManager.
        """

        self.dem = self.T.getDEM()
        self.mask = self.T.getMask()

    # ==============================================================================
    # Stellar engine control
    # ==============================================================================

    def set_coordinates(self, latlong: Tuple[float, float] = (0.0, 0.0)) -> None:
        """
        Sets the coordinates of the lab.

        Args:
            latlong (Tuple[float,float]): The latitude and longitude of the scene on the moon.
        """

        if self.enable_stellar_engine:
            self.SE.set_lat_lon(latlong[1], latlong[0])

    def set_time(self, time: float = 0.0) -> None:
        """
        Sets the time of the stellar engine.

        Args:
            time (float): The time in seconds.
        """

        if self.enable_stellar_engine:
            self.SE.set_time(time)

    def set_time_scale(self, scale: float = 1.0) -> None:
        """
        Sets the time scale of the stellar engine.

        Args:
            scale (float): The time scale.
        """

        if self.enable_stellar_engine:
            self.SE.set_time_scale(scale)

    def update_stellar_engine(self, dt: float = 0.0) -> None:
        """
        Updates the sun and earth pose.

        Args:
            dt (float): The time step.
        """

        if self.enable_stellar_engine:
            update = self.SE.update(dt)
            if update:
                # Unscaled earth position
                earth_pos = self.SE.get_local_position("earth")
                # Sun altitude and azimuth
                alt, az, _ = self.SE.get_alt_az("sun")
                quat = self.SE.convert_alt_az_to_quat(alt, az)

                self.set_sun_pose((0, 0, 0), quat)
                self.set_earth_pose(earth_pos, (1, 0, 0, 0))

    # ==============================================================================
    # Earth control
    # ==============================================================================

    def set_earth_pose(
        self,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """
        Sets the pose of the earth.

        Args:
            position (Tuple[float,float,float]): The position of the earth. In meters. (x,y,z) (unscaled)
            orientation (Tuple[float,float,float,float]): The orientation of the earth. (w,x,y,z)
        """

        w, x, y, z = (orientation[0], orientation[1], orientation[2], orientation[3])
        px, py, pz = (
            position[0] * self.stage_settings.earth_scale,
            position[1] * self.stage_settings.earth_scale,
            position[2] * self.stage_settings.earth_scale,
        )
        set_xform_ops(self._earth_prim, translate=Gf.Vec3d(px, py, pz), orient=Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    # ==============================================================================
    # Sun control
    # ==============================================================================

    def set_sun_pose(
        self,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """
        Sets the pose of the sun.

        Args:
            position (Tuple[float,float,float]): The position of the sun. In meters. (x,y,z)
            orientation (Tuple[float,float,float,float]): The orientation of the sun. (w,x,y,z)
        """

        w, x, y, z = (orientation[0], orientation[1], orientation[2], orientation[3])
        set_xform_ops(self._sun_prim, orient=Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    def set_sun_intensity(self, intensity: float = 0.0) -> None:
        """
        Sets the intensity of the sun.

        Args:
            intensity (float): The intensity of the projector (arbitrary unit).
        """

        self._sun_lux.GetIntensityAttr().Set(intensity)

    def set_sun_color(self, color: Tuple[float, float, float] = (1.0, 1.0, 1.0)) -> None:
        """
        Sets the color of the projector.

        Args:
            color (Tuple[float,float,float]): The color of the projector. (r,g,b)
        """

        color = Gf.Vec3d(color[0], color[1], color[2])
        self._sun_lux.GetColorAttr().Set(color)

    def set_sun_color_temperature(self, temperature: float = 6500.0) -> None:
        """
        Sets the color temperature of the projector.

        Args:
            temperature (float): The color temperature of the projector in Kelvin.
        """

        self._sun_lux.GetColorTemperatureAttr().Set(temperature)

    def set_sun_angle(self, angle: float = 0.53) -> None:
        """
        Sets the angle of the sun. Larger values make the sun larger, and soften the shadows.

        Args:
            angle (float): The angle of the projector.
        """

        self._sun_lux.GetAngleAttr().Set(angle)

    # ==============================================================================
    # Terrain control
    # ==============================================================================

    def switch_terrain(self, flag: int = -1) -> None:
        """
        Switches the terrain to a new DEM.

        Args:
            flag (int): The id of the DEM to be loaded. If negative, a random DEM is generated.
        """

        if flag < 0:
            self.T.randomizeTerrain()
        else:
            self.T.loadTerrainId(flag)
        self.load_DEM()
        self.RM.updateImageData(self.dem, self.mask)
        self.RM.randomizeInstancers(10)

    def enable_rocks(self, flag: bool = True) -> None:
        """
        Turns the rocks on or off.

        Args:
            flag (bool): True to turn the rocks on, False to turn them off.
        """

        self.RM.setVisible(flag)

    def randomize_rocks(self, num: int = 8) -> None:
        """
        Randomizes the placement of the rocks.

        Args:
            num (int): The number of rocks to be placed.
        """

        num = int(num)
        if num == 0:
            num += 1
        self.RM.randomizeInstancers(num)

    def deform_terrain(self) -> None:
        """
        Deforms the terrain.
        Args:
            world_poses (np.ndarray): The world poses of the contact points.
            contact_forces (np.ndarray): The contact forces in local frame reported by rigidprimview.
        """
        world_positions = []
        world_orientations = []
        contact_forces = []
        for rrg in self.robotManager.robots_RG.values():
            position, orientation = rrg.get_pose()
            world_positions.append(position)
            world_orientations.append(orientation)
            contact_forces.append(rrg.get_net_contact_forces())
        world_positions = np.concatenate(world_positions, axis=0)
        world_orientations = np.concatenate(world_orientations, axis=0)
        contact_forces = np.concatenate(contact_forces, axis=0)

        self.T.deformTerrain(
            world_positions,
            world_orientations,
            contact_forces,
        )
        self.load_DEM()
        self.RM.updateImageData(self.dem, self.mask)

    def apply_terramechanics(self) -> None:
        for rrg in self.robotManager.robots_RG.values():
            linear_velocities, angular_velocities = rrg.get_velocities()
            sinkages = np.zeros((linear_velocities.shape[0],))
            force, torque = self.TS.compute_force_and_torque(linear_velocities, angular_velocities, sinkages)
            rrg.apply_force_torque(force, torque)
            rrg.apply_force_torque(force, torque)
