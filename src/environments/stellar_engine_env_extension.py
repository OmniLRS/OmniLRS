__author__ = "Antoine Richard, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC
import math
import os

from scipy.spatial.transform import Rotation as SSTR
from typing import List, Tuple
import omni
from pxr import Gf, UsdLux
from src.terrain_management.large_scale_terrain.pxr_utils import set_texture_path, set_xform_ops
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.environments import LargeScaleTerrainConf, LunaryardConf
from src.stellar.stellar_engine import StellarEngine

class StellarEngineEnvExtension(ABC):
    def init_stellar_engine(
        self,
        stage_settings:LunaryardConf | LargeScaleTerrainConf = None,
        stellar_engine_settings: StellarEngineConf = None,
    ) -> None:
        """
        Args:
            stellar_engine_settings (StellarEngineConf): The settings of the stellar engine.
        """

        self._stage_settings = stage_settings
        self._earth_scale = stage_settings.earth_scale     # stage_settings.earth_scale
        self._sun_path = stage_settings.sun_path           # stage_settings.sun_path
        self._sun_lux = None
        self._sun_prim = None
        self._earth_prim = None

        if stellar_engine_settings is not None:
            self.SE = StellarEngine(stellar_engine_settings)
            self.enable_stellar_engine = True
        else:
            self.enable_stellar_engine = False

    def create_sun(self, sun_settings:SunConf):
        # Should be called inside env's build_scene()
        sun = self.stage.DefinePrim(self._sun_path, "Xform")
        self._sun_prim = sun.GetPrim()
        self._sun_lux: UsdLux.DistantLight = UsdLux.DistantLight.Define(
            self.stage, os.path.join(self._sun_path, "sun")
        )
        self._sun_lux.CreateIntensityAttr(sun_settings.intensity)
        self._sun_lux.CreateAngleAttr(sun_settings.angle)
        self._sun_lux.CreateDiffuseAttr(sun_settings.diffuse_multiplier)
        self._sun_lux.CreateSpecularAttr(sun_settings.specular_multiplier)
        self._sun_lux.CreateColorAttr(
            Gf.Vec3f(sun_settings.color[0], sun_settings.color[1], sun_settings.color[2])
        )
        self._sun_lux.CreateColorTemperatureAttr(sun_settings.temperature)
        x, y, z, w = SSTR.from_euler(
            "xyz", [0, sun_settings.elevation, sun_settings.azimuth - 90], degrees=True
        ).as_quat()
        set_xform_ops(
            self._sun_lux.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(0.5, Gf.Vec3d(0.5, -0.5, -0.5)), Gf.Vec3d(1, 1, 1)
        )
        set_xform_ops(self._sun_prim.GetPrim(), Gf.Vec3d(0, 0, 0), Gf.Quatd(w, Gf.Vec3d(x, y, z)), Gf.Vec3d(1, 1, 1))

    def create_earth(self):
        # Should be called inside env's build_scene()
        self._earth_prim = self.stage.DefinePrim(os.path.join(self.scene_name, "Earth"), "Xform")
        self._earth_prim.GetReferences().AddReference(self._stage_settings.earth_usd_path)
        earth_texture_path = os.path.abspath("assets/Textures/Earth/earth_color_with_clouds.tif")
        material_path = f"{self._stage_settings.earth_path}/Looks/OmniPBR"
        set_texture_path(self.stage, material_path, "Shader", earth_texture_path)
        dist = self._stage_settings.earth_distance * self._earth_scale
        px = math.cos(math.radians(self._stage_settings.earth_azimuth)) * dist
        py = math.sin(math.radians(self._stage_settings.earth_azimuth)) * dist
        pz = math.sin(math.radians(self._stage_settings.earth_elevation)) * dist
        set_xform_ops(self._earth_prim, Gf.Vec3d(px, py, pz), Gf.Quatd(0, 0, 0, 1))
        pass

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
                earth_pos = self.SE.get_local_position("earth")
                alt, az, _ = self.SE.get_alt_az("sun")
                quat = self.SE.convert_alt_az_to_quat(alt, az)

                self.set_sun_pose((0,0,0), quat)
                self.set_earth_pose(earth_pos, (0, 0, 0, 1))
                print("Updated stellar engine")
                print("Earth:", earth_pos)
                print("Sun:", quat)

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
            position (Tuple[float,float,float]): The position of the earth. In meters. (x,y,z)
            orientation (Tuple[float,float,float,float]): The orientation of the earth. (w,x,y,z)
        """

        w, x, y, z = (orientation[0], orientation[1], orientation[2], orientation[3])
        px, py, pz = (
            position[0] * self._earth_scale,
            position[1] * self._earth_scale,
            position[2] * self._earth_scale,
        )
        set_xform_ops(self._earth_prim, translate=Gf.Vec3d(px, py, pz), orient=Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    # ==============================================================================
    # Sun control
    # ==============================================================================
    
    def get_sun_prim_path(self) -> str:
        return self._sun_path

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

    def set_sun_intensity(self, intensity: float) -> None:
        """
        Sets the intensity of the sun.

        Args:
            intensity (float): The intensity of the projector (arbitrary unit).
        """

        self._sun_lux.GetIntensityAttr().Set(intensity)

    def set_sun_color(self, color: List[float]) -> None:
        """
        Sets the color of the projector.

        Args:
            color (List[float]): The color of the projector (RGB).
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
