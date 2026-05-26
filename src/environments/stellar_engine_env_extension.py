__author__ = "Antoine Richard, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from abc import ABC

from scipy.spatial.transform import Rotation as SSTR
from typing import List, Tuple
import omni
from pxr import Gf
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
from src.configurations.stellar_engine_confs import StellarEngineConf, SunConf
from src.configurations.environments import LargeScaleTerrainConf
from src.stellar.stellar_engine import StellarEngine

class StellarEngineEnvExtension(ABC):
    def init_stellar_engine(
        self,
        earth_scale = None,
        sun_path = None,
        stellar_engine_settings: StellarEngineConf = None,
    ) -> None:
        """
        Args:
            stellar_engine_settings (StellarEngineConf): The settings of the stellar engine.
            sun_settings (SunConf): The settings of the sun.
        """

        self._earth_scale = earth_scale     # stage_settings.earth_scale
        self._sun_path = sun_path           # stage_settings.sun_path
        self._sun_lux = None
        self._sun_prim = None
        self._earth_prim = None

        if stellar_engine_settings is not None:
            self.SE = StellarEngine(stellar_engine_settings)
            self.enable_stellar_engine = True
        else:
            self.enable_stellar_engine = False

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
