__author__ = "Antoine Richard, Junnosuke Kamohara, Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from typing import Dict, List, Tuple

from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, UsdLux

from assets import get_assets_path
from src.configurations.environments import LunalabConf
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.base_env import BaseEnv
from src.environments.monitoring_cameras_manager import MonitoringCamerasManager
from src.environments.static_assets_manager import StaticAssetsManager
from src.environments.terrain_control_mixin import TerrainControlMixin
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops


class LunalabController(BaseEnv, TerrainControlMixin):
    """
    This class is used to control the environment's interactive elements."""

    def __init__(
        self,
        mode: SimulatorMode = SimulatorMode.ROS2,
        lunalab_settings: LunalabConf = None,
        rocks_settings: Dict = None,
        terrain_manager: TerrainManagerConf = None,
        static_assets_settings: Dict = None,
        monitoring_cameras_settings: Dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.
        Including:
            - Projector position, intensity, radius, color.
            - Room lights intensity, radius, color.
            - Curtains open or closed.
            - Terrains randomization or using premade DEMs.
            - Rocks random placements.

        Args:
            lunalab_settings (LunalabLabConf): The settings of the lab.
            rocks_settings (Dict): The settings of the rocks.
            terrain_manager (TerrainManagerConf): The settings of the terrain manager.
            **kwargs: Arbitrary keyword arguments."""

        super().__init__(mode, **kwargs)
        self.stage_settings = lunalab_settings
        self.scene_name = "/Lunalab"

        if static_assets_settings:
            self.SAM = StaticAssetsManager(static_assets_settings)

        if monitoring_cameras_settings and monitoring_cameras_settings["enabled"]:
            self.MCM = MonitoringCamerasManager(self._mode, monitoring_cameras_settings)

        self.init_terrain_control(terrain_manager=terrain_manager, rocks_settings=rocks_settings)

    def build_scene(self) -> None:
        """
        Builds the scene. It either loads the scene from a file or creates it from scratch.
        """

        scene_path = get_assets_path() + "/USD_Assets/environments/Lunalab.usd"
        # Loads the Lunalab
        add_reference_to_stage(scene_path, self.scene_name)

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        self.build_scene()
        # Fetches the interactive elements
        self.collect_interactive_assets()
        self.build_RM()  # TODO should first call switch_terrain() as it calls load_DEM() which sets up self.dem and self.mask that are used inside build_RM(), otherwise they are just None (but either ways inside RM.build() neither of those params are being utilized)
        # Loads the DEM and the mask
        self.switch_terrain(0)

        if self.SAM:
            self.SAM.spawn()

        if self.MCM:
            self.MCM.spawn()

    # ==============================================================================
    # Lunalab-specific configs
    # ==============================================================================

    def get_lux_assets(self, prim: "Usd.Prim") -> List[Usd.Prim]:
        """
        Returns the UsdLux prims under a given prim.

        Args:
            prim (Usd.Prim): The prim to be searched.

        Returns:
            list: A list of UsdLux prims.
        """

        lights = []
        for prim in Usd.PrimRange(prim):
            if prim.IsA(UsdLux.SphereLight):
                lights.append(prim)
            if prim.IsA(UsdLux.CylinderLight):
                lights.append(prim)
            if prim.IsA(UsdLux.DiskLight):
                lights.append(prim)
        return lights

    def collect_interactive_assets(self) -> None:
        """
        Collects the interactive assets from the stage and assigns them to class variables.
        """

        # Projector
        self._projector_prim = self.stage.GetPrimAtPath(self.stage_settings.projector_path)
        self._projector_xform = UsdGeom.Xformable(self._projector_prim)
        self._projector_lux = self.get_lux_assets(self._projector_prim)
        self._projector_flare = self.stage.GetPrimAtPath(self.stage_settings.projector_shader_path)
        # Room Lights
        self._room_lights_prim = self.stage.GetPrimAtPath(self.stage_settings.room_lights_path)
        self._room_lights_xform = UsdGeom.Xformable(self._room_lights_prim)
        self._room_lights_lux = self.get_lux_assets(self._room_lights_prim)
        # Curtains
        self._curtain_prims: Dict[str, Usd.Prim] = {}
        for key in self.stage_settings.curtains_path.keys():
            self._curtain_prims[key] = self.stage.GetPrimAtPath(self.stage_settings.curtains_path[key])

    # ==============================================================================
    # Projector control
    # ==============================================================================
    def set_projector_pose(
        self,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        """
        Sets the pose of the projector.

        Args:
            position (Tuple[float,float,float]): The position of the projector. (x,y,z)
            quat (Tuple[float,float,float,float]): The quaternion of the projector. (w,x,y,z)
        """

        w, x, y, z = (orientation[0], orientation[1], orientation[2], orientation[3])
        px, py, pz = (position[0], position[1], position[2])

        set_xform_ops(
            self._projector_xform, Gf.Vec3d(px, py, pz), Gf.Quatd(w, Gf.Vec3d(x, y, z)), Gf.Vec3d(1.0, 1.0, 1.0)
        )

    def set_projector_intensity(self, intensity: float = 0.0) -> None:
        """
        Sets the intensity of the projector.

        Args:
            intensity (float): The intensity of the projector (arbitrary unit).
        """

        self._projector_lux[0].GetAttribute("intensity").Set(intensity)

    def set_projector_radius(self, radius: float = 0.1) -> None:
        """
        Sets the radius of the projector.

        Args:
            radius (float): The radius of the projector (in meters).
        """

        self._projector_lux[0].GetAttribute("radius").Get(radius)

    def set_projector_color(self, color: Tuple[float, float, float] = (1.0, 1.0, 1.0)) -> None:
        """
        Sets the color of the projector.

        Args:
            color (Tuple[float, float, float]): The color of the projector (RGB).
        """

        color = Gf.Vec3d(color[0], color[1], color[2])
        self._projector_flare.GetAttribute("inputs:emissive_color").Set(color)
        self._projector_flare.GetAttribute("inputs:diffuse_color_constant").Set(color)
        self._projector_flare.GetAttribute("inputs:diffuse_tint").Set(color)
        self._projector_lux[0].GetAttribute("color").Set(color)

    def turn_projector_on_off(self, flag: bool = True) -> None:
        """
        Turns the projector on or off.

        Args:
            flag (bool): True to turn the projector on, False to turn it off.
        """

        if flag:
            self._projector_prim.GetAttribute("visibility").Set("visible")
        else:
            self._projector_prim.GetAttribute("visibility").Set("invisible")

    # ==============================================================================
    # Room lights control
    # ==============================================================================
    def set_room_lights_intensity(self, intensity: float = 0.0) -> None:
        """
        Sets the intensity of the room lights.

        Args:
            intensity (float): The intensity of the room lights (arbitrary unit).
        """

        for light in self._room_lights_lux:
            light.GetAttribute("intensity").Set(intensity)

    def set_room_lights_radius(self, radius: float = 0.1) -> None:
        """
        Sets the radius of the room lights.

        Args:
            radius (float): The radius of the room lights (in meters).
        """

        for light in self._room_lights_lux:
            light.GetAttribute("radius").Set(radius)

    def set_room_lights_FOV(self, FOV: float = 42.0) -> None:
        """
        Sets the FOV of the room lights.

        Args:
            FOV (float): The FOV of the room lights (in degrees).
        """

        for light in self._room_lights_lux:
            light.GetAttribute("shaping:cone:angle").Set(FOV)

    def set_room_lights_color(self, color: Tuple[float, float, float] = (1.0, 1.0, 1.0)) -> None:
        """
        Sets the color of the room lights.

        Args:
            color (List[float]): The color of the room lights (RGB).
        """

        color = Gf.Vec3d(color[0], color[1], color[2])
        for light in self._room_lights_lux:
            light.GetAttribute("color").Set(color)

    def turn_room_lights_on_off(self, flag: bool = True) -> None:
        """
        Turns the room lights on or off.

        Args:
            flag (bool): True to turn the room lights on, False to turn them off.
        """

        if flag:
            self._room_lights_prim.GetAttribute("visibility").Set("visible")
        else:
            self._room_lights_prim.GetAttribute("visibility").Set("invisible")

    # ==============================================================================
    # Curtains control
    # ==============================================================================
    def curtains_extend(self, flag: bool = True) -> None:
        """
        Extends or folds the curtains.

        Args:
            flag (bool): True to extend the curtains, False to fold them.
        """

        if flag:
            self._curtain_prims["extended"].GetAttribute("visibility").Set("visible")
            self._curtain_prims["folded"].GetAttribute("visibility").Set("invisible")
        else:
            self._curtain_prims["extended"].GetAttribute("visibility").Set("invisible")
            self._curtain_prims["folded"].GetAttribute("visibility").Set("visible")
