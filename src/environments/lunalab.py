__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple, Dict
import numpy as np

from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
import omni

from pxr import UsdGeom, UsdLux, Gf, Usd

from src.physics.terramechanics_parameters import RobotParameter, TerrainMechanicalParameter
from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.physics.terramechanics_solver import TerramechanicsSolver
from src.terrain_management.terrain_manager import TerrainManager
from src.configurations.environments import LunalabConf
from src.environments.rock_manager import RockManager
from src.environments.base_env import BaseEnv
from src.robots.robot import RobotManager
from assets import get_assets_path


class LunalabController(BaseEnv):
    """
    This class is used to control the environment's interactive elements."""

    def __init__(
        self,
        lunalab_settings: LunalabConf = None,
        rocks_settings: Dict = None,
        terrain_manager: TerrainManagerConf = None,
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

        super().__init__(**kwargs)
        self.stage_settings = lunalab_settings
        self.T = TerrainManager(terrain_manager)
        self.RM = RockManager(**rocks_settings)
        self.TS = TerramechanicsSolver(
            robot_param=RobotParameter(),
            terrain_param=TerrainMechanicalParameter(),
        )
        self.dem = None
        self.mask = None
        self.scene_name = "/Lunalab"
        self.deformation_conf = terrain_manager.moon_yard.deformation_engine

    def build_scene(self) -> None:
        """
        Builds the scene. It either loads the scene from a file or creates it from scratch.
        """

        scene_path = get_assets_path() + "/USD_Assets/environments/Lunalab.usd"
        # Loads the Lunalab
        add_reference_to_stage(scene_path, self.scene_name)

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

        self.build_scene()
        # Fetches the interactive elements
        self.collect_interactive_assets()
        self.RM.build(self.dem, self.mask)
        # Loads the DEM and the mask
        self.switch_terrain(0)

    def add_robot_manager(self, robotManager: RobotManager) -> None:
        """
        Adds the robot manager to the environment.

        Args:
            robotManager (RobotManager): The robot manager to be added.
        """

        self.robotManager = robotManager

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

    def load_DEM(self) -> None:
        """
        Loads the DEM and the mask from the TerrainManager.
        """

        self.dem = self.T.getDEM()
        self.mask = self.T.getMask()

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
        """
        Applies the terramechanics solver to the robots.
        """

        for rrg in self.robotManager.robots_RG.values():
            linear_velocities, angular_velocities = rrg.get_velocities()
            sinkages = np.zeros((linear_velocities.shape[0],))
            force, torque = self.TS.compute_force_and_torque(linear_velocities, angular_velocities, sinkages)
            rrg.apply_force_torque(force, torque)
            rrg.apply_force_torque(force, torque)
