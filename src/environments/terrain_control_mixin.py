__author__ = "Antoine Richard, Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from typing import Dict

import numpy as np

from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.environments.rock_manager import RockManager
from src.physics.terramechanics_parameters import RobotParameter, TerrainMechanicalParameter
from src.physics.terramechanics_solver import TerramechanicsSolver
from src.robots.robot import RobotManager
from src.terrain_management.terrain_manager import TerrainManager


class TerrainControlMixin:
    """
    Mixin providing terrain / rock / terramechanics control for BaseEnv subclasses.

    Host class contract:
        - must also inherit from BaseEnv (provides ``self.stage``)
        - must call ``self.init_terrain_control(...)`` from its ``__init__``
        - ``self.robotManager`` is expected to be set (via ``BaseEnv.add_robot_manager``)
          before ``deform_terrain()`` / ``apply_terramechanics()`` are called

    If you wish to customize the behaviour of a certain class,
    override the corresponding method in the host class.
    """

    robotManager: "RobotManager"  # Provided by BaseEnv via ``add_robot_manager``; type hint for static analysis

    def init_terrain_control(
        self,
        terrain_manager: TerrainManagerConf = None,
        rocks_settings: Dict = None,
    ) -> None:
        """
        Args:
            terrain_manager (TerrainManagerConf): The settings of the terrain manager.
            rocks_settings (Dict): The settings of the rocks.
        """

        self.dem = None  # Digital Elevation Model
        self.mask = None  # Mask for the terrain, used for the rocks placement and deformation
        self.T = TerrainManager(terrain_manager)
        self.RM = RockManager(**rocks_settings)
        self.TS = TerramechanicsSolver(
            robot_param=RobotParameter(),
            terrain_param=TerrainMechanicalParameter(),
        )
        self.deformation_conf = terrain_manager.moon_yard.deformation_engine

    # ==============================================================================
    # Terrain control
    # ==============================================================================

    def build_RM(self):
        self.RM.build(self.dem, self.mask)

    def load_DEM(self) -> None:
        """
        Loads the DEM and the mask from the TerrainManager.
        """

        self.dem = self.T.getDEM()
        self.mask = self.T.getMask()

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
        position, orientation = self.robotManager.robot_RG.get_pose()
        world_positions.append(position)
        world_orientations.append(orientation)
        contact_forces.append(self.robotManager.robot_RG.get_net_contact_forces())
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
        linear_velocities, angular_velocities = self.robotManager.robot_RG.get_velocities()
        sinkages = np.zeros((linear_velocities.shape[0],))
        force, torque = self.TS.compute_force_and_torque(linear_velocities, angular_velocities, sinkages)
        self.robotManager.robot_RG.apply_force_torque(force, torque)
        self.robotManager.robot_RG.apply_force_torque(force, torque) #TODO @Louis should be called twice?
