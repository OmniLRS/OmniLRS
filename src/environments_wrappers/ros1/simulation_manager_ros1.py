__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import Union

from isaacsim.core.api.world import World
import omni

from src.environments_wrappers.ros1.largescale_ros1 import ROS_LargeScaleManager
from src.environments_wrappers.ros1.lunaryard_ros1 import ROS_LunaryardManager
from src.environments_wrappers.ros1.lunalab_ros1 import ROS_LunalabManager
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.physics.physics_scene import PhysicsSceneManager


class ROS1_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(
        self,
        name: str,
        lab_manager: Union[ROS_LunalabManager, ROS_LunaryardManager, ROS_LargeScaleManager],
    ) -> None:
        """
        Registers a lab manager.

        Args:
            name (str): Name of the lab manager.
            lab_manager (Union[ROS_LunalabManager, ROS_LunaryardManager]): Instance of the lab manager.
        """

        self._lab_managers[name] = lab_manager

    def __call__(
        self,
        cfg: dict,
    ) -> Union[ROS_LunalabManager, ROS_LunaryardManager, ROS_LargeScaleManager]:
        """
        Returns an instance of the lab manager corresponding to the environment name.

        Args:
            cfg (dict): Configuration dictionary.

        Returns:
            Union[ROS_LunalabManager, ROS_LunaryardManager]: Instance of the lab manager.
        """

        return self._lab_managers[cfg["environment"]["name"]](
            environment_cfg=cfg["environment"],
            flares_cfg=cfg["rendering"]["lens_flares"],
        )


ROS1_LMF = ROS1_LabManagerFactory()
ROS1_LMF.register("Lunalab", ROS_LunalabManager)
ROS1_LMF.register("Lunaryard", ROS_LunaryardManager)
ROS1_LMF.register("LargeScale", ROS_LargeScaleManager)


class ROS1_SimulationManager:
    """ "
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab and the robot manager thread
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(
        self,
        cfg: dict,
        simulation_app,
    ) -> None:
        """
        Initializes the simulation manager.

        Args:
            cfg (dict): Configuration dictionary.
            simulation_app: Simulation application."""
        self.cfg = cfg
        self.simulation_app = simulation_app
        # Setups the physics and acquires the different interfaces to talk with Isaac
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        PSM = PhysicsSceneManager(cfg["physics"]["physics_scene"])
        for _ in range(100):
            self.world.step(render=False)
        self.world.reset()

        # Lab manager thread (ROS1 does not allow to run multiple threads from the same file)
        # So unlike ROS2, we cannot run the lab manager and the robot manager in parallel.
        # This also means the Lab manager is a mess in ROS1.
        # However, unlike ROS2, I have yet to find the limit of topics you can subscribe to.
        # Penny for your thoughts "Josh".
        self.ROSLabManager = ROS1_LMF(cfg)
        self.terrain_manager_conf: TerrainManagerConf = cfg["environment"]["terrain_manager"]
        self.render_deform_inv = self.terrain_manager_conf.moon_yard.deformation_engine.render_deform_inv
        self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable
        self.world.reset()

        # Preload the assets
        self.ROSLabManager.RM.preload_robot(self.world)
        self.ROSLabManager.LC.add_robot_manager(self.ROSLabManager.RM)

        for _ in range(100):
            self.world.step(render=False)
        self.world.reset()

    def run_simulation(self) -> None:
        """
        Runs the simulation."""
        self.timeline.play()
        while self.simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                # Apply modifications to the lab only once the simulation step is finished
                # This is extremely important as modifying the stage during a simulation step
                # will lead to a crash.
                self.ROSLabManager.periodic_update(dt=self.world.get_physics_dt())
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                self.ROSLabManager.apply_modifications()
                if self.enable_deformation:
                    if self.world.current_time_step_index % self.render_deform_inv == 0:
                        self.ROSLabManager.LC.deform_terrain()
                    # self.ROSLabManager.LC.applyTerramechanics()

        self.timeline.stop()
