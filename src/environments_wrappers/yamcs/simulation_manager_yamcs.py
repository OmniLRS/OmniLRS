__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from isaacsim import SimulationApp
from isaacsim.core.api.world import World
from typing import Union
import logging
import omni

from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.large_scale_lunar import LargeScaleController
from src.environments.lunalab import LunalabController
from src.environments.lunaryard import LunaryardController
from src.environments.utils import set_moon_env_name
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.environments_wrappers.rate import Rate
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

class Yamcs_SimulationManager:
    """
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager 
    - Running the robot manager 
    - Running the simulation
    - Cleaning the simulation

    Yamcs_SimulationManager is implemented in a simpler manner than the ROS2 manager:
    - there is no environment wrapper for each of the implemented environments (Lunalabn, Lunaryard, LargeScale)
    - there is no factory, but instead if/else was used
    for the reason that as for now, there is no implementation of environment alteration through Yamcs commands,
    nor is there a need for a wrapper that inherits from ROS's Node (as is the case with ROS_SimulationManager)
    The implementation may change in the future.
    """

    def __init__(
        self,
        cfg: dict,
        simulation_app: SimulationApp,
    ) -> None:
        """
        Initializes the simulation.

        Args:
            cfg (dict): Configuration dictionary.
            simulation_app (SimulationApp): SimulationApp instance.
        """

        self.cfg = cfg
        self.simulation_app = simulation_app
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=cfg["environment"]["physics_dt"],
            rendering_dt=cfg["environment"]["rendering_dt"],
        )

        set_moon_env_name(cfg["environment"]["name"])
        self._step_world_and_reset()
        self._setup_rate()

        self.EC = self._get_environment_controller(self.cfg["environment"]["name"])
        self.EC.load()

        self.RM = RobotManager(cfg["environment"]["robots_settings"], 
                               mode=SimulatorMode.YAMCS, 
                               yamcs_instance_conf=cfg["mode"]["instance_conf"])
        self._setup_terrain_manager()
        self._preload_robot()
        self.EC.add_robot_manager(self.RM)
        self._step_world_and_reset()

    def _get_environment_controller(self, environment_name:str):
        self.EC= None
        if environment_name == "LargeScale":
            self.EC = LargeScaleController(
                mode=SimulatorMode.YAMCS, **self.cfg["environment"], 
                is_simulation_alive=self.simulation_app.is_running, 
                close_simulation=self.simulation_app.close
            )
        elif environment_name == "Lunaryard":
            self.EC = LunaryardController(
                mode=SimulatorMode.YAMCS, **self.cfg["environment"], 
                is_simulation_alive=self.simulation_app.is_running, 
                close_simulation=self.simulation_app.close
            )
        elif environment_name == "Lunalab":
            self.EC = LunalabController(
                mode=SimulatorMode.YAMCS, **self.cfg["environment"], 
                is_simulation_alive=self.simulation_app.is_running, 
                close_simulation=self.simulation_app.close
            )

        return self.EC

    def _step_world_and_reset(self, n=100):
        for i in range(n):
            self.world.step(render=True)
        self.world.reset()

    def _setup_rate(self):
        if self.cfg["environment"]["enforce_realtime"]:
            self.rate = Rate(dt=self.cfg["environment"]["physics_dt"])
        else:
            self.rate = Rate(is_disabled=True)

    def _setup_terrain_manager(self):
        if "terrain_manager" in self.cfg["environment"].keys():
            self.terrain_manager_conf: TerrainManagerConf = self.cfg["environment"]["terrain_manager"]
            self.deform_delay = self.terrain_manager_conf.moon_yard.deformation_engine.delay
            self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable
        else:
            self.enable_deformation = False

    def _preload_robot(self):
        if self.cfg["environment"]["name"] == "LargeScale":
            height, quat = self.EC.get_height_and_normal((0.0, 0.0, 0.0))
            self.RM.preload_robot_at_pose(self.world, (0, 0, height + 0.5), (1, 0, 0, 0))
        else:
            self.RM.preload_robot(self.world)

    def run_simulation(self) -> None:
        """
        Runs the simulation.
        """

        self.timeline.play()
        while self.simulation_app.is_running():
            self.rate.reset()
            self.world.step(render=True)
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()
            self.rate.sleep()
        self.world.stop()
        self.timeline.stop()
