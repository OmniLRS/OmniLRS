from isaacsim import SimulationApp
from isaacsim.core.api.world import World
from typing import Union
import logging
import omni
import time

from src.environments.utils import set_moon_env_name
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.environments_wrappers.yamcs.largescale_yamcs import Yamcs_LargeScaleManager
from src.environments_wrappers.yamcs.lunalab_yamcs import Yamcs_LunalabManager
from src.environments_wrappers.yamcs.lunaryard_yamcs import Yamcs_LunaryardManager
from src.environments_wrappers.yamcs.robot_manager_yamcs import Yamcs_RobotManager
from src.physics.physics_scene import PhysicsSceneManager

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

#TODO 3rd March 2026 (for after Version 3)
# class Rate should stay, but it should be reused among all Simulation Managers
# therefore, it should be defined in a single place
class Rate:
    """
    Creates a rate object that enables to sleep for a minimum amount
    of time between two iterations of a loop. If freq and dt are
    passed, the object will only use the information provided by dt.
    """

    def __init__(self, freq: float = None, dt: float = None, is_disabled: bool = False) -> None:
        """
        Args:
          freq (float): The frequency at which the loop should be executed.
          dt (float): The delta of time to be kept between two loop iterations.
        """

        self.is_disabled = is_disabled

        if not self.is_disabled:
            if dt is None:
                if freq is None:
                    raise ValueError("You must provide either a frequency or a delta time.")
                else:
                    self.dt = 1.0 / freq
            else:
                self.dt = dt

            self.last_check = time.time()

    def reset(self) -> None:
        """
        Resets the timer.
        """
        if not self.is_disabled:
            self.last_check = time.time()

    def sleep(self) -> None:
        """
        Wait for a minimum amount of time between two iterations of a loop.
        """

        if not self.is_disabled:
            now = time.time()
            delta = now - self.last_check
            # If time delta is too low sleep, else carry on.
            if delta < self.dt:
                to_sleep = self.dt - delta
                time.sleep(to_sleep)

#TODO should use of 'Lab' naming be changed for 'Environment' after Version 3 ?
class Yamcs_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(
        self,
        name: str,
        lab_manager: Union[Yamcs_LunalabManager, Yamcs_LunaryardManager, Yamcs_LargeScaleManager],
    ) -> None:
        """
        Registers a lab manager.

        Args:
            name (str): Name of the lab manager.
            lab_manager (Union[Yamcs_LunalabManager, Yamcs_LunaryardManager]): Instance of the lab manager.
        """

        self._lab_managers[name] = lab_manager

    def __call__(
        self,
        cfg: dict,
        **kwargs,
    ) -> Union[Yamcs_LunalabManager, Yamcs_LunaryardManager, Yamcs_LargeScaleManager]: 
        """
        Returns an instance of the lab manager corresponding to the environment name.

        Args:
            cfg (dict): Configuration dictionary.

        Returns:
            Union[Yamcs_LunalabManager, Yamcs_LunaryardManager]: Instance of the lab manager.
        """

        return self._lab_managers[cfg["environment"]["name"]](
            environment_cfg=cfg["environment"],
            **kwargs,
        )

Yamcs_LMF = Yamcs_LabManagerFactory()
Yamcs_LMF.register("Lunalab", Yamcs_LunalabManager)
Yamcs_LMF.register("Lunaryard", Yamcs_LunaryardManager)
Yamcs_LMF.register("LargeScale", Yamcs_LargeScaleManager)

class Yamcs_SimulationManager:
    """
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager 
    - Running the robot manager 
    - Running the simulation
    - Cleaning the simulation
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
        # Setups the physics and acquires the different interfaces to talk with Isaac
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=cfg["environment"]["physics_dt"],
            rendering_dt=cfg["environment"]["rendering_dt"],
        )

        set_moon_env_name(cfg["environment"]["name"])

        PSM = PhysicsSceneManager(cfg["physics"]["physics_scene"])
        for i in range(100):
            self.world.step(render=True)
        self.world.reset()

        if cfg["environment"]["enforce_realtime"]:
            self.rate = Rate(dt=cfg["environment"]["physics_dt"])
        else:
            self.rate = Rate(is_disabled=True)

        
        #TODO 3rd March 2026 (for after Version 3)
        # ROSLabManager(s) and YamcsLabManager instances should just be self.environmnent_manager
        # ... EnvironmentManagerFactory (EMF), RobotManager, etc. no need to specify in the naming the protocol which is used
        self.YamcsLabManager = Yamcs_LMF(
            cfg, is_simulation_alive=self.simulation_app.is_running, close_simulation=self.simulation_app.close
        )

        self.YamcsRobotManager = Yamcs_RobotManager(cfg["environment"]["robots_settings"], cfg["mode"]["instance_conf"])

        if "terrain_manager" in cfg["environment"].keys():
            self.terrain_manager_conf: TerrainManagerConf = cfg["environment"]["terrain_manager"]
            self.deform_delay = self.terrain_manager_conf.moon_yard.deformation_engine.delay
            self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable
        else:
            self.enable_deformation = False

        # Preload the assets
        if cfg["environment"]["name"] == "LargeScale":
            height, quat = self.YamcsLabManager.LC.get_height_and_normal((0.0, 0.0, 0.0))
            self.YamcsRobotManager.RM.preload_robot_at_pose(self.world, (0, 0, height + 0.5), (1, 0, 0, 0))
        else:
            self.YamcsRobotManager.RM.preload_robot(self.world)
        self.YamcsLabManager.LC.add_robot_manager(self.YamcsRobotManager.RM)

        for i in range(100):
            self.world.step(render=True)
        self.world.reset()

    def run_simulation(self) -> None:
        """
        Runs the simulation.
        """

        self.timeline.play()
        while self.simulation_app.is_running():
            self.rate.reset()
            self.world.step(render=True)
            if self.world.is_playing():
                # Apply modifications to the lab only once the simulation step is finished
                # This is extremely important as modifying the stage during a simulation step
                # will lead to a crash.
                self.YamcsLabManager.periodic_update(dt=self.world.get_physics_dt())
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.YamcsLabManager.reset()
                    self.YamcsRobotManager.reset()
                self.YamcsLabManager.apply_modifications()
                if self.YamcsLabManager.trigger_reset:
                    self.YamcsRobotManager.reset()
                    self.YamcsLabManager.trigger_reset = False
                self.YamcsRobotManager.apply_modifications()
                if self.enable_deformation:
                    if self.world.current_time_step_index >= (self.deform_delay * self.world.get_physics_dt()):
                        self.YamcsLabManager.LC.deform_terrain()

            self.rate.sleep()
        self.world.stop()
        self.timeline.stop()
