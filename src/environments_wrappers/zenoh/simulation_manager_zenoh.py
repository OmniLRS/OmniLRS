__author__ = "Shamistan Karimov, Elian NEPPEL, Bach Nguyen"
__copyright__ = "Copyright 2023-26, JAOPS, Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from isaacsim import SimulationApp
from isaacsim.core.api.world import World
import omni
import time
from typing import Union
import logging
import asyncio

from asyncio_for_robotics.zenoh.sub import Sub

from src.environments.utils import set_moon_env_name
from src.physics.physics_scene import PhysicsSceneManager
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.environments_wrappers.zenoh.lunalab_zenoh import Zenoh_LunalabManager
from src.environments_wrappers.zenoh.lunaryard_zenoh import Zenoh_LunaryardManager
from src.environments_wrappers.zenoh.largescale_zenoh import Zenoh_LargeScaleManager
from src.environments_wrappers.zenoh.robot_manager_zenoh import Zenoh_RobotManager


logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

class Rate:
    """
    Creates a rate object that enables to sleep for a minimum amount of time between two iterations of a loop. If freq and dt are passed, the object will only use the information provided by dt.
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

            # If time delta is too low, sleep, else carry one.
            if delta < self.dt:
                to_sleep = self.dt - delta
                time.sleep(to_sleep)


class Zenoh_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(
        self,
        name: str,
        lab_manager: Union[Zenoh_LunalabManager, Zenoh_LunaryardManager, Zenoh_LargeScaleManager],
    ) -> None:
        """
        Registers a lab manager.

        Args:
            name (str): Name of the lab manager.
            lab_manager (Union[Zenoh_LunalabManager, Zenoh_LunaryardManager, Zenoh_LargeScaleManager]): Instance of the lab manager
        """

        self._lab_managers[name] = lab_manager
    
    def __call__(
        self,
        cfg: dict,
        **kwargs,
    ) -> Union[Zenoh_LunalabManager, Zenoh_LunaryardManager, Zenoh_LargeScaleManager]:
        """
        Returns an instance of the lab manager corresponding to the environment name.

        Args:
            cfg (dict): Configuration dictionary.
        
        Returns:
            Union[Zenoh_LunalabManager, Zenoh_LunaryardManager, Zenoh_LargeScaleManager]: Instance of the lab manager
        """

        return self._lab_managers[cfg["environment"]["name"]](
            environment_cfg=cfg["environment"],
            zenoh_cfg=cfg["mode"],
            **kwargs,
        )

Zenoh_LMF = Zenoh_LabManagerFactory()
Zenoh_LMF.register("Lunalab", Zenoh_LunalabManager)
Zenoh_LMF.register("Lunaryard", Zenoh_LunaryardManager)
Zenoh_LMF.register("LargeScale", Zenoh_LargeScaleManager)

class Zenoh_SimulationManager:
    """
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager thread
    - Running the robot manager thread
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

        # Setups the physics and acquires different interfaces to talk with Isaac
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

        # Lab manager thread
        self.ZenohLabManager = Zenoh_LMF(
            cfg, is_simulation_alive=self.simulation_app.is_running,
            close_simulation=self.simulation_app.close
        )

        self.ZenohRobotManager = Zenoh_RobotManager(cfg["environment"]["robots_settings"], cfg["mode"])

        if "terrain_manager" in cfg["environment"].keys():
            self.terrain_manager_conf: TerrainManagerConf = cfg["environment"]["terrain_manager"]
            self.deform_delay = self.terrain_manager_conf.moon_yard.deformation_engine.delay
            self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable
        else:
            self.enable_deformation = False

        # Preload the assets
        if cfg["environment"]["name"] == "LargeScale":
            height, quat = self.ZenohLabManager.LC.get_height_and_normal((0.0, 0.0, 0.0))
            self.ZenohLabManager.RM.preload_robot_at_pose(self.world, (0, 0, height + 0.5), (1, 0, 0, 0))
        else:
            self.ZenohRobotManager.RM.preload_robot(self.world)

            for t in self.ZenohLabManager.transports:
                t.start()
            self.ZenohLabManager.transports_inited = True

            for t in self.ZenohRobotManager.transports:
                t.start()
            self.ZenohRobotManager.transports_inited = True
                
        self.ZenohLabManager.LC.add_robot_manager(self.ZenohRobotManager.RM)

        for i in range(100):
            self.world.step(render=True)
        self.world.reset()
    
        self.entry_task_is_done = False


    async def _run_zenoh_sub(self):
        """
        Listens for Zenoh messages and enqueues them for processing
        in the main simulation loop.
        """
        sub = Sub(self.ZenohLabManager.rocks_randomize_keyexpr)
        
        try:
            async for sample in sub.listen_reliable():
                self.ZenohLabManager.randomize_rocks(sample)
        finally:
            sub.close()

    async def _entry_point(self):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._run_zenoh_sub())
            # tg.create_task(..)
            # tg.create_task(..)

    def _on_update(self, event) -> None:
        """
        Called once per app frame by Isaac Sim's update event stream.
        Runs synchronous simulation logic and drains the Zenoh queue.
        """
        # Note: cannot use asyncio.create_task() here because it will complain "RuntimeError: no running event loop" -> had to use older api
        if self._entry_task is None:
            self._entry_task = asyncio.ensure_future(self._entry_point())
  
        if self._entry_task.done():
            exc = self._entry_task.exception()
            if exc:
                logger.error(f"Exception: {repr(exc)}")

            # notify
            self.entry_task_is_done = True 
        
        self.rate.reset()

        if self.world.is_playing():
            self.ZenohLabManager.periodic_update(dt=self.world.get_physics_dt())
            if self.world.current_time_step_index == 0:
                self.world.reset()
                self.ZenohLabManager.reset()
                self.ZenohRobotManager.reset()
            self.ZenohLabManager.apply_modifications()
            if self.ZenohLabManager.trigger_reset:
                self.ZenohRobotManager.reset()
                self.ZenohLabManager.trigger_reset = False
            self.ZenohRobotManager.apply_modifications()
            if self.enable_deformation:
                if self.world.current_time_step_index >= (self.deform_delay * self.world.get_physics_dt()):
                    self.ZenohLabManager.LC.deform_terrain()

        if self.ZenohLabManager.transports_inited:
            self.ZenohLabManager.pub_sim_is_running(True)

        if self.ZenohRobotManager.transports_inited:
            self.ZenohRobotManager.publish_cameras()

        self.rate.sleep()

    def run_simulation(self) -> None:
        """
        Runs the simulation in async manner, using existing Isaac Sim's event loop.
        """

        self.timeline.play()
        self._entry_task = None

        # Register a per-frame callback with Isaac Sim's app (ref: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/events.html)
        _update_sub = self.simulation_app._app.get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="zenoh_sim_step"
        )

        # Block the main thread simply by pumping the app until it closes.
        # Isaac Sim's async engine drives the event loop between frames,
        # so all tasks (ours + internal ones) get proper scheduling.
        while self.simulation_app.is_running() and not self.entry_task_is_done:
            self.simulation_app.update()

        # Cleanup
        if self._entry_task is not None:
            self._entry_task.cancel()

        for t in self.ZenohRobotManager.transports:
            t.close()

        self.world.stop()
        self.timeline.stop()
