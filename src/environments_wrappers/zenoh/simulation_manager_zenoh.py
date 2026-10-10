__author__ = "Shamistan Karimov, Elian NEPPEL, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging

import omni
from isaacsim import SimulationApp
from isaacsim.core.api.world import World

from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.environments.utils import set_moon_env_name
from src.environments_wrappers.rate import Rate
from src.environments_wrappers.zenoh.largescale_zenoh import Zenoh_LargeScaleManager
from src.environments_wrappers.zenoh.lunalab_zenoh import Zenoh_LunalabManager
from src.environments_wrappers.zenoh.lunaryard_zenoh import Zenoh_LunaryardManager
from src.environments_wrappers.zenoh.robot_manager_zenoh import Zenoh_RobotManager

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")


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

        self._step_world_and_reset()
        self._setup_rate()

        self.Zenoh_EC = self._get_environment_controller(self.cfg["environment"]["name"])
        assert self.Zenoh_EC is not None

        self.Zenoh_RM = Zenoh_RobotManager(cfg["environment"]["robots_settings"], cfg["mode"])

        self._setup_terrain_manager()
        self._preload_robot()

        self._start_transports()

        self.Zenoh_EC.LC.add_robot_manager(self.Zenoh_RM.get_RM())

        self._step_world_and_reset()

        self.entry_task_is_done = False

    def _step_world_and_reset(self, n=100):
        for i in range(n):
            self.world.step(render=True)
        self.world.reset()

    def _setup_rate(self):
        if self.cfg["environment"]["enforce_realtime"]:
            self.rate = Rate(dt=self.cfg["environment"]["physics_dt"])
        else:
            self.rate = Rate(is_disabled=True)

    def _get_environment_controller(self, environment_name: str):
        self.Zenoh_EC = None
        if environment_name == "LargeScale":
            self.Zenoh_EC = Zenoh_LargeScaleManager(
                environment_cfg=self.cfg["environment"],
                zenoh_cfg=self.cfg["mode"],
                is_simulation_alive=self.simulation_app.is_running,
                close_simulation=self.simulation_app.close,
            )
        elif environment_name == "Lunaryard":
            self.Zenoh_EC = Zenoh_LunaryardManager(
                environment_cfg=self.cfg["environment"],
                zenoh_cfg=self.cfg["mode"],
                is_simulation_alive=self.simulation_app.is_running,
                close_simulation=self.simulation_app.close,
            )
        elif environment_name == "Lunalab":
            self.Zenoh_EC = Zenoh_LunalabManager(
                environment_cfg=self.cfg["environment"],
                zenoh_cfg=self.cfg["mode"],
                is_simulation_alive=self.simulation_app.is_running,
                close_simulation=self.simulation_app.close,
            )

        return self.Zenoh_EC

    def _setup_terrain_manager(self):
        if "terrain_manager" in self.cfg["environment"].keys():
            self.terrain_manager_conf: TerrainManagerConf = self.cfg["environment"]["terrain_manager"]
            self.deform_delay = self.terrain_manager_conf.moon_yard.deformation_engine.delay
            self.enable_deformation = self.terrain_manager_conf.moon_yard.deformation_engine.enable
        else:
            self.enable_deformation = False

    def _preload_robot(self):
        if self.cfg["environment"]["name"] == "LargeScale":
            height, quat = self.Zenoh_EC.LC.get_height_and_normal((0.0, 0.0, 0.0))
            self.Zenoh_RM.get_RM().preload_robot_at_pose(self.world, (0, 0, height + 0.5), (1, 0, 0, 0))
        else:
            self.Zenoh_RM.get_RM().preload_robot(self.world)

    def _start_transports(self):
        """
        Start publishers & subcribers from both environment & robot controllers
        """

        self.Zenoh_EC.start_publishing()
        self.Zenoh_EC.start_listening()

        self.Zenoh_RM.start_publishing()
        self.Zenoh_RM.start_listening()

    def run_simulation(self) -> None:
        """
        Runs simulation in synchronous manner,
        while zenoh's asyncio tasks has already been started in backend.
        """

        self.timeline.play()

        while self.simulation_app.is_running():
            self.rate.reset()

            did_reset = False

            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    did_reset = True
                    self.Zenoh_EC.reset()
                    self.Zenoh_RM.invalidate_articulation_api()
                    self.Zenoh_RM.reset_robot()
                    self.Zenoh_RM.apply_modifications()

                if not did_reset:
                    # Must happen before periodic_update(), because LargeScale calls robot.get_pose().
                    self.Zenoh_RM.update_articulation_api()
                    self.Zenoh_EC.periodic_update(dt=self.world.get_physics_dt())
                    self.Zenoh_EC.apply_modifications()
                    if self.Zenoh_EC.trigger_reset:
                        self.Zenoh_RM.invalidate_articulation_api()
                        self.Zenoh_RM.reset()
                        self.Zenoh_EC.trigger_reset = False
                        did_reset = True
                    self.Zenoh_RM.apply_modifications()
                    if self.enable_deformation:
                        if self.world.current_time_step_index >= (self.deform_delay * self.world.get_physics_dt()):
                            self.Zenoh_EC.LC.deform_terrain()

            if self.Zenoh_EC.pubs_inited:
                self.Zenoh_EC.pub_sim_is_running(True)

            if self.Zenoh_RM.pubs_inited and self.world.is_playing() and not did_reset:
                self.Zenoh_RM.apply_modifications()
                self.Zenoh_RM.publish_telemetry()
                self.Zenoh_RM.publish_gt()

            self.rate.sleep()

            self.simulation_app.update()

        self.Zenoh_EC.close()
        self.Zenoh_RM.close()

        self.world.stop()
        self.timeline.stop()
