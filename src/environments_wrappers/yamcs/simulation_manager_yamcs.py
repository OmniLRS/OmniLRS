__author__ = "Aleksa Stanivuk, Shamistan Karimov, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time
from typing import Union

import omni
from isaacsim import SimulationApp
from isaacsim.core.api.world import World
from yamcs.client import YamcsClient

from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.large_scale_lunar import LargeScaleController
from src.environments.lunalab import LunalabController
from src.environments.lunaryard import LunaryardController
from src.environments.stellar_engine_env_mixin import StellarEngineEnvMixin
from src.environments.utils import set_moon_env_name
from src.environments_wrappers.rate import Rate
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")


class Yamcs_SimulationManager:
    """
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the environment manager
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

        # Block here until the Yamcs server is reachable. This avoids the
        # ConnectionFailure crash users hit when launching the sim before the
        # Yamcs server is up. We do this before creating the World/scene so no
        # simulation state can drift while we wait.
        self._wait_for_yamcs(cfg["mode"]["instance_conf"]["address"])

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

        self.RM = RobotManager(cfg["environment"]["robots_settings"], mode=SimulatorMode.YAMCS)

        self._setup_terrain_manager()
        self._preload_robot()
        if isinstance(self.EC, StellarEngineEnvMixin) and self.RM.robot.subsystems is not None:
            # True for Lunaryard and LargeScale, False for Lunalab
            # subsystems uses sun directions to calculate views no matter if stellar engine is enabled (sun moves) or not
            # subsystems is only initialized for mission-specific robots (e.g. pragyaan); skip otherwise.
            self.RM.robot.subsystems.set_sun_prim_path(self.EC.get_sun_prim_path())
        self._start_TMTC()
        self.EC.add_robot_manager(self.RM)
        self._step_world_and_reset()

    def _wait_for_yamcs(self, address: str, poll_interval: float = 1.0) -> None:
        """
        Block until the Yamcs server at ``address`` responds, polling every
        ``poll_interval`` seconds. Pumps ``simulation_app.update()`` between
        polls so the Isaac Sim window stays responsive while waiting.
        """
        # Silence urllib3's per-connection DEBUG spam during polling. The
        # main YamcsTMTC module also sets this, but it hasn't been imported yet.
        logging.getLogger("urllib3").setLevel(logging.WARNING)

        probe_client = YamcsClient(address)
        attempt = 0
        announced_waiting = False
        while True:
            try:
                probe_client.get_server_info()
                print(f"[Yamcs] Server at {address} is reachable. Continuing startup.", flush=True)
                return
            except Exception as e:
                if not announced_waiting:
                    print(
                        f"[Yamcs] Server at {address} is not reachable yet ({e.__class__.__name__}). "
                        f"Waiting... Press Ctrl-C to abort.",
                        flush=True,
                    )
                    announced_waiting = True
                elif attempt % 10 == 0:
                    # Re-announce every ~10 polls so the user knows we're still waiting.
                    print(f"[Yamcs] Still waiting for server at {address}...", flush=True)
            attempt += 1
            # Keep the Isaac Sim window responsive while we wait.
            try:
                self.simulation_app.update()
            except Exception:
                pass
            time.sleep(poll_interval)

    def _start_TMTC(self):
        robot_name = self.RM.robot.robot_name.replace("/", "")
        controller_name = self.RM.RM_conf.robot_controller

        if controller_name == "pragyaan-controller":
            from src.mission_specific.pragyaan.tmtc.pragyaan_controller import PragyaanController

            self.TMTC = PragyaanController(
                self.cfg["mode"]["instance_conf"],
                self.RM.RM_conf.yamcs_tmtc,
                robot_name,
                self.RM.robot_RG,
                self.RM.robot,
            )
        elif controller_name == "husky-controller":
            from src.mission_specific.husky.tmtc.husky_controller import HuskyController

            self.TMTC = HuskyController(
                self.cfg["mode"]["instance_conf"],
                self.RM.RM_conf.yamcs_tmtc,
                robot_name,
                self.RM.robot_RG,
                self.RM.robot,
            )
        elif controller_name == "":
            raise Exception("No robot controller was setup in yaml configurations.")
        else:
            raise Exception("Settings for '" + str(controller_name) + "' robot controller are not specified.")

        self.TMTC.setup_command_callbacks(self.RM.RM_conf.yamcs_tmtc["commands"])
        self.TMTC.start_streaming_data()

    def _get_environment_controller(self, environment_name: str):
        self.EC = None
        if environment_name == "LargeScale":
            self.EC = LargeScaleController(
                mode=SimulatorMode.YAMCS,
                **self.cfg["environment"],
                is_simulation_alive=self.simulation_app.is_running,
                close_simulation=self.simulation_app.close,
            )
        elif environment_name == "Lunaryard":
            self.EC = LunaryardController(
                mode=SimulatorMode.YAMCS,
                **self.cfg["environment"],
                is_simulation_alive=self.simulation_app.is_running,
                close_simulation=self.simulation_app.close,
            )
        elif environment_name == "Lunalab":
            self.EC = LunalabController(
                mode=SimulatorMode.YAMCS,
                **self.cfg["environment"],
                is_simulation_alive=self.simulation_app.is_running,
                close_simulation=self.simulation_app.close,
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
            robot_pos = self.RM.robot_parameters.pose.position
            robot_ori = self.RM.robot_parameters.pose.orientation
            height, _ = self.EC.get_height_and_normal((robot_pos[0], robot_pos[1], 0.0))
            self.RM.preload_robot_at_pose(self.world, (robot_pos[0], robot_pos[1], height + 0.5), robot_ori)

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
                did_reset = False

                if hasattr(self.EC, "enable_stellar_engine") and self.EC.enable_stellar_engine:
                    self.EC.update_stellar_engine(dt=self.world.get_physics_dt())

                if self.world.current_time_step_index == 0:
                    self.world.reset()

                    did_reset = True

                    if self.RM.robot is not None:
                        self.RM.robot.invalidate_articulation_api()

                if not did_reset and self.RM.robot is not None:
                    self.RM.robot.update_articulation_api()

            self.rate.sleep()

        if self.RM.robot is not None:
            self.RM.robot.invalidate_articulation_api()

        self.world.stop()
        self.timeline.stop()
