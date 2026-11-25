__author__ = "Antoine Richard, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import os

logger = logging.getLogger(__name__)

# Modes that spawn a robot from robots_settings.parameters (set by the robot yaml: robot=<name>)
MODES_WITH_ROBOT = ("ROS2", "Yamcs", "Zenoh")
# Husky USD with the ROS 2 action graphs (cfg/robot/husky.yaml uses a USD without graphs)
ROS2_HUSKY_USD = "assets/USD_Assets/robots/ros2_husky_PhysX_vlp16.usd"

# Isaac Sim extensions whose Python modules OmniLRS imports. Enabled explicitly so OmniLRS does not depend on the
# default app enabling them (or on other extensions pulling them in).
REQUIRED_EXTENSIONS = (
    "isaacsim.core.experimental.prims",
    "isaacsim.core.experimental.utils",
    "isaacsim.core.simulation_manager",
    "isaacsim.sensors.experimental.physics",
    "isaacsim.sensors.experimental.rtx",
    "omni.replicator.core",
)


def _enable_required_extensions() -> None:
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in REQUIRED_EXTENSIONS:
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)


def _wait_for_isaac_sim(simulation_app) -> None:
    """Finish the startup work of Isaac Sim before OmniLRS starts, then log `Isaac Sim ready after <seconds>s`.

    After `app ready`, the Kit material library still loads its MDL list during the next app updates. Waiting here
    keeps that Isaac Sim work out of the OmniLRS startup steps. The RTX ray tracing pipelines compile on the first
    rendered frame, which can come before or after this line (about 5 min on the CI GPU without a warm cache, see
    docs/todo_performance.md). The sanity tests (test/ci_artefacts) read the seconds of this line, which count from
    the Kit start, and the RTX compile from the Kit log.
    """
    import asyncio
    import time

    import omni.kit.app

    wait_start = time.monotonic()
    try:
        import omni.kit.material.library as material_library
    except ImportError:
        material_library = None
    if material_library is not None:
        # Returns once the MDL list is ready (or after 1000 app updates)
        task = asyncio.ensure_future(material_library.get_mdl_list_async())
        while not task.done():
            simulation_app.update()
    logger.info(
        "Isaac Sim ready after %.3fs since the Kit start (material library ready in %.1fs). OmniLRS startup begins.",
        omni.kit.app.get_app().get_time_since_start_s(),
        time.monotonic() - wait_start,
    )


def _require_robot(cfg: dict) -> None:
    """Stop before Isaac Sim starts if a mode that spawns a robot has none: the environment yamls have no robot."""
    mode = cfg["mode"]["name"]
    if mode not in MODES_WITH_ROBOT:
        return
    if (cfg["environment"].get("robots_settings") or {}).get("parameters"):
        return
    hint = (
        " The Husky needs its ROS 2 USD in this mode: robot=husky "
        f"environment.robots_settings.parameters.usd_path={ROS2_HUSKY_USD}"
        if mode == "ROS2"
        else ""
    )
    raise ValueError(f"mode={mode} needs a robot: pass robot=<name> (a file in cfg/robot), e.g. robot=husky.{hint}")


def _set_ros_domain_id(cfg: dict) -> None:
    """ROS2 mode: set the ROS_DOMAIN_ID environment variable to mode.ROS_DOMAIN_ID (cfg/mode/ROS2.yaml) if it is not set.

    The OmniLRS nodes (rclpy) and the ROS2Context nodes of the robot graphs (useDomainIDEnvVar) read the variable.
    Without it, a robot graph uses the domain_id of its USD. The variable wins.
    """
    if cfg["mode"]["name"] != "ROS2" or "ROS_DOMAIN_ID" not in cfg["mode"]:
        return
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = str(cfg["mode"]["ROS_DOMAIN_ID"])
    logger.info("ROS 2 domain: ROS_DOMAIN_ID=%s", os.environ["ROS_DOMAIN_ID"])


def startSim(cfg: dict):
    _require_robot(cfg)
    _set_ros_domain_id(cfg)

    from isaacsim import SimulationApp

    from src.environments.rendering import (
        set_chromatic_aberrations,
        set_lens_flares,
        set_motion_blur,
        set_tbn_frame_mode,
    )

    class SimulationApp_wait(SimulationApp):
        def __init__(self, launch_config: dict = None, experience: str = "") -> None:
            super().__init__(launch_config, experience)
            self.wait_for_threads = []

        def add_wait(self, waiting_functions: list) -> None:
            """
            Adds a list of functions that will wait until a condition is met before closing the simulation.
            """
            self.wait_for_threads += waiting_functions

        def close(self, wait_for_replicator: bool = True, **kwargs) -> None:
            """Joins the registered external threads, then lets Isaac Sim close the application.

            Isaac Sim's own close() stops/flushes Replicator, closes the stage, and shuts Kit down
            (fast shutdown with a watchdog); it also ignores repeated calls.
            """
            if not self._exiting:
                for wait in self.wait_for_threads:
                    self._app.print_and_log(f"Waiting for external thread to join: {wait}")
                    wait()
                self.wait_for_threads = []
            super().close(wait_for_replicator=wait_for_replicator, **kwargs)

    # Starts the simulation and allows to import things related to Isaac and PXR
    renderer_cfg = cfg["rendering"]["renderer"]
    launch_config = renderer_cfg.__dict__.copy()
    tbn_frame_mode = launch_config.pop("tbn_frame_mode")
    simulation_app = SimulationApp_wait(launch_config)
    _enable_required_extensions()
    set_tbn_frame_mode(tbn_frame_mode)
    set_lens_flares(cfg)
    set_motion_blur(cfg)
    set_chromatic_aberrations(cfg)
    _wait_for_isaac_sim(simulation_app)

    # NOTE mode is chosen by setting the value of "mode" property within "defaults" @cfg/config.yaml
    # it can also be overwritten through console, same as environment
    # mode is required to exist also @cfg/mode/[mode_name].yaml with "name" property

    # Zenoh mode
    if cfg["mode"]["name"] == "Zenoh":
        from src.environments_wrappers.zenoh.simulation_manager_zenoh import (
            Zenoh_SimulationManager,
        )

        SM = Zenoh_SimulationManager(cfg, simulation_app)

    # Omits ROS settings, purely Yamcs-native
    if cfg["mode"]["name"] == "Yamcs":
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.yamcs.simulation_manager_yamcs import (
            Yamcs_SimulationManager,
        )

        SM = Yamcs_SimulationManager(cfg, simulation_app)

    # Starts the ROS2 extension. Allows to import ROS2 related things.
    if cfg["mode"]["name"] == "ROS2":
        # ROS2 startup routine
        from src.environments_wrappers.ros2 import enable_ros2

        enable_ros2(simulation_app, bridge_name=cfg["mode"]["bridge_name"])
        import rclpy

        rclpy.init()
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.ros2.simulation_manager_ros2 import (
            ROS2_SimulationManager,
        )

        SM = ROS2_SimulationManager(cfg, simulation_app)

    # dataset=<name> (cfg/dataset) merges dataset_settings into the mode node; only SDG consumes it.
    if "dataset_settings" in cfg["mode"] and cfg["mode"]["name"] != "SDG":
        raise ValueError(f"dataset= requires mode=SDG, got mode={cfg['mode']['name']}")

    # Starts the replicator stuff. Allows to acquire synthetic data.
    if cfg["mode"]["name"] == "SDG":
        if "dataset_settings" in cfg["mode"]:
            # Dataset generation (terrains x frames with a camera rig). See docs/sdg_dataset.md.
            from src.environments_wrappers.sdg.dataset.manager import SDGDataset_SimulationManager

            SM = SDGDataset_SimulationManager(cfg, simulation_app)
        else:
            # Call to the environment factory to load the correct environment.
            from src.environments_wrappers.sdg.simulation_manager_sdg import (
                SDG_SimulationManager,
            )

            SM = SDG_SimulationManager(cfg, simulation_app)

    return SM, simulation_app
