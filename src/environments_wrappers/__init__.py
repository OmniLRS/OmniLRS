__author__ = "Antoine Richard, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

# Deprecated Isaac Sim extensions OmniLRS still uses. Since Isaac Sim 6.0 they ship under extsDeprecated
# and are not guaranteed to be enabled by the default app.
LEGACY_EXTENSIONS = (
    "isaacsim.core.api",
    "isaacsim.core.prims",
    "isaacsim.core.utils",
    "isaacsim.sensors.camera",
)


def _enable_legacy_extensions() -> None:
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in LEGACY_EXTENSIONS:
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)


def startSim(cfg: dict):
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
    _enable_legacy_extensions()
    set_tbn_frame_mode(tbn_frame_mode)
    set_lens_flares(cfg)
    set_motion_blur(cfg)
    set_chromatic_aberrations(cfg)

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

    # Starts the replicator stuff. Allows to acquire synthetic data.
    if cfg["mode"]["name"] == "SDG":
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.sdg.simulation_manager_sdg import (
            SDG_SimulationManager,
        )

        SM = SDG_SimulationManager(cfg, simulation_app)

    return SM, simulation_app
