__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"


def startSim(cfg: dict):
    from isaacsim import SimulationApp
    import omni
    from src.environments.rendering import set_lens_flares, set_chromatic_aberrations, set_motion_blur

    class SimulationApp_wait(SimulationApp):
        def __init__(self, launch_config: dict = None, experience: str = "") -> None:
            super().__init__(launch_config, experience)
            self.wait_for_threads = []

        def add_wait(self, waiting_functions: list) -> None:
            """
            Adds a list of functions that will wait until a condition is met before closing the simulation.
            """
            self.wait_for_threads += waiting_functions

        def close(self, wait_for_replicator=True) -> None:
            """Close the running Omniverse Toolkit."""
            try:
                # make sure that any replicator workflows finish rendering/writing
                import omni.replicator.core as rep

                if rep.orchestrator.get_status() not in [
                    rep.orchestrator.Status.STOPPED,
                    rep.orchestrator.Status.STOPPING,
                ]:
                    rep.orchestrator.stop()
                if wait_for_replicator:
                    rep.orchestrator.wait_until_complete()

                # Disable capture on play to avoid replicator engaging on any new timeline events
                rep.orchestrator.set_capture_on_play(False)
            except Exception:
                pass

            for wait in self.wait_for_threads:
                self._app.print_and_log(f"Waiting for external thread to join: {wait}")
                wait()

            # workaround for exit issues, clean the stage first:
            if omni.usd.get_context().can_close_stage():
                omni.usd.get_context().close_stage()
            # omni.kit.app.get_app().update()
            # check if exited already
            if not self._exiting:
                self._exiting = True
                self._app.print_and_log("Simulation App Shutting Down")

                # We are exisitng but something is still loading, wait for it to load to avoid a deadlock
                def is_stage_loading() -> bool:
                    """Convenience function to see if any files are being loaded.
                    bool: Convenience function to see if any files are being loaded. True if loading, False otherwise
                    """
                    import omni.usd

                    context = omni.usd.get_context()
                    if context is None:
                        return False
                    else:
                        _, _, loading = context.get_stage_loading_status()
                        return loading > 0

                if is_stage_loading():
                    print(
                        "   Waiting for USD resource operations to complete (this may take a few seconds), use Ctrl-C to exit immediately"
                    )
                while is_stage_loading():
                    self._app.update()

                self._app.shutdown()
                # disabled on linux to workaround issues where unloading plugins causes carb to fail
                self._framework.unload_all_plugins()
                # Force all omni module to unload on close
                # This prevents crash on exit
                # for m in list(sys.modules.keys()):
                #     if "omni" in m and m != "omni.kit.app":
                #         del sys.modules[m]
                print("Simulation App Shutdown Complete")

    # Starts the simulation and allows to import things related to Isaac and PXR
    renderer_cfg = cfg["rendering"]["renderer"]
    simulation_app = SimulationApp_wait(renderer_cfg.__dict__)
    set_lens_flares(cfg)
    set_motion_blur(cfg)
    set_chromatic_aberrations(cfg)

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

    # Starts the ROS1 extension. Allows to import ROS1 related things.
    if cfg["mode"]["name"] == "ROS1":
        # ROS1 startup routine
        from src.environments_wrappers.ros1 import enable_ros1

        enable_ros1(simulation_app)
        import rospy

        rospy.init_node("omni_isaac_ros1")
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.ros1.simulation_manager_ros1 import (
            ROS1_SimulationManager,
        )

        SM = ROS1_SimulationManager(cfg, simulation_app)

    # Starts the replicator stuff. Allows to acquire synthetic data.
    if cfg["mode"]["name"] == "SDG":
        # Call to the environment factory to load the correct environment.
        from src.environments_wrappers.sdg.simulation_manager_sdg import (
            SDG_SimulationManager,
        )

        SM = SDG_SimulationManager(cfg, simulation_app)

    return SM, simulation_app
