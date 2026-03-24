__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import time
from contextlib import contextmanager


class FrameProfiler:
    """
    Accumulates named per-frame timings and logs a summary periodically.
    Keys are discovered dynamically from track() calls.
    When disabled, track() is a bare no-op context manager.
    """

    def __init__(self, interval: int = 100, enabled: bool = True) -> None:
        self.enabled = enabled
        if enabled:
            self._interval = interval
            self._acc = {}
            self._n = 0
            self._frame_start = 0.0

    def begin_frame(self) -> None:
        if self.enabled:
            self._frame_start = time.perf_counter()

    @contextmanager
    def track(self, key: str):
        """Context manager that times the enclosed block under *key*.
        When profiling is disabled, yields immediately with no overhead."""
        if not self.enabled:
            yield
            return
        t0 = time.perf_counter()
        yield
        self._acc[key] = self._acc.get(key, 0.0) + (time.perf_counter() - t0)

    def end_frame(self) -> None:
        if not self.enabled:
            return
        self._acc["total"] = self._acc.get("total", 0.0) + (time.perf_counter() - self._frame_start)
        self._n += 1
        if self._n >= self._interval:
            n = self._n
            keys = [k for k in self._acc if k != "total"]
            all_keys = ["total"] + keys
            max_key_len = max(len(k) for k in all_keys)
            lines = [f"  {k:<{max_key_len}} = {1000*self._acc[k]/n:6.1f} ms" for k in all_keys]
            fps = n / self._acc["total"] if self._acc["total"] > 0 else 0
            lines.append(f"  {'fps':<{max_key_len}} = {fps:6.1f}")
            print(f"[PROFILE] avg over {n} frames:\n" + "\n".join(lines))
            self._acc = {}
            self._n = 0


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
    launch_config = renderer_cfg.__dict__
    # Troubleshooting: if cache issues arise e.g 100GB and warnings regarding garbage collection.
    # Delete the cache with:
    # rm -rf ~/docker/isaac-sim/cache/kit/DerivedDataCache
    # Uncomment these lines to disable caching entirely
    # launch_config["extra_args"] = [
    #     "--/UJITSO/enabled=false",
    # ]
    # launch_config["extra_args"] = [
    #     "--/UJITSO/datastore/localDataStore/largeChunkDiskBudgetMB=32768",
    # ]
    # Troubleshooting: enable these flags to show the visual mesh caching process.
    # launch_config["extra_args"] = [
    #     "--/UJITSO/failedDepLoadingLogging=true",
    #     "--/UJITSO/logBuildResults=true",
    #     "--/UJITSO/logInSingleLine=true",
    # ]
    simulation_app = SimulationApp_wait(launch_config)
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
