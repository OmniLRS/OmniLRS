from __future__ import annotations

__author__ = "Louis Burtz"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.physics.core
import omni.timeline
from isaacsim.core.simulation_manager import PhysxScene, SimulationManager
from pxr import PhysxSchema, Usd

from src.configurations.physics_confs import PhysicsSceneConf

_PLAY_SIMULATIONS = "/app/player/playSimulations"


class Simulation:
    """
    Simulation lifecycle for OmniLRS, built on SimulationManager.

    Replaces the deprecated isaacsim.core.api World/SimulationContext (Isaac Sim 6.0 migration guide: stage utils
    for the stage, SimulationManager for physics, the timeline for play/stop). It keeps the behavior OmniLRS relied
    on: a CPU PhysX scene at `physics_prim_path`, Kit's main loop in manual mode stepping `rendering_dt` per update
    (physics substeps follow from `physics_dt`), and a physics step counter that resets when the timeline stops.

    Args:
        physics_dt (float): Physics step in seconds.
        rendering_dt (float): Rendering (app update) step in seconds.
        physics_scene (PhysicsSceneConf, optional): Physics scene settings to apply on top of the defaults.
        physics_prim_path (str): Path of the physics scene prim.
    """

    def __init__(
        self,
        physics_dt: float,
        rendering_dt: float,
        physics_scene: PhysicsSceneConf | None = None,
        physics_prim_path: str = "/physicsScene",
    ) -> None:
        self._app = omni.kit.app.get_app()
        self._settings = carb.settings.get_settings()
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.set_auto_update(True)
        if self._timeline.is_playing():
            self.stop()

        SimulationManager.set_backend("numpy")
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        self.render()

        self._scene = PhysxScene(physics_prim_path)
        SimulationManager.set_default_physics_scene(physics_prim_path)
        self._set_cpu_defaults()
        self._set_physics_dt(physics_dt)
        self._set_simulation_dt(physics_dt, rendering_dt)
        if physics_scene is not None:
            self.configure_physics_scene(physics_scene)

        self._num_steps = 0
        self._physics_step_sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=False, order=0, on_update=self._on_physics_step
        )
        self._timeline_stop_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._on_timeline_stop,
            observer_name="omnilrs.Simulation._on_timeline_stop",
        )

    @property
    def physics_scene(self) -> PhysxScene:
        return self._scene

    @property
    def current_time_step_index(self) -> int:
        """Number of physics steps since the timeline was last stopped."""
        return self._num_steps

    def get_physics_dt(self) -> float:
        return self._scene.get_dt()

    def is_playing(self) -> bool:
        return self._timeline.is_playing()

    def step(self) -> None:
        """Advance the app by one update: renders and runs the physics substeps of one rendering_dt."""
        self._app.update()

    def render(self) -> None:
        """Update the app without stepping physics."""
        self._settings.set_bool(_PLAY_SIMULATIONS, False)
        self._app.update()
        self._settings.set_bool(_PLAY_SIMULATIONS, True)

    def play(self) -> None:
        self._timeline.play()
        self._timeline.commit()
        self.render()

    def stop(self) -> None:
        self._timeline.stop()
        self.render()

    def reset(self) -> None:
        """Stop and restart the timeline, then make sure the physics views exist."""
        self.stop()
        self.play()
        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()

    def configure_physics_scene(self, conf: PhysicsSceneConf) -> None:
        """Apply OmniLRS physics scene settings (cfg/physics)."""
        args = conf.physics_scene_args
        api = PhysxSchema.PhysxSceneAPI(self._scene.prim)
        if "gravity" in args:
            self._scene.set_gravity(args["gravity"])
        if "dt" in args:
            self._set_physics_dt(args["dt"], args.get("substeps"))
            stage = stage_utils.get_current_stage(backend="usd")
            with Usd.EditContext(stage, Usd.EditTarget(stage.GetRootLayer())):
                stage.SetTimeCodesPerSecond(1 / args["dt"])
        self._settings.set_bool("/physics/suppressReadback", bool(args.get("use_gpu_pipeline", False)))
        if "worker_thread_count" in args:
            self._settings.set_int("/persistent/physics/numThreads", args["worker_thread_count"])
        if args.get("use_fabric"):
            SimulationManager.enable_fabric(True)
        if "enable_scene_query_support" in args:
            api.CreateEnableSceneQuerySupportAttr().Set(args["enable_scene_query_support"])
        gpu_cfg = {
            key: args[key]
            for key in (
                "gpu_max_rigid_contact_count",
                "gpu_found_lost_pairs_capacity",
                "gpu_total_aggregate_pairs_capacity",
                "gpu_max_soft_body_contacts",
                "gpu_max_particle_contacts",
                "gpu_heap_capacity",
                "gpu_temp_buffer_capacity",
                "gpu_collision_stack_size",
            )
            if key in args
        }
        if gpu_cfg:
            self._scene.set_gpu_configuration(gpu_cfg)
        if "solver_type" in args:
            self._scene.set_solver_type(args["solver_type"])
        if "enable_stabilization" in args:
            self._scene.set_enabled_stabilization(args["enable_stabilization"])
        if "bounce_threshold_velocity" in args:
            api.CreateBounceThresholdAttr().Set(args["bounce_threshold_velocity"])
        if "friction_offset_threshold" in args:
            api.CreateFrictionOffsetThresholdAttr().Set(args["friction_offset_threshold"])
        if "friction_correlation_distance" in args:
            api.CreateFrictionCorrelationDistanceAttr().Set(args["friction_correlation_distance"])
        if conf.enable_ccd:
            self._scene.set_enabled_ccd(True)
        if conf.broadphase_type is not None:
            self._scene.set_broadphase_type(conf.broadphase_type)

    def _set_cpu_defaults(self) -> None:
        self._scene.set_broadphase_type("MBP")
        self._scene.set_enabled_gpu_dynamics(False)
        self._scene.set_gravity((0.0, 0.0, -9.81))
        self._scene.set_enabled_stabilization(False)
        self._scene.set_enabled_ccd(True)
        self._settings.set_bool("/physics/suppressReadback", False)
        self._scene.set_solver_type("TGS")
        self._set_physics_dt(1.0 / 60.0)

    def _set_physics_dt(self, dt: float, substeps: int | None = None) -> None:
        substeps = substeps if substeps and substeps > 1 else 1
        steps_per_second = int(1.0 / dt) if dt > 0 else 0
        self._scene.set_steps_per_second(steps_per_second)
        self._settings.set_int("/persistent/simulation/minFrameRate", int(steps_per_second / substeps))

    def _set_simulation_dt(self, physics_dt: float, rendering_dt: float) -> None:
        self._set_physics_dt(physics_dt, max(int(rendering_dt / physics_dt), 1) if physics_dt > 0 else 1)
        rendering_hz = 1.0 / rendering_dt if rendering_dt > 0 else 0
        if self._settings.get_as_bool("/app/runLoops/main/rateLimitEnabled"):
            self._settings.set_float("/app/runLoops/main/rateLimitFrequency", rendering_hz)
            self._timeline.set_target_framerate(rendering_hz)
        if rendering_hz > 0:
            stage = stage_utils.get_current_stage(backend="usd")
            with Usd.EditContext(stage, stage.GetRootLayer()):
                stage.SetTimeCodesPerSecond(rendering_hz)
            self._timeline.set_time_codes_per_second(rendering_hz)
        from omni.kit.loop import _loop as omni_loop

        loop_runner = omni_loop.acquire_loop_interface()
        loop_runner.set_manual_step_size(rendering_dt)
        loop_runner.set_manual_mode(True)

    def _on_physics_step(self, step_size: float, context: object) -> None:
        self._num_steps += 1

    def _on_timeline_stop(self, event: object) -> None:
        self._num_steps = 0
