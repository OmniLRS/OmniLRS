__author__ = "Antoine Richard"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from src.configurations.physics_confs import PhysicsSceneConf

from isaacsim.core.api.physics_context.physics_context import PhysicsContext


class PhysicsSceneManager:
    def __init__(self, settings: PhysicsSceneConf) -> None:
        self.settings = settings
        self.physics_context = PhysicsContext(sim_params=self.settings.physics_scene_args, set_defaults=True)
        if self.settings.enable_ccd:
            self.physics_context.enable_ccd(True)
        if self.settings.broadphase_type is not None:
            self.physics_context.set_broadphase_type(self.settings.broadphase_type)
        if self.settings.solver_type is not None:
            self.physics_context.set_solver_type(self.settings.solver_type)
