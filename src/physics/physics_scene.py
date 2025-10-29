__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

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
