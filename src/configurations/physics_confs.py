__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import Tuple
import dataclasses


@dataclasses.dataclass
class PhysicsSceneConf:
    dt: float = dataclasses.field(default_factory=float)
    gravity: Tuple[float, float, float] = None
    substeps: int = None
    use_gpu_pipeline: bool = None
    worker_thread_count: int = None
    use_fabric: bool = None
    enable_scene_query_support: bool = None
    gpu_max_rigid_contact_count: int = None
    gpu_max_rigid_patch_contact_count: int = None
    gpu_found_lost_pairs_capacity: int = None
    gpu_total_aggregate_pairs_capacity: int = None
    gpu_max_soft_body_contacts: int = None
    gpu_max_particle_contacts: int = None
    gpu_heap_capacity: int = None
    gpu_temp_buffer_capacity: int = None
    gpu_max_num_partions: int = None
    gpu_collision_stack_size: int = None
    solver_type: str = None
    broadphase_type: str = None
    enable_stabilization: bool = None
    bounce_threshold_velocity: float = None
    friction_offset_threshold: float = None
    friction_correlation_distance: float = None
    enable_ccd: bool = None

    def __post_init__(self):
        self.physics_scene_args = {}
        for attribute in dataclasses.fields(self):
            if getattr(self, attribute.name) is not None:
                self.physics_scene_args[attribute.name] = getattr(self, attribute.name)

        if self.broadphase_type is not None:
            assert self.broadphase_type in ["SAP", "MBP", "GPU"]
        if self.solver_type is not None:
            assert self.solver_type in ["PGS", "TGS"]
