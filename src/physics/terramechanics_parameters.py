__author__ = "Junnosuke Kamohara"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import math
from dataclasses import dataclass, field


@dataclass
class RobotParameter:
    """
    Robot mechanical dimension parameters."""

    mass: float = 20.0
    num_wheels: int = 4
    wheel_radius: float = 0.09
    wheel_base: float = 0.1
    wheel_lug_height: float = 0.02


@dataclass
class TerrainMechanicalParameter:
    """
    Terrain mechanical parameters.
    - bearing_parameter: (k_c, k_phi, n_0, n_1) bearing parameters.
    - shearing_parameter: (c, phi, K) shearing parameters."""

    k_c: float = 0.0
    k_phi: float = 100.0
    a_0: float = 0.4
    a_1: float = 0.3
    n: float = 0.7
    c: float = 0.0
    phi: float = 30.0 * math.pi / 180.0
    K: float = 0.03
    rho: float = 1400.0
    # m/s^2, from the physics scene (cfg/physics): Simulation's physics_scene_gravity(). No default, so that the model
    # never uses another gravity than the simulation.
    gravity: float = field(kw_only=True)
