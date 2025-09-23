__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import dataclasses
from typing import Any, Union, List


@dataclasses.dataclass
class CraterGeneratorConf:
    profiles_path: str = dataclasses.field(default_factory=str)
    min_xy_ratio: float = dataclasses.field(default_factory=float)
    max_xy_ratio: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    pad_size: int = dataclasses.field(default_factory=int)
    random_rotation: bool = dataclasses.field(default_factory=bool)
    z_scale: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        print(self.__dict__)
        assert type(self.profiles_path) is str, "profile_path must be a string"
        assert type(self.min_xy_ratio) is float, "min_xy_ratio must be a float"
        assert type(self.max_xy_ratio) is float, "max_xy_ratio must be a float"
        assert type(self.resolution) is float, "resolution must be a float"
        assert type(self.pad_size) is int, "pad_size must be an integer"
        assert type(self.random_rotation) is bool, "random_rotation must be a boolean"
        assert type(self.z_scale) is float, "z_scale must be a float"
        assert type(self.seed) is int, "seed must be an integer"

        assert self.min_xy_ratio <= self.max_xy_ratio, "min_xy_ratio must be smaller than max_xy_ratio"
        assert self.min_xy_ratio > 0, "min_xy_ratio must be greater than 0"
        assert self.max_xy_ratio > 0, "max_xy_ratio must be greater than 0"
        assert self.min_xy_ratio <= 1, "min_xy_ratio must be smaller than 1"
        assert self.max_xy_ratio <= 1, "max_xy_ratio must be smaller than 1"
        assert self.resolution > 0, "resolution must be greater than 0"
        assert self.pad_size >= 0, "pad_size must be greater or equal to 0"
        assert self.z_scale > 0, "z_scale must be greater than 0"


@dataclasses.dataclass
class CraterDistributionConf:
    x_size: float = dataclasses.field(default_factory=float)
    y_size: float = dataclasses.field(default_factory=float)
    densities: list = dataclasses.field(default_factory=list)
    radius: list = dataclasses.field(default_factory=list)
    radius_range: list = dataclasses.field(default_factory=list)
    dist_exp: float = dataclasses.field(default_factory=float)
    dist_coef: float = dataclasses.field(default_factory=float)
    radius_step: float = dataclasses.field(default_factory=float)
    radius_step_factor: float = dataclasses.field(default_factory=float)
    num_repeat: int = dataclasses.field(default_factory=int)
    seed: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        assert type(self.x_size) is float, "x_size must be a float"
        assert type(self.y_size) is float, "y_size must be a float"
        assert type(self.num_repeat) is int, "num_repeat must be an integer"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.dist_exp) is float,"dist_exp must be float"
        assert type(self.dist_coef) is float,"dist_coef must be float"

        assert self.x_size > 0, "x_size must be greater than 0"
        assert self.y_size > 0, "y_size must be greater than 0"
        assert self.num_repeat >= 0, "num_repeat must be greater or equal to 0"
        assert self.dist_exp < 0, "density distribution exponential term must be less than 0"
        assert self.dist_coef > 0, "density distribution coefficient term must be greater than 0"


@dataclasses.dataclass
class BaseTerrainGeneratorConf:
    x_size: float = dataclasses.field(default_factory=float)
    y_size: float = dataclasses.field(default_factory=float)
    resolution: float = dataclasses.field(default_factory=float)
    max_elevation: float = dataclasses.field(default_factory=float)
    min_elevation: float = dataclasses.field(default_factory=float)
    seed: int = dataclasses.field(default_factory=int)
    z_scale: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert type(self.x_size) is float, "x_size must be a float"
        assert type(self.y_size) is float, "y_size must be a float"
        assert type(self.resolution) is float, "resolution must be a float"
        assert type(self.max_elevation) is float, "max_elevation must be a float"
        assert type(self.min_elevation) is float, "min_elevation must be a float"
        assert type(self.seed) is int, "seed must be an integer"
        assert type(self.z_scale) is float, "z_scale must be a float"

        assert self.x_size > 0, "x_size must be greater than 0"
        assert self.y_size > 0, "y_size must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"
        assert self.max_elevation > self.min_elevation, "max_elevation must be greater than min_elevation"
        assert self.z_scale > 0, "z_scale must be greater than 0"


@dataclasses.dataclass
class FootprintConf:
    """
    Footprint dimension parameters.
    We use FLU (Front, Left, Up) coordinate system for the footprint.
    Args:
        width (float): Width of the footprint.
        height (float): Height of the footprint.
        shape (str): Shape of the footprint.
    """

    width: float = 0.5
    height: float = 0.25
    shape: str = "rectangle"

    def __post_init__(self):
        assert type(self.width) is float, "wheel_width must be a float"
        assert type(self.height) is float, "wheel_radius must be a float"
        assert type(self.shape) is str, "shape must be a string"
        assert self.width > 0, "wheel_width must be greater than 0"
        assert self.height > 0, "wheel_radius must be greater than 0"


@dataclasses.dataclass
class DeformConstrainConf:
    """
    Deformation constrain parameters.
    Args:
        x_deform_offset (float): X offset betweem deformation center and contact point.
        y_deform_offset (float): Y offset betweem deformation center and contact point.
        deform_decay_ratio (float): Decay ratio of the deformation.
    """

    x_deform_offset: float = 0.0
    y_deform_offset: float = 0.0
    deform_decay_ratio: float = 0.01

    def __post_init__(self):
        assert type(self.x_deform_offset) is float, "deform_offset must be a float"
        assert type(self.y_deform_offset) is float, "deform_offset must be a float"
        assert type(self.deform_decay_ratio) is float, "deform_decay_ratio must be a float"
        assert self.deform_decay_ratio > 0, "deform_decay_ratio must be greater than 0"


@dataclasses.dataclass
class BoundaryDistributionConf:
    """
    Boundary distribution parameters.
    Args:
        distribution (str): Distribution of the boundary.
        angle_of_repose (float): Angle of repose of the boundary (only used for trapezoidal distribution)
    """

    distribution: str = "uniform"
    angle_of_repose: float = 1.047

    def __post_init__(self):
        assert type(self.distribution) is str, "distribution must be a string"
        assert type(self.angle_of_repose) is float, "angle_of_repose must be a float"
        assert self.angle_of_repose > 0, "angle_of_repose must be greater than 0"


@dataclasses.dataclass
class DepthDistributionConf:
    """
    Deformation depth distribution parameters.
    Args:
        distribution (str): Distribution of the force.
        wave_frequency (float): Frequency of the wave. Under no slip condition, this is num_grouser/pi
    """

    distribution: str = "uniform"
    wave_frequency: float = 1.0

    def __post_init__(self):
        assert type(self.distribution) is str, "distribution must be a string"
        assert self.wave_frequency > 0, "wave_frequency must be greater than 0"


@dataclasses.dataclass
class ForceDepthRegressionConf:
    """
    Force depth regression parameters.
    For now, linear regression.
    Args:
        amplitude_slope (float): Slope of the amplitude.
        amplitude_intercept (float): Intercept of the amplitude.
        mean_slope (float): Slope of the mean.
        mean_intercept (float): Intercept of the mean.
    """

    amplitude_slope: float = 1.0
    amplitude_intercept: float = 0.0
    mean_slope: float = 1.0
    mean_intercept: float = 0.0

    def __post_init__(self):
        assert type(self.amplitude_slope) is float, "slope must be a float"
        assert type(self.amplitude_intercept) is float, "intercept must be a float"
        assert type(self.mean_slope) is float, "slope must be a float"
        assert type(self.mean_intercept) is float, "intercept must be a float"


@dataclasses.dataclass
class DeformationEngineConf:
    """
    Deformation engine parameters.
    Args:
        enable (bool): Enable deformation.
        delay (float): Delay time (s) for the deformation.
        terrain_resolution (float): Resolution of the terrain.
        terrain_width (float): Width of the terrain.
        terrain_height (float): Height of the terrain.
        gravity (list): Gravity vector.
        footprint (dict): Footprint parameters.
        deform_constrain (dict): Deformation constrain parameters.
        boundary_distribution (dict): Boundary distribution parameters.
        depth_distribution (dict): Deformation depth distribution parameters.
        force_depth_regression (dict): Force depth regression parameters.
        num_links (int): Total number of links = num_robot * num_target_links.
    """

    enable: bool = False
    delay: float = 1.0
    terrain_resolution: float = dataclasses.field(default_factory=float)
    terrain_width: float = dataclasses.field(default_factory=float)
    terrain_height: float = dataclasses.field(default_factory=float)
    gravity: List[float] = dataclasses.field(default_factory=list)
    footprint: FootprintConf = dataclasses.field(default_factory=dict)
    deform_constrain: DeformConstrainConf = dataclasses.field(default_factory=dict)
    boundary_distribution: BoundaryDistributionConf = dataclasses.field(default_factory=dict)
    depth_distribution: DepthDistributionConf = dataclasses.field(default_factory=dict)
    force_depth_regression: ForceDepthRegressionConf = dataclasses.field(default_factory=dict)
    num_links: int = 4

    def __post_init__(self):
        assert type(self.delay) is float, "delay must be float"
        assert type(self.terrain_resolution) is float, "terrain_resolution must be a float"
        assert self.delay >= 0, "render_deform_inv must be greater than or equal to 1"
        assert self.terrain_resolution > 0, "terrain_resolution must be greater than 0"
        assert self.terrain_width > 0, "terrain_width must be greater than 0"
        assert self.terrain_height > 0, "terrain_height must be greater than 0"
        assert self.num_links > 0, "num_links must be greater than 0"

        self.footprint = FootprintConf(**self.footprint)
        self.deform_constrain = DeformConstrainConf(**self.deform_constrain)
        self.boundary_distribution = BoundaryDistributionConf(**self.boundary_distribution)
        self.depth_distribution = DepthDistributionConf(**self.depth_distribution)
        self.force_depth_regression = ForceDepthRegressionConf(**self.force_depth_regression)


@dataclasses.dataclass
class MoonYardConf:
    crater_generator: CraterGeneratorConf = None
    crater_distribution: CraterDistributionConf = None
    base_terrain_generator: BaseTerrainGeneratorConf = None
    deformation_engine: DeformationEngineConf = None
    is_yard: bool = dataclasses.field(default_factory=bool)
    is_lab: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        self.crater_generator = CraterGeneratorConf(**self.crater_generator)
        self.crater_distribution = CraterDistributionConf(**self.crater_distribution)
        self.base_terrain_generator = BaseTerrainGeneratorConf(**self.base_terrain_generator)
        self.deformation_engine = DeformationEngineConf(**self.deformation_engine)

        assert type(self.is_yard) is bool, "is_yard must be a boolean"
        assert type(self.is_lab) is bool, "is_lab must be a boolean"


@dataclasses.dataclass
class TerrainManagerConf:
    moon_yard: MoonYardConf = None
    root_path: str = dataclasses.field(default_factory=str)
    texture_path: str = dataclasses.field(default_factory=str)
    dems_path: str = dataclasses.field(default_factory=str)
    mesh_position: tuple = dataclasses.field(default_factory=tuple)
    mesh_orientation: tuple = dataclasses.field(default_factory=tuple)
    mesh_scale: tuple = dataclasses.field(default_factory=tuple)
    sim_length: int = dataclasses.field(default_factory=int)
    sim_width: int = dataclasses.field(default_factory=int)
    resolution: float = dataclasses.field(default_factory=float)
    augmentation: bool = False

    def __post_init__(self):
        self.moon_yard = MoonYardConf(**self.moon_yard)

        assert type(self.root_path) is str, "root_path must be a string"
        assert type(self.texture_path) is str, "texture_path must be a string"
        assert type(self.dems_path) is str, "dems_path must be a string"
        assert type(self.sim_length) is float, "sim_length must be a float"
        assert type(self.sim_width) is float, "sim_width must be a float"
        assert type(self.resolution) is float, "resolution must be a float"

        assert len(self.mesh_position) == 3, "mesh_position must be a tuple of length 3"
        assert len(self.mesh_orientation) == 4, "mesh_orientation must be a tuple of length 4"
        assert len(self.mesh_scale) == 3, "mesh_scale must be a tuple of length 3"
        assert self.sim_length > 0, "sim_length must be greater than 0"
        assert self.sim_width > 0, "sim_width must be greater than 0"
        assert self.resolution > 0, "resolution must be greater than 0"
