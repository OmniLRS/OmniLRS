__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import dataclasses


@dataclasses.dataclass
class FlaresConf:
    enable: bool = dataclasses.field(default_factory=bool)
    scale: float = dataclasses.field(default_factory=float)
    blades: int = dataclasses.field(default_factory=int)
    aperture_rotation: float = dataclasses.field(default_factory=float)
    sensor_diagonal: float = dataclasses.field(default_factory=float)
    sensor_aspect_ratio: float = dataclasses.field(default_factory=float)
    fstop: float = dataclasses.field(default_factory=float)
    focal_length: float = dataclasses.field(default_factory=float)

    def __post_init__(self):
        assert type(self.enable) is bool, "enalbe must be a boolean"
        assert type(self.scale) is float, "scale must be a float"
        assert type(self.blades) is int, "blades must be an integer"
        assert type(self.aperture_rotation) is float, "aperture_rotation must be a float"
        assert type(self.sensor_diagonal) is float, "sensor_diagonal must be a float"
        assert type(self.sensor_aspect_ratio) is float, "sensor_aspect_ratio must be a float"
        assert type(self.fstop) is float, "fstop must be a float"
        assert type(self.focal_length) is float, "focal_length must be a float"

        assert self.scale > 0, "scale must be greater than 0"
        assert self.blades > 0, "blades must be greater than 0"
        assert self.aperture_rotation >= 0, "aperture_rotation must be greater or equal to 0"
        assert self.aperture_rotation <= 360, "aperture_rotation must be smaller or equal to 360"
        assert self.sensor_diagonal > 0, "sensor_diagonal must be greater than 0"
        assert self.sensor_aspect_ratio > 0, "sensor_aspect_ratio must be greater than 0"
        assert self.fstop > 0, "fstop must be greater than 0"
        assert self.focal_length > 0, "focal_length must be greater than 0"


@dataclasses.dataclass
class MotionBlurConf:
    enable: bool = dataclasses.field(default_factory=bool)
    max_blur_diameter_fraction: float = dataclasses.field(default_factory=float)
    exposure_fraction: float = dataclasses.field(default_factory=float)
    num_samples: int = dataclasses.field(default_factory=int)

    def __post_init__(self):
        assert type(self.enable) is bool, "enable must be a boolean"
        assert type(self.max_blur_diameter_fraction) is float, "max_blur_diameter_fraction must be a float"
        assert type(self.exposure_fraction) is float, "exposure_fraction must be a float"
        assert type(self.num_samples) is int, "num_samples must be an integer"

        assert self.max_blur_diameter_fraction > 0, "max_blur_diameter_fraction must be greater than 0"
        assert self.exposure_fraction > 0, "exposure_fraction must be greater than 0"
        assert self.num_samples > 0, "num_samples must be greater than 0"


@dataclasses.dataclass
class ChromaticAberrationsConf:
    enable: bool = dataclasses.field(default_factory=bool)
    strength: tuple = (0, 0, 0)
    model: tuple = ("Radial", "Radial", "Radial")
    enable_lanczos: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        assert type(self.enable) is bool, "enable must be a boolean"
        assert type(self.strength) is tuple, "strength must be a tuple"
        assert all([type(i) is float for i in self.strength]), "strength must be a tuple of floats"
        assert len(self.strength) == 3, "strength must be a tuple of 3 floats"
        assert type(self.model) is tuple, "model must be a tuple"
        assert all([type(i) is str for i in self.model]), "model must be a tuple of strings"
        assert len(self.model) == 3, "model must be a tuple of 3 strings"
        assert type(self.enable_lanczos) is bool, "enable_lanzcos must be a boolean"

        assert all([i in ["Radial", "Barrel"] for i in self.model]), "model must be a tuple of 'Radial' or 'Barrel'"
        assert all([i <= 1 and i >= -1 for i in self.strength]), "strength must be between -1 and 1"


@dataclasses.dataclass
class RendererConf:
    samples_per_pixel_per_frame: int = dataclasses.field(default_factory=int)
    max_bounces: int = dataclasses.field(default_factory=int)
    max_specular_transmission_bounces: int = dataclasses.field(default_factory=int)
    max_volume_bounces: int = dataclasses.field(default_factory=int)
    subdiv_refinement_level: int = dataclasses.field(default_factory=int)
    renderer: str = dataclasses.field(default_factory=str)
    headless: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        assert type(self.samples_per_pixel_per_frame) is int, "samples_per_pixel_per_frame must be an integer"
        assert type(self.max_bounces) is int, "max_bounces must be an integer"
        assert (
            type(self.max_specular_transmission_bounces) is int
        ), "max_specular_transmission_bounces must be an integer"
        assert type(self.max_volume_bounces) is int, "max_volume_bounces must be an integer"
        assert type(self.subdiv_refinement_level) is int, "subdiv_refinement_level must be an integer"
        assert type(self.headless) is bool, "headless must be a boolean"
        assert type(self.renderer) is str, "renderer must be a string"

        assert self.samples_per_pixel_per_frame > 0, "samples_per_pixel_per_frame must be greater than 0"
        assert self.max_bounces > 0, "max_bounces must be greater than 0"
        assert self.max_specular_transmission_bounces > 0, "max_specular_transmission_bounces must be greater than 0"
        assert self.max_volume_bounces > 0, "max_volume_bounces must be greater than 0"
        assert self.subdiv_refinement_level >= 0, "subdiv_refinement_level must be greater or equal to 0"
        assert self.renderer in [
            "PathTracing",
            "RayTracedLighting",
        ], "renderer must be PATH, PathTracing or RayTracedLighting"
