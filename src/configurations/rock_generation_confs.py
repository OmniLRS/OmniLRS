__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import dataclasses


@dataclasses.dataclass
class RockGenerationConf:
    instancers_path: str = dataclasses.field(default_factory=str)
    rocks_settings: dict = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class RequestGroupConf:
    seed: int = dataclasses.field(default_factory=int)
    collections: list = dataclasses.field(default_factory=list)
    use_point_instancer: bool = dataclasses.field(default_factory=bool)
    requests: list = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class RequestConf:
    attribute: str = dataclasses.field(default_factory=str)
    num: int = dataclasses.field(default_factory=int)
    parent: str = dataclasses.field(default_factory=str)
    axes: list = dataclasses.field(default_factory=list)
    layer: dict = dataclasses.field(default_factory=dict)
    sampler: dict = dataclasses.field(default_factory=dict)
