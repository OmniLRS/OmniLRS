__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import os
from dataclasses import dataclass, field
from typing import List, Dict


@dataclass
class Pose:
    position: List[float] = field(default_factory=list)
    orientation: List[float] = field(default_factory=list)


@dataclass
class RobotParameters:
    robot_name: str = field(default_factory=str)
    usd_path: str = field(default_factory=str)
    pose: Pose = field(default_factory=dict)
    domain_id: int = field(default_factory=int)
    target_links: List[str] = field(default_factory=list)
    wheel_joints: Dict = field(default_factory=dict)
    pose_base_link: str = field(default_factory=str)

    def __post_init__(self):
        self.usd_path = os.path.join(os.getcwd(), self.usd_path)
        self.pose = Pose(**self.pose)


@dataclass
class RobotManagerConf:
    uses_nucleus: str = False
    is_ROS2: str = False
    max_robots: int = 5
    robots_root: str = "/Robots"
    parameters: List[RobotParameters] = field(default_factory=list)
    yamcs_tmtc: Dict = field(default_factory=dict)

    def __post_init__(self):
        if self.parameters:
            assert len(self.parameters) <= self.max_robots, "number of robots to register should not exeed max_robots"
            self.parameters = [RobotParameters(**param) for param in self.parameters]
        else:
            self.parameters = []
