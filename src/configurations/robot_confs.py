__author__ = "Antoine Richard, Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from src.configurations.robot_camera_confs import RobotCameraConf, parse_cameras


@dataclass
class Pose:
    position: List[float] = field(default_factory=list)
    orientation: List[float] = field(default_factory=list)
    height_above_terrain_override: Optional[float] = None


@dataclass
class DriveConf:
    """
    Wheel kinematics of a skid-steered robot, used by the Yamcs drive commands (src/tmtc/drive_handler.py).

    A straight drive at v m/s turns the wheels at v / wheel_radius rad/s. A turn at w rad/s turns the left and right
    wheels at -/+ w * (track_width / 2) * skid_steer_factor / wheel_radius rad/s. A skid-steered robot turns slower than
    its wheel geometry gives, because its wheels slide sideways: skid_steer_factor is the effective track width divided
    by track_width, measured from the turn angle of a drive_turn command.
    """

    wheel_radius: float  # m
    track_width: float  # m, between the left and right wheel centers
    skid_steer_factor: float = 1.0

    def __post_init__(self):
        if self.wheel_radius <= 0 or self.track_width <= 0:
            raise ValueError(f"drive: wheel_radius and track_width must be positive, got {self}")
        if self.skid_steer_factor < 1:
            raise ValueError(f"drive: skid_steer_factor must be 1 or more, got {self.skid_steer_factor}")


@dataclass
class RobotParameters:
    robot_name: str = field(default_factory=str)
    usd_path: str = field(default_factory=str)
    pose: Pose = field(default_factory=dict)
    domain_id: int = field(default_factory=int)
    target_links: List[str] = field(default_factory=list)
    wheel_joints: Dict = field(default_factory=dict)
    base_link: str = field(default_factory=str)
    pos_relative_to_prim: str = field(default_factory=str)
    camera: List[RobotCameraConf] = field(default_factory=list)  # robot yaml list, parsed by parse_cameras
    imu_sensor_path: str = field(default_factory=str)
    drive: Optional[DriveConf] = None  # robot yaml `drive` section; only robots driven by Yamcs commands need it
    solar_panel_joint: str = field(default_factory=str)
    zenoh: Dict = field(default_factory=dict)

    def __post_init__(self):
        self.usd_path = os.path.join(os.getcwd(), self.usd_path)
        self.pose = Pose(**self.pose)
        self.camera = parse_cameras(self.camera)
        if isinstance(self.drive, dict):
            self.drive = DriveConf(**self.drive)


@dataclass
class RobotManagerConf:
    robots_root: str = "/Robots"
    parameters: RobotParameters = None
    yamcs_tmtc: Dict = field(
        default_factory=dict
    )  # TODO for v4: separate yamcs_tmtc and robot_controller into separate dataclass (and don't merge with environment configs)
    robot_controller: str = field(default_factory=str)

    def __post_init__(self):
        if self.parameters:
            self.parameters = RobotParameters(**self.parameters)
        else:
            self.parameters = None
