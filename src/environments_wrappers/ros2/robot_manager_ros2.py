__author__ = "Antoine Richard, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
from pxr import Gf
from rclpy.node import Node
from std_msgs.msg import Empty, String

from src.configurations.simulator_mode_enum import SimulatorMode
from src.robots.robot import RobotManager


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf, mode=SimulatorMode.ROS2)
        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Spawn", self.spawn_robot, 1)
        self.create_subscription(PoseStamped, "/OmniLRS/Robots/Teleport", self.teleport_robot, 1)
        self.create_subscription(String, "/OmniLRS/Robots/Reset", self.reset_robot, 1)
        self.create_subscription(Empty, "/OmniLRS/Robots/ResetAll", self.reset_robots, 1)

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []

    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """

        self.clear_modifications()
        self.reset_robots(Empty())

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """

        self.modifications: List[Tuple[callable, dict]] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def spawn_robot(self, data: PoseStamped) -> None:
        """
        Spawns a robot.

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path
        """

        assert len(data.header.frame_id.split(":")) == 2, "The data should be in the format: robot_name:usd_path"
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        self.modifications.append(
            [
                self.RM.add_robot,
                {"usd_path": usd_path, "robot_name": robot_name, "p": p, "q": q, "domain_id": self.domain_id},
            ]
        )

    def teleport_robot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.modifications.append([self.RM.teleport_robot, {"robot_name": robot_name, "position": p, "orientation": q}])

    def reset_robot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset.
        """

        robot_name = data.data
        self.modifications.append([self.RM.reset_robot, {"robot_name": robot_name}])

    def reset_robots(self, data: Empty) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def cleanRobots(self) -> None:
        """
        Cleans the robots."""

        self.destroy_node()
