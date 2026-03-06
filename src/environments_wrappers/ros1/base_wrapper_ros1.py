__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple

# Custom libs
import src.environments.rendering as rndr
from src.robots.robot import RobotManager

# Loads ROS1 dependent libraries
import rospy
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from geometry_msgs.msg import Pose, PoseStamped


class ROS_BaseManager:
    """
    ROS1 node that manages the Lunalab and robots."""

    def __init__(
        self,
        environment_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the ROS1 node.

        Args:
            environment_cfg (dict): Environment configuration dictionary.
            flares_cfg (dict): Lens flares configuration dictionary.
            **kwargs: Additional keyword arguments.
        """

        self.RM = RobotManager(environment_cfg["robots_settings"])
        self.trigger_reset = False

        self.render_subs = []
        self.render_subs.append(
            rospy.Subscriber("/OmniLRS/Render/EnableRTXRealTime", Empty, self.use_RTX_real_time_render, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/OmniLRS/Render/EnableRTXInteractive", Empty, self.use_RTX_interactive_render, queue_size=1
            )
        )
        self.render_subs.append(
            rospy.Subscriber("/OmniLRS/LensFlare/EnableLensFlares", Bool, self.set_lens_flare_on, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber("/OmniLRS/LensFlare/NumBlades", Int8, self.set_lens_flare_num_blade, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber("/OmniLRS/LensFlare/Scale", Float32, self.set_lens_flare_scale, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/OmniLRS/LensFlare/ApertureRotation", Float32, self.set_lens_flare_aperture_rotation, queue_size=1
            )
        )
        self.render_subs.append(
            rospy.Subscriber("/OmniLRS/LensFlare/FocalLength", Float32, self.set_lens_flare_focal_length, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber("/OmniLRS/LensFlare/Fstop", Float32, self.set_lens_flare_fstop, queue_size=1)
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/OmniLRS/LensFlare/SensorAspectRatio", Float32, self.set_lens_flare_sensor_aspect_ratio, queue_size=1
            )
        )
        self.render_subs.append(
            rospy.Subscriber(
                "/OmniLRS/LensFlare/SensorDiagonal", Float32, self.set_lens_flare_sensor_diagonal, queue_size=1
            )
        )
        self.robot_subs = []
        self.robot_subs.append(rospy.Subscriber("/OmniLRS/Robots/Spawn", PoseStamped, self.spawn_robot, queue_size=1))
        self.robot_subs.append(
            rospy.Subscriber("/OmniLRS/Robots/Teleport", PoseStamped, self.teleport_robot, queue_size=1)
        )
        self.robot_subs.append(rospy.Subscriber("/OmniLRS/Robots/Reset", String, self.reset_robot, queue_size=1))
        self.robot_subs.append(rospy.Subscriber("/OmniLRS/Robots/ResetAll", Empty, self.reset_robots, queue_size=1))
        self.modifications: List[Tuple[callable, dict]] = []

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        raise NotImplementedError

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """

        raise NotImplementedError

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

    def use_RTX_real_time_render(self, data: Empty) -> None:
        """
        Enables or disables RTX real time rendering.

        Args:
            data (Empty): Empty message.
        """

        self.modifications.append([rndr.enable_RTX_real_time, {}])

    def use_RTX_interactive_render(self, data: Empty) -> None:
        """
        Enables or disables RTX interactive rendering.

        Args:
            data (Empty): Empty message.
        """

        self.modifications.append([rndr.enable_RTX_interactive, {}])

    def set_lens_flare_on(self, data: Bool) -> None:
        """
        Enables or disables lens flares.

        Args:
            data (Bool): True to enable lens flares, False to disable them.
        """

        self.modifications.append([rndr.enable_lens_flare, {"enable": data.data}])

    def set_lens_flare_num_blade(self, data: Int8) -> None:
        """
        Sets the number of blades of the lens flares.

        Args:
            data (Int8): Number of blades.
        """

        data = int(data.data)
        assert data > 2, "The number of blades must be greater than 2"
        self.modifications.append([rndr.set_flare_num_blades, {"value": data}])

    def set_lens_flare_scale(self, data: Float32) -> None:
        """
        Sets the scale of the lens flares.

        Args:
            data (Float32): Scale.
        """

        data = float(data.data)
        assert data >= 0, "The scale must be greater than or equal to 0"
        self.modifications.append([rndr.set_flare_scale, {"value": data}])

    def set_lens_flare_fstop(self, data: Float32) -> None:
        """
        Sets the fstop of the lens flares.

        Args:
            data (Float32): Fstop.
        """

        data = float(data.data)
        assert data > 0, "The fstop must be greater than 0"
        self.modifications.append([rndr.set_flare_fstop, {"value": data}])

    def set_lens_flare_focal_length(self, data: Float32) -> None:
        """
        Sets the focal length of the lens flares.

        Args:
            data (Float32): Focal length.
        """

        data = float(data.data)
        assert data > 0, "The focal length must be greater than 0"
        self.modifications.append([rndr.set_flare_focal_length, {"value": data}])

    def set_lens_flare_sensor_aspect_ratio(self, data: Float32) -> None:
        """
        Sets the sensor aspect ratio of the lens flares.

        Args:
            data (Float32): Sensor aspect ratio.
        """

        data = float(data.data)
        assert data > 0, "The sensor aspect ratio must be greater than 0"
        self.modifications.append([rndr.set_flare_sensor_aspect_ratio, {"value": data}])

    def set_lens_flare_sensor_diagonal(self, data: Float32) -> None:
        """
        Sets the sensor diagonal of the lens flares.

        Args:
            data (Float32): Sensor diagonal.
        """

        data = float(data.data)
        assert data > 0, "The sensor diagonal must be greater than 0"
        self.modifications.append([rndr.set_flare_sensor_diagonal, {"value": data}])

    def set_lens_flare_aperture_rotation(self, data: Float32) -> None:
        """
        Sets the aperture rotation of the lens flares.

        Args:
            data (Float32): Aperture rotation.
        """

        data = float(data.data)
        self.modifications.append([rndr.set_flare_aperture_rotation, {"value": data}])

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
        q = [
            data.pose.orientation.w,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.x,
        ]
        self.modifications.append([self.RM.add_robot, {"usd_path": usd_path, "robot_name": robot_name, "p": p, "q": q}])

    def teleport_robot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS1 Pose format.
        """

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
        self.modifications.append([self.RM.teleport_robot, {"robot_name": robot_name, "p": p, "q": q}])

    def reset_robot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset.
        """

        robot_name = data.data
        self.modifications.append([self.RM.reset_robot, {"robot_name": robot_name}])

    def reset_robots(self, data: Empty):
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def cleanScene(self):
        """
        Cleans the scene.
        """

        for sub in self.render_subs:
            sub.unregister()
        for sub in self.robot_subs:
            sub.unregister()
        rospy.signal_shutdown("Shutting down")
