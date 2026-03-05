__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

# System imports
from typing import List, Tuple

# Custom libs
import src.environments.rendering as rndr

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from rclpy.executors import SingleThreadedExecutor as Executor
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
import rclpy


class ROS_BaseManager(Node):
    """
    ROS2 node that manages the lab environment"""

    def __init__(
        self,
        environment_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab manager.

        Args:
            environment_cfg (dict): Environment configuration.
            flares_cfg (dict): Flares configuration.
            **kwargs: Additional arguments."""

        super().__init__("Lab_controller_node")
        self.trigger_reset = False

        self.create_subscription(Empty, "/OmniLRS/Render/EnableRTXRealTime", self.use_RTX_real_time_render, 1)
        self.create_subscription(Empty, "/OmniLRS/Render/EnableRTXInteractive", self.use_RTX_interactive_render, 1)
        self.create_subscription(Bool, "/OmniLRS/LensFlare/EnableLensFlares", self.set_lens_flare_on, 1)
        self.create_subscription(Int8, "/OmniLRS/LensFlare/NumBlades", self.set_lens_flare_num_blade, 1)
        self.create_subscription(Float32, "/OmniLRS/LensFlare/Scale", self.set_lens_flare_scale, 1)
        self.create_subscription(
            Float32, "/OmniLRS/LensFlare/ApertureRotation", self.set_lens_flare_aperture_rotation, 1
        )
        self.create_subscription(Float32, "/OmniLRS/LensFlare/FocalLength", self.set_lens_flare_focal_length, 1)
        self.create_subscription(Float32, "/OmniLRS/LensFlare/Fstop", self.set_lens_flare_fstop, 1)
        self.create_subscription(
            Float32, "/OmniLRS/LensFlare/SensorAspectRatio", self.set_lens_flare_sensor_aspect_ratio, 1
        )
        self.create_subscription(Float32, "/OmniLRS/LensFlare/SensorDiagonal", self.set_lens_flare_sensor_diagonal, 1)

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
        assert data > 2, "The number of blades must be greater than 2."
        self.modifications.append([rndr.set_flare_num_blades, {"value": data}])

    def set_lens_flare_scale(self, data: Float32) -> None:
        """
        Sets the scale of the lens flares.

        Args:
            data (Float32): Scale.
        """

        data = float(data.data)
        assert data > 0, "The scale must be greater than 0."
        self.modifications.append([rndr.set_flare_scale, {"value": data}])

    def set_lens_flare_fstop(self, data: Float32) -> None:
        """
        Sets the fstop of the lens flares.

        Args:
            data (Float32): Fstop.
        """

        data = float(data.data)
        assert data > 0, "The fstop must be greater than 0."
        self.modifications.append([rndr.set_flare_fstop, {"value": data}])

    def set_lens_flare_focal_length(self, data: Float32) -> None:
        """
        Sets the focal length of the lens flares.

        Args:
            data (Float32): Focal length.
        """

        data = float(data.data)
        assert data > 0, "The focal length must be greater than 0."
        self.modifications.append([rndr.set_flare_focal_length, {"value": data}])

    def set_lens_flare_sensor_aspect_ratio(self, data: Float32) -> None:
        """
        Sets the sensor aspect ratio of the lens flares.

        Args:
            data (Float32): Sensor aspect ratio.
        """

        data = float(data.data)
        assert data > 0, "The sensor aspect ratio must be greater than 0."
        self.modifications.append([rndr.set_flare_sensor_aspect_ratio, {"value": data}])

    def set_lens_flare_sensor_diagonal(self, data: Float32) -> None:
        """
        Sets the sensor diagonal of the lens flares.

        Args:
            data (Float32): Sensor diagonal.
        """

        data = float(data.data)
        assert data > 0, "The sensor diagonal must be greater than 0."
        self.modifications.append([rndr.set_flare_sensor_diagonal, {"value": data}])

    def set_lens_flare_aperture_rotation(self, data: Float32) -> None:
        """
        Sets the aperture rotation of the lens flares.

        Args:
            data (Float32): Aperture rotation.
        """

        data = float(data.data)
        self.modifications.append([rndr.set_flare_aperture_rotation, {"value": data}])

    def clean_scene(self):
        """
        Cleans the scene.
        """

        self.destroy_node()

    def monitor_thread_is_alive(self):
        """
        Checks if the monitor thread is alive.
        """

        return True

    def get_wait_for_threads(self):
        """
        Returns the list of waiting threads.
        """

        return []
