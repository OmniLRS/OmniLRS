__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

# Custom libs
from src.environments_wrappers.ros2.base_wrapper_ros2 import ROS_BaseManager
from src.environments.lunaryard import LunaryardController

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int32
from geometry_msgs.msg import Pose
import rclpy


class ROS_LunaryardManager(ROS_BaseManager):
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
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        self.LC = LunaryardController(**environment_cfg)
        self.LC.load()

        self.create_subscription(Float32, "/OmniLRS/Sun/Intensity", self.set_sun_intensity, 1)
        self.create_subscription(Pose, "/OmniLRS/Sun/Pose", self.set_sun_pose, 1)
        self.create_subscription(ColorRGBA, "/OmniLRS/Sun/Color", self.set_sun_color, 1)
        self.create_subscription(Float32, "/OmniLRS/Sun/ColorTemperature", self.set_sun_color_temperature, 1)
        self.create_subscription(Float32, "/OmniLRS/Sun/AngularSize", self.set_sun_angle, 1)
        self.create_subscription(Int32, "/OmniLRS/Terrain/Switch", self.switch_terrain, 1)
        self.create_subscription(Bool, "/OmniLRS/Terrain/EnableRocks", self.enable_rocks, 1)
        self.create_subscription(Int32, "/OmniLRS/Terrain/RandomizeRocks", self.randomize_rocks, 1)

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        self.modifications.append([self.LC.update_stellar_engine, {"dt": dt}])

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass

    def set_sun_intensity(self, data: Float32) -> None:
        """
        Sets the projector intensity.

        Args:
            data (Float32): Intensity in percentage."""

        assert data.data >= 0, "The intensity must be greater than or equal to 0."
        self.modifications.append([self.LC.set_sun_intensity, {"intensity": data.data}])

    def set_sun_color(self, data: ColorRGBA) -> None:
        """
        Sets the projector color.

        Args:
            data (ColorRGBA): Color in RGBA format."""

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0 <= c <= 1, "The color must be between 0 and 1."
        self.modifications.append([self.LC.set_sun_color, {"color": color}])

    def set_sun_color_temperature(self, data: Float32) -> None:
        """
        Sets the projector color temperature.

        Args:
            data (Float32): Color temperature in Kelvin.
        """

        assert data.data >= 0, "The color temperature must be greater than or equal to 0"
        self.modifications.append([self.LC.set_sun_color_temperature, {"temperature": data.data}])

    def set_sun_angle(self, data: Float32) -> None:
        """
        Sets the projector angle.

        Args:
            data (Float32): Angle in degrees.
        """

        assert data.data >= 0, "The angle must be greater than or equal to 0"
        self.modifications.append([self.LC.set_sun_angle, {"angle": data.data}])

    def set_sun_pose(self, data: Pose) -> None:
        """
        Sets the projector pose.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        position = [data.position.x, data.position.y, data.position.z]
        orientation = [data.orientation.w, data.orientation.y, data.orientation.z, data.orientation.x]
        self.modifications.append([self.LC.set_sun_pose, {"position": position, "orientation": orientation}])

    def switch_terrain(self, data: Int32) -> None:
        """
        Switches the terrain.

        Args:
            data (Int32): 0 for the first terrain, 1 for the second terrain.
        """

        self.modifications.append([self.LC.switch_terrain, {"flag": data.data}])
        self.trigger_reset = True

    def enable_rocks(self, data: Bool) -> None:
        """
        Enables or disables the rocks.

        Args:
            data (Bool): True to enable the rocks, False to disable them.
        """

        self.modifications.append([self.LC.enable_rocks, {"flag": data.data}])
        self.trigger_reset = True

    def randomize_rocks(self, data: Int32) -> None:
        """
        Randomizes the rocks.

        Args:
            data (Int32): Number of rocks to randomize.
        """

        data = int(data.data)
        assert data > 0, "The number of rocks must be greater than 0."
        self.modifications.append([self.LC.randomize_rocks, {"num": data}])
        self.trigger_reset = True
