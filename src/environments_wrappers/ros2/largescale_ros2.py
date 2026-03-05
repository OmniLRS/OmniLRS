__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

# Custom libs
from src.environments_wrappers.ros2.base_wrapper_ros2 import ROS_BaseManager
from src.environments.large_scale_lunar import LargeScaleController

# Loads ROS2 dependent libraries
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Pose
import rclpy


class ROS_LargeScaleManager(ROS_BaseManager):
    """
    ROS2 node that manages the environment
    """

    def __init__(
        self,
        environment_cfg: dict = None,
        is_simulation_alive: callable = lambda: True,
        close_simulation: callable = lambda: None,
        **kwargs,
    ) -> None:
        """
        Initializes the environment manager.

        Args:
            environment_cfg (dict): Environment configuration.
            is_simulation_alive (callable): function to check if the simulation is alive.
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        self.LC = LargeScaleController(
            **environment_cfg, is_simulation_alive=is_simulation_alive, close_simulation=close_simulation
        )
        self.LC.load()

        self.create_subscription(Float32, "/OmniLRS/Sun/Intensity", self.set_sun_intensity, 1)
        self.create_subscription(Pose, "/OmniLRS/Sun/Pose", self.set_sun_pose, 1)
        self.create_subscription(ColorRGBA, "/OmniLRS/Sun/Color", self.set_sun_color, 1)
        self.create_subscription(Float32, "/OmniLRS/Sun/ColorTemperature", self.set_sun_color_temperature, 1)
        self.create_subscription(Float32, "/OmniLRS/Sun/AngularSize", self.set_sun_angle, 1)

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        self.modifications.append([self.LC.update_stellar_engine, {"dt": dt}])
        self.LC.update()

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

    def monitor_thread_is_alive(self):
        return self.LC.monitor_thread_is_alive()

    def get_wait_for_threads(self):
        return self.LC.get_wait_for_threads()
