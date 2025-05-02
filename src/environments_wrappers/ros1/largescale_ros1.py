__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# Custom libs
from src.environments_wrappers.ros1.base_wrapper_ros1 import ROS_BaseManager
from src.environments.large_scale_lunar import LargeScaleController

# Loads ROS1 dependent libraries
import rospy
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Pose


class ROS_LargeScaleManager(ROS_BaseManager):
    """
    ROS1 node that manages the Lunalab and robots."""

    def __init__(
        self,
        environment_cfg: dict = None,
        is_simulation_alive: callable = lambda: True,
        **kwargs,
    ) -> None:
        """
        Initializes the ROS1 node.

        Args:
            environment_cfg (dict): Environment configuration dictionary.
            flares_cfg (dict): Lens flares configuration dictionary.
            is_simulation_alive (callable): function to check if the simulation is alive.
            **kwargs: Additional keyword arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        self.LC = LargeScaleController(**environment_cfg, is_simulation_alive=is_simulation_alive)
        self.LC.load()

        self.projector_subs = []
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Sun/Intensity", Float32, self.set_sun_intensity, queue_size=1)
        )
        self.projector_subs.append(rospy.Subscriber("/OmniLRS/Sun/Pose", Pose, self.set_sun_pose, queue_size=1))
        self.projector_subs.append(rospy.Subscriber("/OmniLRS/Sun/Color", ColorRGBA, self.set_sun_color, queue_size=1))
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Sun/ColorTemperature", Float32, self.set_sun_color_temperature, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Sun/AngularSize", Float32, self.set_sun_angle, queue_size=1)
        )

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        self.LC.update_stellar_engine(dt=dt)

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """
        pass

    def set_sun_intensity(self, data: Float32) -> None:
        """
        Sets the projector intensity.

        Args:
            data (Float32): Intensity in percentage.
        """

        assert data.data >= 0, "The intensity must be greater than or equal to 0"
        self.modifications.append([self.LC.set_sun_intensity, {"intensity", data.data}])

    def set_sun_color(self, data: ColorRGBA) -> None:
        """
        Sets the projector color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert c >= 0 and c <= 1, "The color must be between 0 and 1"
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
        orientation = [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
        self.modifications.append([self.LC.set_sun_pose, {"position": position, "orientation": orientation}])

    def cleanScene(self):
        """
        Cleans the scene.
        """

        super().cleanScene()

        for sub in self.projector_subs:
            sub.unregister()
        rospy.signal_shutdown("Shutting down")
