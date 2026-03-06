__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

# Custom libs
from src.environments_wrappers.ros2.base_wrapper_ros2 import ROS_BaseManager
from src.environments.lunalab import LunalabController

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int32
from geometry_msgs.msg import Pose
import rclpy


class ROS_LunalabManager(ROS_BaseManager):
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
        self.LC = LunalabController(**environment_cfg)
        self.LC.load()
        self.trigger_reset = False

        self.create_subscription(Bool, "/OmniLRS/Projector/TurnOn", self.set_projector_on, 1)
        self.create_subscription(Float32, "/OmniLRS/Projector/Intensity", self.set_projector_intensity, 1)
        self.create_subscription(Float32, "/OmniLRS/Projector/Radius", self.set_projector_radius, 1)
        self.create_subscription(Pose, "/OmniLRS/Projector/Pose", self.set_projector_pose, 1)
        self.create_subscription(Bool, "/OmniLRS/CeilingLights/TurnOn", self.set_ceiling_on, 1)
        self.create_subscription(Float32, "/OmniLRS/CeilingLights/Intensity", self.set_ceiling_intensity, 1)
        self.create_subscription(Bool, "/OmniLRS/Curtains/Extend", self.set_curtains_mode, 1)
        self.create_subscription(Int32, "/OmniLRS/Terrain/Switch", self.switch_terrain, 1)
        self.create_subscription(Bool, "/OmniLRS/Terrain/EnableRocks", self.enable_rocks, 1)
        self.create_subscription(Int32, "/OmniLRS/Terrain/RandomizeRocks", self.randomize_rocks, 1)

    def periodic_update(self, dt: float) -> None:
        pass

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass

    def set_projector_on(self, data: Bool) -> None:
        """
        Turns the projector on or off.

        Args:
            data (Bool): True to turn the projector on, False to turn it off.
        """

        self.modifications.append([self.LC.turn_projector_on_off, {"flag": data.data}])

    def set_projector_intensity(self, data: Float32) -> None:
        """
        Sets the projector intensity.

        Args:
            data (Float32): Intensity in percentage.
        """

        default_intensity = 300000000.0
        data = default_intensity * float(data.data) / 100.0
        assert data >= 0, "The intensity must be greater than or equal to 0."
        self.modifications.append([self.LC.set_projector_intensity, {"intensity": data}])

    def set_projector_radius(self, data: Float32) -> None:
        """
        Sets the projector radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0.0"
        self.modifications.append([self.LC.set_projector_radius, {"radius": data.data}])

    def set_projector_color(self, data: ColorRGBA) -> None:
        """
        Sets the projector color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0 <= c <= 1, "The color must be between 0 and 1."
        self.modifications.append([self.LC.set_projector_color, {"color": color}])

    def set_projector_pose(self, data: Pose) -> None:
        """
        Sets the projector pose.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        position = (data.position.x, data.position.y, data.position.z)
        orientation = (data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        self.modifications.append([self.LC.set_projector_pose, {"position": position, "orientation": orientation}])

    def set_ceiling_on(self, data: Bool) -> None:
        """
        Turns the ceiling lights on or off.

        Args:
            data (Bool): True to turn the lights on, False to turn them off.
        """

        self.modifications.append([self.LC.turn_room_lights_on_off, {"flag": data.data}])

    def set_ceiling_intensity(self, data: Float32) -> None:
        """
        Sets the ceiling lights intensity.

        Args:
            data (Float32): Intensity in percentage.
        """

        assert data.data >= 0, "The intensity must be greater than or equal to 0."
        self.modifications.append([self.LC.set_room_lights_intensity, {"intensity": data.data}])

    def set_ceiling_radius(self, data: Float32) -> None:
        """
        Sets the ceiling lights radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0.0"
        self.modifications.append([self.LC.set_room_lights_radius, {"radius": data.data}])

    def set_ceiling_FOV(self, data: Float32) -> None:
        """
        Sets the ceiling lights field of view.

        Args:
            data (Float32): Field of view in degrees.
        """

        assert 0 <= data.data <= 180, "The field of view must be between 0 and 180."
        self.modifications.append([self.LC.set_room_lights_FOV, {"FOV": data.data}])

    def set_ceiling_color(self, data: ColorRGBA) -> None:
        """
        Sets the ceiling lights color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0 <= c <= 1, "The color must be between 0 and 1."
        self.modifications.append([self.LC.set_room_lights_color, {"color": color}])

    def set_curtains_mode(self, data: Bool) -> None:
        """
        Sets the curtains mode.

        Args:
            data (Bool): True to extend the curtains, False to retract them.
        """

        self.modifications.append([self.LC.curtains_extend, {"flag": data.data}])

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
