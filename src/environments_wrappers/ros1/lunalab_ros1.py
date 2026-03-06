__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

# Custom libs
from src.environments_wrappers.ros1.base_wrapper_ros1 import ROS_BaseManager
from src.environments.lunalab import LunalabController
import src.environments.rendering as rndr

# Loads ROS1 dependent libraries
import rospy
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from geometry_msgs.msg import Pose, PoseStamped


class ROS_LunalabManager(ROS_BaseManager):
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

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        self.LC = LunalabController(**environment_cfg)
        self.LC.load()

        self.projector_subs = []
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Projector/TurnOn", Bool, self.set_projector_on, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Projector/Intensity", Float32, self.set_projector_intensity, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Projector/Radius", Float32, self.set_projector_radius, queue_size=1)
        )
        self.projector_subs.append(
            rospy.Subscriber("/OmniLRS/Projector/Pose", Pose, self.set_projector_pose, queue_size=1)
        )
        self.ceiling_subs = []
        self.ceiling_subs.append(
            rospy.Subscriber("/OmniLRS/CeilingLights/TurnOn", Bool, self.set_ceiling_on, queue_size=1)
        )
        self.ceiling_subs.append(
            rospy.Subscriber("/OmniLRS/CeilingLights/Intensity", Float32, self.set_ceiling_intensity, queue_size=1)
        )
        self.terrains_subs = []
        self.terrains_subs.append(rospy.Subscriber("/OmniLRS/Terrain/Switch", Int32, self.switch_terrain, queue_size=1))
        self.terrains_subs.append(
            rospy.Subscriber("/OmniLRS/Terrain/EnableRocks", Bool, self.enable_rocks, queue_size=1)
        )
        self.terrains_subs.append(
            rospy.Subscriber("/OmniLRS/Terrain/RandomizeRocks", Int32, self.randomize_rocks, queue_size=1)
        )

    def periodic_update(self, dt: float) -> None:
        pass

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """
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

        assert 0.0 <= data.data <= 100.0, "Intensity must be between 0 and 100."
        default_intensity = 120000000.0
        data = default_intensity * float(data.data) / 100.0
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
            assert 0.0 <= c <= 1.0, "Color values must be between 0 and 1."
        self.modifications.append([self.LC.set_projector_color, {"color": color}])

    def set_projector_pose(self, data: Pose) -> None:
        """
        Sets the projector pose.

        Args:
            data (Pose): Pose in ROS Pose format.
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
            data (Float32): Intensity in arbitrary units.
        """

        assert 0.0 <= data.data, "Intensity must be greater than 0."

        self.modifications.append([self.LC.set_room_lights_intensity, {"intensity": data.data}])

    def set_ceiling_radius(self, data: Float32) -> None:
        """
        Sets the ceiling lights radius.

        Args:
            data (Float32): Radius in meters.
        """

        assert data.data > 0.0, "Radius must be greater than 0."
        self.modifications.append([self.LC.set_room_lights_radius, {"radius": data.data}])

    def set_ceiling_FOV(self, data: Float32) -> None:
        """
        Sets the ceiling lights field of view.

        Args:
            data (Float32): Field of view in degrees.
        """

        assert 0.0 <= data.data <= 180.0, "Field of view must be between 0 and 180."
        self.modifications.append([self.LC.set_room_lights_FOV, {"FOV": data.data}])

    def set_ceiling_color(self, data: ColorRGBA) -> None:
        """
        Sets the ceiling lights color.

        Args:
            data (ColorRGBA): Color in RGBA format.
        """

        color = [data.r, data.g, data.b]
        for c in color:
            assert 0.0 <= c <= 1.0, "Color values must be between 0 and 1."
        self.modifications.append([self.LC.set_room_lights_color, {"color": color}])

    def set_curtains_mode(self, data: Bool) -> None:
        """
        Sets the curtains mode.

        Args:
            data (Bool): True to extend the curtains, False to retract them.
        """

        self.modifications.append([self.LC.curtains_extend, {"flag": data.data}])

    def switch_terrain(self, data: Bool) -> None:
        """
        Switches the terrain.

        Args:
            data (Int32): 0 for the first terrain, 1 for the second terrain.
        """

        self.modifications.append([self.LC.switch_terrain, {"flag": data.data}])

    def enable_rocks(self, data: Bool) -> None:
        """
        Enables or disables the rocks.

        Args:
            data (Bool): True to enable the rocks, False to disable them.
        """

        self.modifications.append([self.LC.enable_rocks, {"flag": data.data}])

    def randomize_rocks(self, data: Int32) -> None:
        """
        Randomizes the rocks.

        Args:
            data (Int32): Number of rocks to randomize.
        """

        data = int(data.data)
        assert data > 0, "Number of rocks must be greater than 0."
        self.modifications.append([self.LC.randomize_rocks, {"num": data}])

    def clean_scene(self):
        """
        Cleans the scene.
        """

        super().clean_scene()

        for sub in self.projector_subs:
            sub.unregister()
        for sub in self.ceiling_subs:
            sub.unregister()
        for sub in self.terrains_subs:
            sub.unregister()
        rospy.signal_shutdown("Shutting down")
