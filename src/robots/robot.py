__author__ = "Antoine Richard, Junnosuke Kamohara, Aleksa Stanivuk, Shamistan Karimov"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import math
import os
from typing import Dict, List, Optional, Tuple

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.xform as xform_utils
import numpy as np
import omni
from isaacsim.core.experimental.prims import RigidPrim

# from src.robots.subsystems_manager import RobotSubsystemsManager
from isaacsim.sensors.experimental.physics import IMUSensor
from pxr import Gf, Usd
from scipy.spatial.transform import Rotation as R
from WorldBuilders.pxr_utils import createObject, createXform

from src.configurations.robot_confs import RobotManagerConf
from src.configurations.simulator_mode_enum import SimulatorMode
from src.environments.utils import resolve_position, transform_orientation_from_xyzw_into_xyz
from src.physics.simulation import Simulation
from src.robots.articulation_control import ArticulationControl
from src.robots.articulation_telemetry import ArticulationTelemetry
from src.sensors.camera import RtxCamera
from src.subsystems.robot_subsystems_handler import RobotSubsystemsHandler


# TODO for v4: rethink which methods should be in Manager, RRG, what should be in Robot
# TODO for v4: separate into a different file (very complex and lengthy classes)
class RobotManager:
    """
    RobotManager class.
    It allows to spawn, reset, teleport robots. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""

    def __init__(
        self,
        RM_conf: RobotManagerConf,
        mode: SimulatorMode = SimulatorMode.ROS2,
    ) -> None:
        """
        Args:
            RM_conf (RobotManagerConf): The configuration of the robot manager.
        """

        self.stage = (
            omni.usd.get_context().get_stage()
        )  # TODO for v4: logcally an instance of a robot, should not have access to the instance of stage... change?
        self.RM_conf = RobotManagerConf(**RM_conf)
        self.is_ROS2 = mode == SimulatorMode.ROS2
        self.robot_parameters = self.RM_conf.parameters
        self.robots_root = self.RM_conf.robots_root
        createXform(self.stage, self.robots_root)
        self.robot: Robot = None
        self.robot_RG: RobotRigidGroup = None

    def preload_robot(
        self,
        sim: Simulation,
        get_height_func: callable = None,
    ) -> None:
        """
        Preload the robot in the scene.
        Args:
            sim (Simulation): The simulation, used to (re)initialize physics.
            get_height_func (callable): terrain height query, used when pose.height_above_terrain_override is set.
        """
        pose = self.robot_parameters.pose
        self.add_robot(
            self.robot_parameters.usd_path,
            self.robot_parameters.robot_name,
            resolve_position(pose.position, pose.height_above_terrain_override, get_height_func),
            pose.orientation,
            self.robot_parameters.domain_id,
            self.robot_parameters.wheel_joints,
            self.robot_parameters.camera,
            self.robot_parameters.imu_sensor_path,
            self.robot_parameters.dimensions,
            self.robot_parameters.turn_speed_coef,
            self.robot_parameters.pos_relative_to_prim,
            self.robot_parameters.solar_panel_joint,
        )
        self.add_RRG(
            self.robot_parameters.robot_name,
            self.robot_parameters.target_links,
            self.robot_parameters.base_link,
            sim,
        )

    def preload_robot_at_pose(
        self,
        sim: Simulation,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> None:
        """
        Preload the robot in the scene.
        Args:
            sim (Simulation): The simulation, used to (re)initialize physics.
            position (Tuple[float, float, float]): The position of the robot. (x, y, z)
            orientation (Tuple[float, float, float, float]): The orientation of the robot. (w, x, y, z)
        """
        self.add_robot(
            self.robot_parameters.usd_path,
            self.robot_parameters.robot_name,
            position,
            orientation,
            self.robot_parameters.domain_id,
            self.robot_parameters.wheel_joints,
            self.robot_parameters.camera,
            self.robot_parameters.imu_sensor_path,
            self.robot_parameters.dimensions,
            self.robot_parameters.turn_speed_coef,
            self.robot_parameters.pos_relative_to_prim,
            self.robot_parameters.solar_panel_joint,
        )
        self.add_RRG(
            self.robot_parameters.robot_name,
            self.robot_parameters.target_links,
            self.robot_parameters.base_link,
            sim,
        )

    def add_robot(
        self,
        usd_path: str = None,
        robot_name: str = None,
        p: Tuple[float, float, float] = [0, 0, 0],
        q: Tuple[float, float, float, float] = [0, 0, 0, 1],
        domain_id: int = None,
        wheel_joints: dict = {},
        camera_conf: dict = {},
        imu_sensor_path: str = "",
        dimensions: dict = {},
        turn_speed_coef: float = 1,
        pos_relative_to_prim: str = "",
        solar_panel_joint: str = None,
    ) -> None:
        """
        Add a robot to the scene.

        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            p (Tuple[float, float, float]): The position of the robot. (x, y, z)
            q (Tuple[float, float, float, float]): The orientation of the robot. (w, x, y, z)
            domain_id (int): The domain id of the robot. Not required if the robot is not ROS2 enabled.
        """

        if robot_name[0] != "/":
            print(robot_name)
            robot_name = "/" + robot_name

        self.robot = Robot(
            usd_path,
            robot_name,
            is_ROS2=self.is_ROS2,
            domain_id=domain_id,
            robots_root=self.robots_root,
            wheel_joints=wheel_joints,
            camera_conf=camera_conf,
            imu_sensor_path=imu_sensor_path,
            dimensions=dimensions,
            turn_speed_coef=turn_speed_coef,
            pos_relative_to_prim=pos_relative_to_prim,
            solar_panel_joint=solar_panel_joint,
        )
        self.robot.load(p, q)

    def add_RRG(
        self,
        robot_name: str = None,
        target_links: List[str] = None,
        pose_base_link: str = None,
        sim: Simulation = None,
    ) -> None:
        """
        Add a robot rigid group to the scene.

        Args:
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names.
            sim (Simulation): The simulation, used to (re)initialize physics.
        """
        rrg = RobotRigidGroup(
            self.robots_root,
            robot_name,
            target_links,
            pose_base_link,
        )
        rrg.initialize(sim)
        self.robot_RG = rrg

    def reset_robot(self) -> None:
        """
        Reset the robot to its original position.
        """
        self.robot.reset()

    def teleport_robot(self, position: np.ndarray = None, orientation: np.ndarray = None) -> None:
        """
        Teleport the robot to a specific position and orientation.
        """
        self.robot.teleport(position, orientation)


class Robot:
    """
    Robot class.
    It allows to spawn, reset, teleport a robot. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation.
    """

    # TODO for v4: simplify and refactor the initialization, put some inits into a separate init methods,
    # TODO for v4: lower the number of arguments, simplify
    def __init__(
        self,
        usd_path: str,
        robot_name: str,
        robots_root: str = "/Robots",
        is_ROS2: bool = False,
        domain_id: int = 0,
        wheel_joints: Dict = {},
        camera_conf: Dict = {},
        imu_sensor_path: str = "",
        dimensions: dict = {},
        turn_speed_coef: float = 1,
        pos_relative_to_prim: str = "",
        solar_panel_joint: str = None,
    ) -> None:
        """
        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            robots_root (str, optional): The root path of the robots. Defaults to "/Robots".
            is_ROS2 (bool, optional): Whether the robots are ROS2 enabled or not. Defaults to False.
            domain_id (int, optional): The domain id of the robot. Defaults to 0."""

        self.stage: Usd.Stage = omni.usd.get_context().get_stage()
        self.usd_path = str(usd_path)
        self.robots_root = robots_root
        self.robot_name = robot_name
        self.robot_path = os.path.join(self.robots_root, self.robot_name.strip("/"))
        self.is_ROS2 = is_ROS2
        self.domain_id = int(domain_id)
        self._wheel_joint_names = wheel_joints or {}
        self.telemetry = ArticulationTelemetry(
            prim_path=self.robot_path,
            name=f"{self.robot_name.strip('/')}_telemetry",
            sample_period_s=0.0,
        )
        self.control = ArticulationControl(
            prim_path=self.robot_path,
            name=f"{self.robot_name.strip('/')}_control",
            apply_period_s=0.0,
        )
        self._articulation_api_enabled = False
        self._last_joint_positions: dict[str, float] = {}
        self._last_joint_velocities: dict[str, float] = {}
        self._last_joint_efforts: dict[str, float] = {}
        self._last_wheel_joint_angles: list[float] = []
        self._camera_conf = camera_conf
        self._cameras = {}
        self.dimensions = dimensions
        self.turn_speed_coef = turn_speed_coef
        self._imu_sensor: Optional[IMUSensor] = None
        self._imu_sensor_path: str = imu_sensor_path
        self._solar_panel_joint = solar_panel_joint
        self._setup_subsystems_handler(pos_relative_to_prim)

    def _setup_subsystems_handler(self, pos_relative_to_prim):
        self.subsystems: RobotSubsystemsHandler = None
        robot_name = self.robot_name.strip("/")

        if robot_name == "pragyaan":
            from src.mission_specific.pragyaan.subsystems.pragyaan_subsystems_handler import PragyaanSubsystemsHandler

            self.subsystems = PragyaanSubsystemsHandler(pos_relative_to_prim)
        elif robot_name == "husky":
            from src.mission_specific.husky.subsystems.husky_subsystems_handler import HuskySubsystemsHandler

            self.subsystems = HuskySubsystemsHandler(pos_relative_to_prim)

    def edit_graphs(self) -> None:
        """
        Edit the graphs of the robot to add namespaces to topics and tfs.
        """

        selected_paths = []
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.robot_path)):
            graph_attrs = [attr for attr in prim.GetAttributes() if attr.GetName().split(":")[0] == "graph"]
            if graph_attrs:
                selected_paths.append(prim.GetPath())

        for path in selected_paths:
            prim = self.stage.GetPrimAtPath(path)
            prim.GetAttribute("graph:variable:Namespace").Set(self.robot_name)
            if self.is_ROS2:
                prim.GetAttribute("graph:variable:Context").Set(self.domain_id)

    def load(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Load the robot in the scene, and automatically edit its graphs.

        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.set_reset_pose(position, orientation)
        createObject(
            self.robot_path,
            self.stage,
            self.usd_path,
            is_instance=False,
            position=Gf.Vec3d(*np.asarray(position, dtype=np.float64)),
            rotation=Gf.Quatd(*np.asarray(orientation, dtype=np.float64)),
        )
        self.edit_graphs()
        self._initialize_cameras()

    def is_articulation_api_ready(self) -> bool:
        return self._articulation_api_enabled

    def update_articulation_api(self) -> None:
        """
        Call this from the simulation update loop after timeline.play().
        This is the only place where Robot should actively update articulation-backed state.
        """
        control_ready = self.control.maybe_initialize()
        telemetry_ready = self.telemetry.maybe_initialize()

        if not control_ready or not telemetry_ready:
            self._articulation_api_enabled = False
            return

        frame = self.telemetry.sample()
        if frame is None:
            self._articulation_api_enabled = False
            return

        self._articulation_api_enabled = True

        self._last_joint_positions = {name: sample.position for name, sample in frame.joints.items()}
        self._last_joint_velocities = {name: sample.velocity for name, sample in frame.joints.items()}
        self._last_joint_efforts = {name: sample.effort for name, sample in frame.joints.items()}

        self.update_wheel_joint_angles()

    def update_wheel_joint_angles(self) -> None:
        """
        Update cached wheel joint angles from the latest articulation telemetry.

        This should not call Isaac directly.
        It only uses self._last_joint_positions, which is updated by update_articulation_api().
        """
        if not self._wheel_joint_names:
            self._last_wheel_joint_angles = []
            return

        joint_angles = []

        for side in ["left", "right"]:
            for joint_name in self._wheel_joint_names.get(side, []):
                joint_angles.append(float(self._last_joint_positions.get(joint_name, 0.0)))

        self._last_wheel_joint_angles = joint_angles

    def invalidate_articulation_api(self) -> None:
        """
        Call after a simulation reset or when physics views may have been invalidated.
        """
        self._articulation_api_enabled = False

        self._last_joint_positions = {}
        self._last_joint_velocities = {}
        self._last_joint_efforts = {}
        self._last_wheel_joint_angles = []
        self._imu_sensor = None

        try:
            self.control.close()
        except Exception:
            pass

        try:
            self.telemetry.close()
        except Exception:
            pass

    def get_streaming_cam_resolution(self):
        return (self._camera_conf["resolutions"]["low"][0], self._camera_conf["resolutions"]["low"][1])

    def get_high_cam_resolution(self):
        return (self._camera_conf["resolutions"]["high"][0], self._camera_conf["resolutions"]["high"][1])

    def _initialize_cameras(self) -> None:
        # One RTX render product per resolution on the camera prim (created if missing), with RGBA and depth.
        if "resolutions" not in self._camera_conf:
            return

        resolutions = list(self._camera_conf.get("resolutions").keys())

        for res in resolutions:
            self._cameras[res] = RtxCamera(
                self._camera_conf["prim_path"],
                resolution=self._camera_conf["resolutions"][res],
                depth=True,
            )

    def get_rgba_camera_view(self, resolution) -> np.ndarray:
        return self._cameras[resolution].get_rgba()

    def get_depth_camera_view(self, resolution) -> np.ndarray:
        """Returns depth image in meters as (H, W) float32 array."""
        depth = self._cameras[resolution].get_depth()
        print("depth")
        print(depth)
        return depth

    def get_imu_readings(self):
        if self._imu_sensor_path == "":
            raise Exception(
                "Path to imu sensor is not defined. Please check your .yaml configuration file. 'imu_sensor_path' should be defined on the same level as 'robot_name'."
            )

        # https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_physics_imu.html
        # The reading is a snapshot of the latest physics step and includes gravity (specific force).
        if self._imu_sensor is None:
            self._imu_sensor = IMUSensor(self._imu_sensor_path)
        sensor_reading = self._imu_sensor.get_sensor_reading()
        linear_acceleration = {
            "ax": sensor_reading.linear_acceleration_x,
            "ay": sensor_reading.linear_acceleration_y,
            "az": sensor_reading.linear_acceleration_z,
        }
        angular_velocity = {
            "gx": sensor_reading.angular_velocity_x,
            "gy": sensor_reading.angular_velocity_y,
            "gz": sensor_reading.angular_velocity_z,
        }

        raw_orientation = np.array(
            [
                sensor_reading.orientation_x,
                sensor_reading.orientation_y,
                sensor_reading.orientation_z,
                sensor_reading.orientation_w,
            ],
            dtype=float,
        )

        if raw_orientation.shape != (4,):
            print(f"[WARN] Invalid IMU orientation shape: {raw_orientation}")
            xyz_orientation = np.zeros(3)

        elif np.linalg.norm(raw_orientation) < 1e-8:
            print(
                f"[WARN] IMU returned zero quaternion at path "
                f"{self._imu_sensor_path}: {raw_orientation}. "
                "Sensor is probably not initialized yet."
            )
            xyz_orientation = np.zeros(3)

        else:
            raw_orientation = raw_orientation / np.linalg.norm(raw_orientation)

            xyz_orientation = transform_orientation_from_xyzw_into_xyz(raw_orientation)

        orientation = {
            "roll": -float(xyz_orientation[0]),
            "pitch": -float(xyz_orientation[1]),
            "yaw": float(xyz_orientation[2]),
        }

        return linear_acceleration, angular_velocity, orientation

    def get_pose(self) -> List[float]:
        """
        Get the pose of the robot root.

        Returns:
            position, orientation_xyzw
        """
        if not self._articulation_api_enabled or self.telemetry.articulation is None:
            raise RuntimeError(
                f"Articulation API is not ready: {self.robot_path}. "
                "Call update_articulation_api() from the simulation loop first."
            )

        positions, orientations = self.telemetry.articulation.get_world_poses()
        position, quat_wxyz = positions.numpy()[0], orientations.numpy()[0]

        quat_xyzw = [
            float(quat_wxyz[1]),
            float(quat_wxyz[2]),
            float(quat_wxyz[3]),
            float(quat_wxyz[0]),
        ]

        return position, quat_xyzw

    def set_reset_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Set the reset pose of the robot.

        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot.
        """

        self.reset_position = position
        self.reset_orientation = orientation

    def teleport(self, p: List[float], q: List[float]) -> None:
        """
        Teleport the robot to a specific position and orientation.

        Args:
            p: position [x, y, z]
            q: orientation [x, y, z, w]
        """
        if not self._articulation_api_enabled or self.control.articulation is None:
            raise RuntimeError(
                f"Articulation API is not ready: {self.robot_path}. "
                "Call update_articulation_api() from the simulation loop first."
            )

        quat_wxyz = np.asarray([q[3], q[0], q[1], q[2]], dtype=np.float32)
        position = np.asarray(p, dtype=np.float32)

        self.control.articulation.set_world_poses(positions=position[np.newaxis], orientations=quat_wxyz[np.newaxis])
        self.control.articulation.set_velocities(
            linear_velocities=np.zeros((1, 3), dtype=np.float32),
            angular_velocities=np.zeros((1, 3), dtype=np.float32),
        )

        self._last_joint_positions = {}
        self._last_joint_velocities = {}
        self._last_joint_efforts = {}
        self._last_wheel_joint_angles = []

    def reset(self) -> None:
        """
        Reset the robot to its original position and orientation.

        Note:
            This requires articulation API to be ready. If called during a simulation reset,
            queue this from the simulation manager after update_articulation_api() succeeds.
        """
        if not self._articulation_api_enabled:
            print(f"Cannot reset robot yet. Articulation API is not ready: {self.robot_path}")
            return

        self.teleport(
            [
                self.reset_position[0],
                self.reset_position[1],
                self.reset_position[2],
            ],
            [
                self.reset_orientation[1],
                self.reset_orientation[2],
                self.reset_orientation[3],
                self.reset_orientation[0],
            ],
        )

    def drive_straight(self, linear_velocity):
        left_ok = self._set_wheels_velocity(linear_velocity, "left")
        right_ok = self._set_wheels_velocity(linear_velocity, "right")
        return bool(left_ok and right_ok)

    def drive_turn(self, wheel_speed):
        print(wheel_speed)
        if wheel_speed > 0:
            print("turns left")
        else:
            print("turns right")

        left_ok = self._set_wheels_velocity(-wheel_speed, "left")
        right_ok = self._set_wheels_velocity(wheel_speed, "right")
        return bool(left_ok and right_ok)

    def stop_drive(self):
        left_ok = self._set_wheels_velocity(0.0, "left")
        right_ok = self._set_wheels_velocity(0.0, "right")
        return bool(left_ok and right_ok)

    def set_joint_positions(self, targets: dict[str, float]) -> bool:
        """
        Command joint position targets by joint name.

        Args:
            targets:
                {
                    "joint_name": position_rad_or_m,
                    ...
                }
        """
        if not self._articulation_api_enabled:
            print(f"Articulation API is not ready: {self.robot_path}")
            return False

        return self.control.command_positions(targets)

    def set_joint_velocities(self, targets: dict[str, float]) -> bool:
        """
        Command joint velocity targets by joint name.

        Args:
            targets:
                {
                    "joint_name": velocity_rad_s_or_m_s,
                    ...
                }
        """
        if not self._articulation_api_enabled:
            print(f"Articulation API is not ready: {self.robot_path}")
            return False

        return self.control.command_velocities(targets)

    def set_joint_efforts(self, targets: dict[str, float]) -> bool:
        """
        Command joint effort targets by joint name.

        Args:
            targets:
                {
                    "joint_name": effort_Nm_or_N,
                    ...
                }
        """
        if not self._articulation_api_enabled:
            print(f"Articulation API is not ready: {self.robot_path}")
            return False

        return self.control.command_efforts(targets)

    def set_joint_targets(
        self,
        *,
        position_targets: dict[str, float] | None = None,
        velocity_targets: dict[str, float] | None = None,
        effort_targets: dict[str, float] | None = None,
    ) -> bool:
        """
        Mixed joint command.

        Useful when some joints are position-controlled and others are velocity/effort-controlled.
        """
        if not self._articulation_api_enabled:
            print(f"Articulation API is not ready: {self.robot_path}")
            return False

        return self.control.command_mixed(
            position_targets=position_targets or {},
            velocity_targets=velocity_targets or {},
            effort_targets=effort_targets or {},
        )

    def _set_wheels_velocity(self, velocity, side: str):
        if not self._wheel_joint_names:
            return False

        if side not in ["left", "right"]:
            print("Wrong side param:", side, "Side can only be [left] or [right].")
            return False

        targets = {joint_name: float(velocity) for joint_name in self._wheel_joint_names.get(side, [])}

        return self.set_joint_velocities(targets)

    def get_wheels_joint_angles(self):
        return list(self._last_wheel_joint_angles)

    def get_wheel_joint_names(self, side: str | None = None) -> list[str]:
        if not self._wheel_joint_names:
            return []

        if side is None:
            names = []
            for rover_side in ["left", "right"]:
                names.extend(self._wheel_joint_names.get(rover_side, []))
            return names

        if side not in ["left", "right"]:
            raise ValueError("side must be 'left', 'right', or None")

        return list(self._wheel_joint_names.get(side, []))

    def get_joint_positions(self) -> dict[str, float]:
        return dict(self._last_joint_positions)

    def get_joint_velocities(self) -> dict[str, float]:
        return dict(self._last_joint_velocities)

    def get_joint_efforts(self) -> dict[str, float]:
        return dict(self._last_joint_efforts)

    def get_available_joint_names(self) -> list[str]:
        if self.control.is_ready:
            return list(self.control.joint_names)

        return list(self._last_joint_positions.keys())

    def _verify_solar_panel_joint(self) -> None:
        if not self._solar_panel_joint:
            raise Exception(
                "Solar panel joint is not specified. "
                "Please check your .yaml configuration file. "
                "'solar_panel_joint' should be defined on the same level as 'robot_name'."
            )

        if not self._articulation_api_enabled:
            raise RuntimeError(
                f"Articulation API is not ready: {self.robot_path}. "
                "Call update_articulation_api() from the simulation loop first."
            )

        if self._solar_panel_joint not in self.control.joint_name_to_index:
            raise RuntimeError(
                f"Solar panel joint '{self._solar_panel_joint}' was not found in articulation "
                f"{self.robot_path}. Available joints: {self.control.joint_names}"
            )

    def set_solar_panel_angle(self, angle_deg: float):
        self._verify_solar_panel_joint()
        self.set_joint_positions({self._solar_panel_joint: math.radians(angle_deg)})


# TODO for v4: rethink which methods should be in RRG, what should be in Robot
# TODO for v4: separate into a different file (very complex and lengthy classes)
class RobotRigidGroup:
    """
    Class which deals with the target link rigid bodies of a single robot (one batched RigidPrim).
    It is used to retrieve world pose, and contact forces, or apply force/torque.
    """

    def __init__(
        self, root_path: str = "/Robots", robot_name: str = None, target_links: List[str] = None, base_link: str = None
    ):
        """
        Args:
            root_path (str): The root path of the robots.
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names.
        """

        self.root_path = root_path
        self.robot_name = robot_name
        self.target_links = target_links
        self.links: Optional[RigidPrim] = None
        self.base_link = base_link
        self.base_prim_path = None

    def initialize(self, sim: Simulation) -> None:
        """
        Initialize the target link rigid bodies and the base link of the robot.

        Args:
            sim (Simulation): The simulation, used to (re)initialize physics.
        """

        self.dt = sim.get_physics_dt()
        # Create the links on a stopped simulation: RigidPrim builds its physics views when the simulation plays,
        # after PhysX has parsed the contact reporting authored here.
        sim.stop()
        self._initialize_target_links(sim)
        self._initialize_base_link()
        sim.reset()

        print("initialized")

    def _initialize_target_links(self, sim: Simulation):
        if not self.target_links:
            return
        paths = [os.path.join(self.root_path, self.robot_name, link) for link in self.target_links]
        print(self.target_links)
        # max_contact_count > 0 makes RigidPrim create the contact view behind get_net_contact_forces().
        self.links = RigidPrim(paths, resolve_paths=False, max_contact_count=1)
        # Contact reporting setup, as the deprecated RigidPrim(track_contact_forces=True) did: report all
        # contacts, never sleep (sleeping bodies report no contacts), and disable scene stabilization.
        self.links.set_enabled_contact_tracking(True, threshold=0.0)
        self.links.set_sleep_thresholds(0.0)
        sim.physics_scene.set_enabled_stabilization(False)

    def _initialize_base_link(self):
        if not self.base_link:
            raise ValueError(
                f"Robot '{self.robot_name}' is missing required 'base_link' in its YAML configuration. "
                'Please add e.g. base_link: "base_link" under the robot\'s parameters.'
            )
        # The base link only needs its world pose, read from USD (no physics view needed).
        base_prim_path = os.path.join(self.root_path, self.robot_name, self.base_link)
        if not prim_utils.is_prim_valid(base_prim_path):
            raise ValueError(f"Robot '{self.robot_name}' base link prim not found at '{base_prim_path}'.")
        self.base_prim_path = base_prim_path
        print("initialized base link")

    def get_world_poses(self) -> np.ndarray:
        """
        Returns the world pose matrix of target links.

        Returns:
            pose (np.ndarray): The world pose matrix of target links.
        """

        n_links = len(self.target_links)
        pose = np.zeros((n_links, 4, 4))
        for i, (position, orientation) in enumerate(zip(*self._get_link_world_poses())):
            orientation = R.from_quat(orientation, scalar_first=True).as_matrix()
            pose[i, :3, 3] = 1
            pose[i, :3, :3] = orientation
            pose[i, :3, 3] = position
        return pose

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns the pose (position and orientation) of target links in the global frame.

        Notes:
        - Orientations are quaternions in (w, x, y, z) format.
        - The local coordinate system of each wheel rotates as the wheels rotate.
          To ensure consistent orientations in the global frame, the pitch rotation
          is removed. This aligns each wheel's local coordinate system with the global frame.

        Returns:
            positions (np.ndarray): The position of target links. (x, y, z)
            orientations (np.ndarray): The orientation of target links. (w, x, y, z)
        """

        n_links = len(self.target_links)
        positions = np.zeros((n_links, 3))
        orientations = np.zeros((n_links, 4))
        for i, (position, orientation) in enumerate(zip(*self._get_link_world_poses())):
            # Rearrange quaternion from (w, x, y, z) to (x, y, z, w) for scipy
            quaternion = [orientation[1], orientation[2], orientation[3], orientation[0]]
            rotation = R.from_quat(quaternion)

            # Remove pitch rotation to align wheel's local frame with global frame
            pitch_angle = 2 * np.arctan2(rotation.as_quat()[1], rotation.as_quat()[3])
            pitch_correction_quat = [0, -np.sin(pitch_angle / 2), 0, np.cos(pitch_angle / 2)]
            inverse_pitch_rotation = R.from_quat(pitch_correction_quat)
            rotation_corrected = rotation * inverse_pitch_rotation

            # Convert back to (w, x, y, z) and store results
            quaternion_corrected = rotation_corrected.as_quat()
            orientation_corrected = [
                quaternion_corrected[3],
                quaternion_corrected[0],
                quaternion_corrected[1],
                quaternion_corrected[2],
            ]
            positions[i, :] = position
            orientations[i, :] = orientation_corrected
        return positions, orientations

    def get_pose_of_base_link(self) -> Tuple[list, list]:
        """
        Returns a pair of value representing the robot's pose, and orientation respectively, based on the base_link.

        Returns:
            position (np.ndarray): The position of base link. (x, y, z)
            orientation (np.ndarray): The orientation of base link. (w, x, y, z)
        """
        position, orientation = xform_utils.get_world_pose(self.base_prim_path, device="cpu")

        return position.numpy(), orientation.numpy()

    def get_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns the linear/angular velocity of target links.

        Returns:
            linear_velocities (np.ndarray): The linear velocity of target links.
            angular_velocities (np.ndarray): The angular velocity of target links.
        """

        if self.links is None:
            return np.zeros((0, 3)), np.zeros((0, 3))
        linear_velocities, angular_velocities = self.links.get_velocities()
        return linear_velocities.numpy().astype(np.float64), angular_velocities.numpy().astype(np.float64)

    def get_net_contact_forces(self) -> np.ndarray:
        """
        Returns net contact forces on each target link.

        Returns:
            contact_forces (np.ndarray): The net contact forces on each target link.
        """

        if self.links is None:
            return np.zeros((0, 3))
        return self.links.get_net_contact_forces(dt=self.dt).numpy().astype(np.float64)

    def apply_force_torque(self, forces: np.ndarray, torques: np.ndarray) -> None:
        """
        Apply force and torque (defined in local body frame) to body frame of the four wheels.

        Args:
            forces (np.ndarray): The forces to apply to the body origin of the four wheels.
                                 (Fx, Fy, Fz) = (F_DP, F_S, F_N)
            torques (np.ndarray): The torques to apply to the body origin of the four wheels.
                                 (Mx, My, Mz0 = (M_O,-M_R, M_S)
        """

        n_links = len(self.target_links)
        assert forces.shape[0] == n_links, "given force does not have matching shape."
        assert torques.shape[0] == n_links, "given torque does not have matching shape."
        if self.links is not None:
            self.links.apply_forces_and_torques_at_pos(forces=forces, torques=torques, local_frame=True)

    def _get_link_world_poses(self) -> Tuple[np.ndarray, np.ndarray]:
        """Target link positions (N, 3) and orientations (N, 4, wxyz)."""
        if self.links is None:
            return np.zeros((0, 3)), np.zeros((0, 4))
        positions, orientations = self.links.get_world_poses()
        return positions.numpy(), orientations.numpy()
