__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import threading
import time
from typing import Dict, List, Tuple
from scipy.spatial.transform import Rotation as R
import numpy as np
import warnings
import os

import omni
from isaacsim.core.api.world import World
import omni.graph.core as og
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from isaacsim.core.prims import SingleRigidPrim, RigidPrim
from pxr import Gf, Usd

from WorldBuilders.pxr_utils import createXform, createObject
from src.configurations.robot_confs import RobotManagerConf
import numpy as np
from scipy.spatial.transform import Rotation as R

from src.environments.utils import transform_orientation_into_xyz
from src.robots.subsystems_manager import RobotSubsystemsManager
from src.robots.yamcs_TMTC import YamcsTMTC
from omni.isaac.sensor import Camera

from isaacsim.sensors.physics import _sensor

class RobotManager:
    """
    RobotManager class.
    It allows to spawn, reset, teleport robots. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""

    def __init__(
        self,
        RM_conf: RobotManagerConf,
    ) -> None:
        """
        Args:
            RM_conf (RobotManagerConf): The configuration of the robot manager.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.RM_conf = RobotManagerConf(**RM_conf)
        self.robot_parameters = self.RM_conf.parameters
        self.uses_nucleus = self.RM_conf.uses_nucleus
        self.is_ROS2 = self.RM_conf.is_ROS2
        self.max_robots = self.RM_conf.max_robots
        self.robots_root = self.RM_conf.robots_root
        createXform(self.stage, self.robots_root)
        self.robots: Dict[str, Robot] = {}
        self.robots_RG: Dict[str, RobotRigidGroup] = {}
        self.TMTC: YamcsTMTC
        self.num_robots = 0

    def preload_robot(
        self,
        world: World,
    ) -> None:
        """
        Preload the robots in the scene.
        Args:
            world (Usd.Stage): The usd stage scene.
        """
        if len(self.robot_parameters) > 0:
            for robot_parameter in self.robot_parameters:
                self.add_robot(
                    robot_parameter.usd_path,
                    robot_parameter.robot_name,
                    robot_parameter.pose.position,
                    robot_parameter.pose.orientation,
                    robot_parameter.domain_id,
                    robot_parameter.wheel_joints,
                    robot_parameter.camera,
                    robot_parameter.imu_sensor_path,
                    robot_parameter.dimensions,
                )
                self.add_RRG(
                    robot_parameter.robot_name,
                    robot_parameter.target_links,
                    robot_parameter.base_link,
                    world,
                )

    def preload_robot_at_pose(
        self,
        world: World,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> None:
        """
        Preload the robots in the scene.
        Args:
            world (Usd.Stage): The usd stage scene.
            position (Tuple[float, float, float]): The position of the robot. (x, y, z)
            orientation (Tuple[float, float, float, float]): The orientation of the robot. (w, x, y, z)
        """
        if len(self.robot_parameters) > 0:
            for robot_parameter in self.robot_parameters:
                self.add_robot(
                    robot_parameter.usd_path,
                    robot_parameter.robot_name,
                    position,
                    orientation,
                    robot_parameter.domain_id,
                    robot_parameter.wheel_joints,
                    robot_parameter.camera,
                    robot_parameter.imu_sensor_path,
                    robot_parameter.dimensions,
                )
                self.add_RRG(
                    robot_parameter.robot_name,
                    robot_parameter.target_links,
                    robot_parameter.base_link,
                    world,
                )

    def add_robot(
        self,
        usd_path: str = None,
        robot_name: str = None,
        p: Tuple[float, float, float] = [0, 0, 0],
        q: Tuple[float, float, float, float] = [0, 0, 0, 1],
        domain_id: int = None,
        wheel_joints: dict = {},
        camera_conf :dict={},
        imu_sensor_path:str="",
        dimensions:dict={},
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
            robot_name = "/" + robot_name
        if self.num_robots >= self.max_robots:
            pass
        else:
            if robot_name in self.robots.keys():
                warnings.warn("Robot already exists. Ignoring request.")
            else:
                self.robots[robot_name] = Robot(
                    usd_path,
                    robot_name,
                    is_on_nucleus=self.uses_nucleus,
                    is_ROS2=self.is_ROS2,
                    domain_id=domain_id,
                    robots_root=self.robots_root,
                    wheel_joints=wheel_joints,
                    camera_conf=camera_conf,
                    imu_sensor_path=imu_sensor_path,
                    dimensions=dimensions
                )
                self.robots[robot_name].load(p, q)
                self.num_robots += 1

    def add_RRG(
        self,
        robot_name: str = None,
        target_links: List[str] = None,
        pose_base_link: str = None,
        world = None,
    ) -> None:
        """
        Add a robot rigid group to the scene.

        Args:
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names.
            world (Usd.Stage): usd stage scene.
        """
        rrg = RobotRigidGroup(
            self.robots_root,
            robot_name,
            target_links,
            pose_base_link,
        )
        rrg.initialize(world)
        self.robots_RG[robot_name] = rrg

    def reset_robots(self) -> None:
        """
        Reset all the robots to their original position.
        """

        for robot in self.robots.keys():
            self.robots[robot].reset()

    def reset_robot(self, robot_name: str = None) -> None:
        """
        Reset a specific robot to its original position.

        Args:
            robot_name (str): The name of the robot.
        """

        if robot_name in self.robots.keys():
            self.robots[robot_name].reset()
        else:
            warnings.warn("Robot does not exist. Ignoring request.")

    def teleport_robot(
        self, robot_name: str = None, position: np.ndarray = None, orientation: np.ndarray = None
    ) -> None:
        """
        Teleport a specific robot to a specific position and orientation.

        Args:
            robot_name (str): The name of the robot.
        """
        if robot_name in self.robots.keys():
            self.robots[robot_name].teleport(position, orientation)
        else:
            warnings.warn("Robot does not exist. Ignoring request.")
            print("available robots: ", self.robots.keys())

    def start_TMTC(self):
        robot_name = list(self.robots.keys())[0].replace("/","") # assumes only 1 robot for workshop use
        self.TMTC = YamcsTMTC(self.RM_conf.yamcs_tmtc, robot_name, self.robots_RG, self.robots["/" + robot_name])
        self.TMTC.start_streaming_data()

class Robot:
    """
    Robot class.
    It allows to spawn, reset, teleport a robot. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation.
    """

    def __init__(
        self,
        usd_path: str,
        robot_name: str,
        robots_root: str = "/Robots",
        is_on_nucleus: bool = False,
        is_ROS2: bool = False,
        domain_id: int = 0,
        wheel_joints: Dict = {},
        camera_conf:Dict = {},
        imu_sensor_path:str = "",
        dimensions:dict = {},

    ) -> None:
        """
        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            robots_root (str, optional): The root path of the robots. Defaults to "/Robots".
            is_on_nucleus (bool, optional): Whether the robots are loaded from the nucleus or not. Defaults to False.
            is_ROS2 (bool, optional): Whether the robots are ROS2 enabled or not. Defaults to False.
            domain_id (int, optional): The domain id of the robot. Defaults to 0."""

        self.stage: Usd.Stage = omni.usd.get_context().get_stage()
        self.usd_path = str(usd_path)
        self.robots_root = robots_root
        self.robot_name = robot_name
        self.robot_path = os.path.join(self.robots_root, self.robot_name.strip("/"))
        self.is_on_nucleus = is_on_nucleus
        self.is_ROS2 = is_ROS2
        self.domain_id = int(domain_id)
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.root_body_id = None
        self._wheel_joint_names = wheel_joints
        self._dofs = {} # dof = Degree of Freedom
        self._camera_conf = camera_conf
        self._cameras = {}
        self._depth_cameras = {}
        self.subsystems = RobotSubsystemsManager()
        self._imu_sensor_interface = _sensor.acquire_imu_sensor_interface()
        self._imu_sensor_path:str = imu_sensor_path
        self.dimensions = dimensions

    def get_root_rigid_body_path(self) -> None:
        """
        Get the root rigid body path of the robot.
        """

        art = self.dc.get_articulation(self.robot_path)
        self.root_body_id = self.dc.get_articulation_root_body(art)

    def _get_art(self):
        return self.dc.get_articulation(self.robot_path)

    def edit_graphs(self) -> None:
        """
        Edit the graphs of the robot to add namespaces to topics and tfs.
        """

        selected_paths = []
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.robot_path)):
            l = [attr for attr in prim.GetAttributes() if attr.GetName().split(":")[0] == "graph"]
            if l:
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
        if self.is_on_nucleus:
            nucleus = get_assets_root_path()
            self.usd_path = os.path.join(nucleus, self.usd_path)
        createObject(
            self.robot_path,
            self.stage,
            self.usd_path,
            is_instance=False,
            position=Gf.Vec3d(*position),
            rotation=Gf.Quatd(*orientation),
        )
        self.edit_graphs()
        self._initialize_cameras()
        
    def _initialize_cameras(self) -> None:
        # Camera is a wrapper, therefore it just wraps around the camera instance if it already exists
        # otherwise it creates a new camera instance on the provided prim_path
        if "resolutions" not in self._camera_conf:
            return
        
        resolutions = list(self._camera_conf.get("resolutions").keys())

        for res in resolutions:
            self._cameras[res] = Camera(self._camera_conf["prim_path"], 
                                resolution=(self._camera_conf["resolutions"][res][0], self._camera_conf["resolutions"][res][1]))
            self._cameras[res].initialize()

        for res in resolutions:
            self._depth_cameras[res] = Camera(self._camera_conf["prim_path"], 
                                resolution=(self._camera_conf["resolutions"][res][0], self._camera_conf["resolutions"][res][1]))
            self._depth_cameras[res].initialize()
            self._depth_cameras[res].add_distance_to_image_plane_to_frame()

    def get_rgba_camera_view(self, resolution) -> np.ndarray:
        return self._cameras[resolution].get_rgba()
    
    def get_depth_camera_view(self, resolution) -> np.ndarray:
        """Returns depth image in meters as (H, W) float32 array."""
        depth = self._depth_cameras[resolution].get_depth()
        print("depth")
        print(depth)
        return depth
    
    def get_imu_readings(self):
        # https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physics_imu.html#reading-sensor-output
        sensor_reading = self._imu_sensor_interface.get_sensor_reading(self._imu_sensor_path, use_latest_data = True, read_gravity = True)
        linear_acceleration = {"ax": sensor_reading.lin_acc_x, "ay": sensor_reading.lin_acc_y, "az": sensor_reading.lin_acc_z}
        angular_velocity = {"gx":sensor_reading.ang_vel_x, "gy":sensor_reading.ang_vel_y, "gz":sensor_reading.ang_vel_z} 
        
        orientation = sensor_reading.orientation # w, x, y, z
        xyz_orientation = transform_orientation_into_xyz(orientation) 
        orientation = {"roll":float(xyz_orientation[0]), "pitch":float(xyz_orientation[1]), "yaw":float(xyz_orientation[2])}

        # print(linear_acceleration, angular_velocity, orientation)
        return linear_acceleration, angular_velocity, orientation


    def get_pose(self) -> List[float]:
        """
        Get the pose of the robot.
        Returns:
            List[float]: The pose of the robot. (x, y, z), (qx, qy, qz, qw)
        """
        if self.root_body_id is None:
            self.get_root_rigid_body_path()
        pose = self.dc.get_rigid_body_pose(self.root_body_id)
        return pose.p, pose.r
    

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
            p (list): The position of the robot.
            q (list): The orientation of the robot. (x, y, z, w)
        """

        self.get_root_rigid_body_path()
        transform = _dynamic_control.Transform(p, q)
        self.dc.set_rigid_body_pose(self.root_body_id, transform)
        self.dc.set_rigid_body_linear_velocity(self.root_body_id, [0, 0, 0])
        self.dc.set_rigid_body_angular_velocity(self.root_body_id, [0, 0, 0])

    def reset(self) -> None:
        """
        Reset the robot to its original position and orientation.
        """

        # w = self.reset_orientation.GetReal()
        # xyz = self.reset_orientation.GetImaginary()
        self.root_body_id = None
        self.teleport(
            [self.reset_position[0], self.reset_position[1], self.reset_position[2]],
            [
                self.reset_orientation[1],
                self.reset_orientation[2],
                self.reset_orientation[3],
                self.reset_orientation[0],
            ],
        )

    def drive_straight(self, linear_velocity):
        self._set_wheels_velocity(linear_velocity, "left")
        self._set_wheels_velocity(linear_velocity, "right")

    def drive_turn(self, wheel_speed):
        print(wheel_speed)
        if (wheel_speed > 0):
            print("turns left")
        else:
            print("turns right")
        self._set_wheels_velocity(-wheel_speed, "left")
        self._set_wheels_velocity(wheel_speed, "right")

    def stop_drive(self):
        self._set_wheels_velocity(0, "left")
        self._set_wheels_velocity(0, "right")

    def _set_wheels_velocity(self, velocity, side:str):
        self._init_dofs()

        if side not in ["left","right"]:
            print("Wrong side param:", side, "Side can only be [left] or [right].")
            return

        for dof in self._dofs[side]:
            self.dc.set_dof_velocity_target(dof, velocity)

    def _init_dofs(self):
        #NOTE idealy, this would be initialized inside load(),
        # however, for an unknown reason art, and dc do not work well when invoked there
        # thus not populating dofs correctly
        # therefore, it was implemented as singleton, and should be called at the begging of every commanding function
        #
        # more about the use of dofs for robot movement can be read on: 
        # https://docs.isaacsim.omniverse.nvidia.com/5.0.0/python_scripting/robots_simulation.html#velocity-control
        if not self._wheel_joint_names:
            return

        if "left" in list(self._dofs.keys()):
            return # it means it is already initialized
        
        self._dofs = {
            "left": [],
            "right": []
        }
        art = self._get_art()

        for rover_side in ["left", "right"]:
            for joint_name in self._wheel_joint_names[rover_side]:
                dof = self.dc.find_articulation_dof(art, joint_name)
                self._dofs[rover_side].append(dof)


class RobotRigidGroup:
    """
    Class which deals with rigidprims and rigidprimview of a single robot.
    It is used to retrieve world pose, and contact forces, or apply force/torque.
    """

    def __init__(self, root_path: str = "/Robots", robot_name: str = None, target_links: List[str] = None, base_link:str=None):
        """
        Args:
            root_path (str): The root path of the robots.
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names.
        """

        self.root_path = root_path
        self.robot_name = robot_name
        self.target_links = target_links
        self.prims = []
        self.prim_views = []
        self.base_link = base_link
        self.base_prim = None

    def initialize(self, world: World) -> None:
        """
        Initialize the rigidprims and rigidprimviews of the robot.

        Args:
            world (World): A Omni.isaac.core.world.World object.
        """

        self.dt = world.get_physics_dt()
        world.reset()
        self._initialize_target_links()
        self._initialize_base_link()
        world.reset()

        print("initialized")

    def _initialize_target_links(self):
        if len(self.target_links) > 0:
            for target_link in self.target_links:
                print(target_link)
                rigid_prim, rigid_prim_view = self._initialize_link(target_link)
                self.prims.append(rigid_prim)
                self.prim_views.append(rigid_prim_view)

    def _initialize_base_link(self):
        rigid_prim, rigid_prim_view = self._initialize_link(self.base_link)
        self.base_prim = rigid_prim
        print("initialized base link")

    def _initialize_link(self, link):
        rigid_prim = SingleRigidPrim(
            prim_path=os.path.join(self.root_path, self.robot_name, link),
            name=f"{self.robot_name}/{link}",
        )
        rigid_prim_view = RigidPrim(
            prim_paths_expr=os.path.join(self.root_path, self.robot_name, link),
            name=f"{self.robot_name}/{link}_view",
            track_contact_forces=True,
        )
        rigid_prim_view.initialize()

        return rigid_prim, rigid_prim_view

    def get_world_poses(self) -> np.ndarray:
        """
        Returns the world pose matrix of target links.

        Returns:
            pose (np.ndarray): The world pose matrix of target links.
        """

        n_links = len(self.target_links)
        pose = np.zeros((n_links, 4, 4))
        for i, prim in enumerate(self.prims):
            position, orientation = prim.get_world_pose()
            orientation = quat_to_rot_matrix(orientation)
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
        for i, prim in enumerate(self.prims):
            position, orientation = prim.get_world_pose()

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
            orientation_corrected = [quaternion_corrected[3], quaternion_corrected[0], quaternion_corrected[1], quaternion_corrected[2]]
            positions[i, :] = position
            orientations[i, :] = orientation_corrected
        return positions, orientations
    
    def get_pose_of_base_link(self) -> Tuple[list, list]:
        """
        Returns a pair of value representing the robot's pose, and orientation respectively, based on the base_link.

        Returns:
            position (np.ndarray): The position of base link. (x, y, z)
            orientation (np.ndarray): The orientation of base link. (x, y, z, w)
        """
        position, orientation = self.base_prim.get_world_pose()

        return position, orientation

    def get_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns the linear/angular velocity of target links.

        Returns:
            linear_velocities (np.ndarray): The linear velocity of target links.
            angular_velocities (np.ndarray): The angular velocity of target links.
        """

        n_links = len(self.target_links)
        linear_velocities = np.zeros((n_links, 3))
        angular_velocities = np.zeros((n_links, 3))
        for i, prim in enumerate(self.prims):
            linear_velocity, angular_velocity = prim.get_velocities()
            linear_velocities[i, :] = linear_velocity
            angular_velocities[i, :] = angular_velocity
        return linear_velocities, angular_velocities

    def get_net_contact_forces(self) -> np.ndarray:
        """
        Returns net contact forces on each target link.

        Returns:
            contact_forces (np.ndarray): The net contact forces on each target link.
        """

        n_links = len(self.target_links)
        contact_forces = np.zeros((n_links, 3))
        for i, prim_view in enumerate(self.prim_views):
            contact_force = prim_view.get_net_contact_forces(dt = self.dt).squeeze()
            contact_forces[i, :] = contact_force
        return contact_forces

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
        for i, prim_view in enumerate(self.prim_views):
            prim_view.apply_forces_and_torques_at_pos(forces=forces[i], torques=torques[i], is_global=False)
