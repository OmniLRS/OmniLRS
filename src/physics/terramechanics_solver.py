__author__ = "Junnosuke Kamohara"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import numpy as np
import torch
import scipy.integrate as integ

from src.physics.terramechanics_parameters import RobotParameter, TerrainMechanicalParameter


class TerramechanicsSolver:
    """
    WIP.
    Terramechanics solver class. This class computes the force and torque acting on the robot as a disturbance.
    Fx: Draw bar pull
    Fy: Side slip force
    Fz: Normal force
    Mx: Resistance torque
    My: Overturn torque
    Mz: Steering resistance torque"""

    def __init__(self, robot_param: RobotParameter = None, terrain_param: TerrainMechanicalParameter = None):
        self.robot_param = robot_param
        self.terrain_param = terrain_param
        self.num_wheels = robot_param.num_wheels

    ## Eventually, input to this solver is sinkage z and velocity, and angular velocity
    def compute_slip_ratio(self, v: float, omega: float) -> np.ndarray:
        """
        Compute the slip ratio.
        Args:
            v (float): The linear velocity of the robot.
            w (float): The angular velocity of the robot."""
        if v <= omega * self.robot_param.wheel_radius:
            self.slip_ratio = 1 - v / (omega * self.robot_param.wheel_radius)
        else:
            self.slip_ratio = omega * self.robot_param.wheel_radius / v - 1

    def compute_thetas(self, z: float) -> None:
        """
        Compute theta_f, theta_r, theta_m.
        Args:
            z (np.ndarray): sinkage (num_wheels, )
            slip_ratio (np.ndarray): The slip ratio (num_wheels, )"""
        self.theta_f = np.arctan(1 - z / self.robot_param.wheel_radius)
        self.theta_r = np.zeros_like(self.theta_f)
        self.theta_m = (self.terrain_param.a_0 + self.terrain_param.a_1 * self.slip_ratio) * self.theta_f

    def sigma_cos_theta_lower(self, theta):
        return self.sigma_max * (np.cos(theta) - np.cos(self.theta_f)) * np.cos(theta)

    def sigma_sin_theta_lower(self, theta):
        return self.sigma_max * (np.cos(theta) - np.cos(self.theta_f)) * np.sin(theta)

    def sigma_cos_theta_upper(self, theta):
        return (
            self.sigma_max
            * (
                np.cos(
                    self.theta_f
                    - ((theta - self.theta_r) / (self.theta_m - self.theta_r)) * (self.theta_f - self.theta_m)
                )
                - np.cos(self.theta_f)
            )
            * np.cos(theta)
        )

    def sigma_sin_theta_upper(self, theta):
        return (
            self.sigma_max
            * (
                np.cos(
                    self.theta_f
                    - ((theta - self.theta_r) / (self.theta_m - self.theta_r)) * (self.theta_f - self.theta_m)
                )
                - np.cos(self.theta_f)
            )
            * np.sin(theta)
        )

    def tau_theta_lower(self, theta):
        j_theta = self.robot_param.wheel_radius * (
            self.theta_f - theta - (1 - self.slip_ratio) * (np.sin(self.theta_f) - np.sin(theta))
        )
        sigma = self.sigma_max * (np.cos(theta) - np.cos(self.theta_f))
        return (self.terrain_param.c + sigma * np.tan(self.terrain_param.phi)) * (
            1 - np.exp(-j_theta / self.terrain_param.K)
        )

    def tau_cos_theta_lower(self, theta):
        j_theta = self.robot_param.wheel_radius * (
            self.theta_f - theta - (1 - self.slip_ratio) * (np.sin(self.theta_f) - np.sin(theta))
        )
        sigma = self.sigma_max * (np.cos(theta) - np.cos(self.theta_f))
        return (
            (self.terrain_param.c + sigma * np.tan(self.terrain_param.phi))
            * (1 - np.exp(-j_theta / self.terrain_param.K))
            * np.cos(theta)
        )

    def tau_sin_theta_lower(self, theta):
        j_theta = self.robot_param.wheel_radius * (
            self.theta_f - theta - (1 - self.slip_ratio) * (np.sin(self.theta_f) - np.sin(theta))
        )
        sigma = self.sigma_max * (np.cos(theta) - np.cos(self.theta_f))
        return (
            (self.terrain_param.c + sigma * np.tan(self.terrain_param.phi))
            * (1 - np.exp(-j_theta / self.terrain_param.K))
            * np.sin(theta)
        )

    def tau_theta_upper(self, theta):
        j_theta = self.robot_param.wheel_radius * (
            self.theta_f - theta - (1 - self.slip_ratio) * (np.sin(self.theta_f) - np.sin(theta))
        )
        sigma = self.sigma_max * (
            np.cos(
                self.theta_f - ((theta - self.theta_r) / (self.theta_m - self.theta_r)) * (self.theta_f - self.theta_m)
            )
            - np.cos(self.theta_f)
        )
        return (self.terrain_param.c + sigma * np.tan(self.terrain_param.phi)) * (
            1 - np.exp(-j_theta / self.terrain_param.K)
        )

    def tau_cos_theta_upper(self, theta):
        j_theta = self.robot_param.wheel_radius * (
            self.theta_f - theta - (1 - self.slip_ratio) * (np.sin(self.theta_f) - np.sin(theta))
        )
        sigma = self.sigma_max * (
            np.cos(
                self.theta_f - ((theta - self.theta_r) / (self.theta_m - self.theta_r)) * (self.theta_f - self.theta_m)
            )
            - np.cos(self.theta_f)
        )
        return (
            (self.terrain_param.c + sigma * np.tan(self.terrain_param.phi))
            * (1 - np.exp(-j_theta / self.terrain_param.K))
            * np.cos(theta)
        )

    def tau_sin_theta_upper(self, theta):
        j_theta = self.robot_param.wheel_radius * (
            self.theta_f - theta - (1 - self.slip_ratio) * (np.sin(self.theta_f) - np.sin(theta))
        )
        sigma = self.sigma_max * (
            np.cos(
                self.theta_f - ((theta - self.theta_r) / (self.theta_m - self.theta_r)) * (self.theta_f - self.theta_m)
            )
            - np.cos(self.theta_f)
        )
        return (
            (self.terrain_param.c + sigma * np.tan(self.terrain_param.phi))
            * (1 - np.exp(-j_theta / self.terrain_param.K))
            * np.sin(theta)
        )

    def compute_sigma_max(self):
        self.sigma_max = (
            (self.terrain_param.c * self.terrain_param.k_c)
            + (self.terrain_param.rho * 9.81 * self.robot_param.wheel_base * self.terrain_param.k_phi)
        ) * (self.robot_param.wheel_radius / self.robot_param.wheel_base) ** self.terrain_param.n

    def compute_fx(self):
        return (
            self.robot_param.wheel_radius
            * self.robot_param.wheel_base
            * (
                integ.quad(self.tau_cos_theta_upper, self.theta_r, self.theta_m)
                + integ.quad(self.tau_cos_theta_lower, self.theta_m, self.theta_f)
                - integ.quad(self.sigma_sin_theta_upper, self.theta_r, self.theta_m)
                - integ.quad(self.sigma_sin_theta_lower, self.theta_m, self.theta_f)
            )
        )

    def compute_fz(self):
        return (
            self.robot_param.wheel_radius
            * self.robot_param.wheel_base
            * (
                integ.quad(self.tau_sin_theta_upper, self.theta_r, self.theta_m)
                + integ.quad(self.tau_sin_theta_lower, self.theta_m, self.theta_f)
                - integ.quad(self.sigma_cos_theta_upper, self.theta_r, self.theta_m)
                - integ.quad(self.sigma_cos_theta_lower, self.theta_m, self.theta_f)
            )
        )

    def compute_my(self):
        return (
            (self.robot_param.wheel_radius**2)
            * self.robot_param.wheel_base
            * (
                integ.quad(self.tau_theta_upper, self.theta_r, self.theta_m)
                + integ.quad(self.tau_theta_lower, self.theta_m, self.theta_f)
            )
        )

    def compute_force_and_torque(self, velocity: np.ndarray, omega: np.ndarray, sinkage: np.ndarray) -> np.ndarray:
        """
        Computes the force and torque.
        Args:
            velocity (np.ndarray): The forward velocity (vx) of the robot (num_wheels, ).
            omega (np.ndarray): The angular velocity of the robot (num_wheels, ).
            sinkage (np.ndarray): The sinkage of the robot (num_wheels, )."""
        forces = np.zeros((self.num_wheels, 3))
        torques = np.zeros((self.num_wheels, 3))
        for i in range(self.num_wheels):
            self.compute_slip_ratio(velocity[i], omega[i])
            self.compute_thetas(sinkage[i])
            self.compute_sigma_max()
            fx = self.compute_fx()
            fy = 0
            fz = self.compute_fz()
            my = self.compute_my()
            forces[i, 0] = fx
            forces[i, 1] = fy
            forces[i, 2] = fz
            torques[i, 0] = 0
            torques[i, 1] = my
            torques[i, 2] = 0
        return forces, torques

    # def compute_force_and_torque(self)->np.ndarray:
    #     """
    #     Computes the force and torque."""
    #     force = 10 * np.random.rand(self.num_wheels, 3)
    #     torque = 10 * np.random.rand(self.num_wheels, 3)
    #     return force, torque
