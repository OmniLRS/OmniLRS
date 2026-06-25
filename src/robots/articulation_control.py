from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

logger = logging.getLogger(__name__)


class JointControlMode(str, Enum):
    """
    Supported joint command modes.

    POSITION:
        Command only joint position targets.
    VELOCITY:
        Command only joint velocity targets.
    EFFORT:
        Command only joint effort/torque targets.
    MIXED:
        Allow position, velocity, and effort targets in the same high-level
        command. This is useful for robots where different joints are driven in
        different ways, for example a rover with velocity-controlled wheels and
        a position-controlled deployment mechanism such as a solar panel.
    """

    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"
    MIXED = "mixed"


@dataclass(frozen=True)
class JointCommand:
    stamp_s: float
    mode: JointControlMode = JointControlMode.MIXED
    position_targets: Dict[str, float] | None = None
    velocity_targets: Dict[str, float] | None = None
    effort_targets: Dict[str, float] | None = None


@dataclass(frozen=True)
class JointCommandStatus:
    stamp_s: float
    prim_path: str
    mode: JointControlMode
    applied: bool
    unknown_joints: List[str]
    n_position_targets: int
    n_velocity_targets: int
    n_effort_targets: int


class ArticulationControl:
    """
    OmniLRS wrapper around Isaac Sim Articulation Controller.

    In Isaac Sim, an articulation is a robot/mechanism made of rigid bodies
    connected by joints and controlled from an articulation root. This class
    wraps one robot articulation with `SingleArticulation` and applies joint
    commands through Isaac Sim's `ArticulationAction` API.

    The class is transport-agnostic and should be used through `Robot`;
    simulation managers are responsible for initializing/updating it
    after the world is stepped and after resets.

    Isaac Sim articulation reference:
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/articulation_controller.html
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.core.prims/docs/index.html#isaacsim.core.prims.SingleArticulation
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "articulation_control",
        init_retry_s: float = 0.5,
        apply_period_s: float = 0.0,
        logger=logger,
    ):
        """
        Create an articulation joint controller.

        Args:
            prim_path:
                USD prim path of the articulation root to control.
            name:
                Name assigned to the Isaac `SingleArticulation` wrapper.
            init_retry_s:
                Minimum wall-clock time between initialization attempts. This
                avoids spamming Isaac initialization while the stage or physics
                view is not ready yet.
            apply_period_s:
                Minimum wall-clock time between command applications. A value
                of 0.0 means every call to `apply_command()` is allowed to send
                an action. A positive value rate-limits command application,
                which is useful when commands arrive faster than the desired
                simulation/control rate.
            logger:
                Logger-like object used for status messages.
        """

        self.prim_path = prim_path
        self.name = name
        self.init_retry_s = float(init_retry_s)
        self.apply_period_s = float(apply_period_s)

        self.log = logger.info

        self.articulation: Optional[SingleArticulation] = None

        self.joint_names: List[str] = []
        self.joint_name_to_index: Dict[str, int] = {}

        self._inited = False
        self._t_last_init_try = 0.0
        self._t_last_apply = 0.0

        self.last_status = JointCommandStatus(
            stamp_s=0.0,
            prim_path=self.prim_path,
            mode=JointControlMode.MIXED,
            applied=False,
            unknown_joints=[],
            n_position_targets=0,
            n_velocity_targets=0,
            n_effort_targets=0,
        )

    @property
    def is_ready(self) -> bool:
        return self._inited and self.articulation is not None

    def maybe_initialize(self) -> bool:
        """
        Initialize the Isaac articulation wrapper if it is not ready yet.

        Returns:
            True when the controller has a valid `SingleArticulation` wrapper
            and joint-name/index mapping, False otherwise.

        Notes:
            This method intentionally does not read joint positions or other
            physics state. Those reads require Isaac's physics simulation view
            to be created, which usually happens only after the world has been
            stepped/playing. Initialization only needs the articulation DOF
            names so commands can later be mapped from joint names to indices.
        """

        if self.is_ready:
            return True

        now = time.time()
        if now - self._t_last_init_try < self.init_retry_s:
            return False
        self._t_last_init_try = now

        try:
            self.articulation = SingleArticulation(
                prim_path=self.prim_path,
                name=self.name,
            )
            self.articulation.initialize()

            dof_names = self.articulation.dof_names

            if dof_names is None:
                raise RuntimeError(f"No DOF names found for articulation {self.prim_path}")

            self.joint_names = list(dof_names)
            self.joint_name_to_index = {name: i for i, name in enumerate(self.joint_names)}

            self._inited = True

            self.log(f"ArticulationControl initialized: " f"prim={self.prim_path}, dofs={len(self.joint_names)}")
            self.log(f"DOF names: {self.joint_names}")
            return True

        except Exception as e:
            self.log(f"ArticulationControl init failed, retrying in {self.init_retry_s} s. Exception was: {e}")
            self._inited = False
            self.articulation = None
            return False

    def apply_command(self, command: JointCommand) -> bool:
        """
        Apply one JointCommand to the articulation.

        The command is given by joint name and may contain position, velocity,
        effort, or mixed targets. The method validates the command mode, filters
        unknown joints, converts joint names to Isaac DOF indices, and sends the
        corresponding `ArticulationAction`.

        Returns:
            True if at least one valid target was applied, otherwise False.
        """

        if not self.is_ready:
            return False

        now = time.time()

        if self.apply_period_s > 0.0 and now - self._t_last_apply < self.apply_period_s:
            return False

        self._t_last_apply = now

        articulation = self.articulation

        position_targets = dict(command.position_targets or {})
        velocity_targets = dict(command.velocity_targets or {})
        effort_targets = dict(command.effort_targets or {})

        self._validate_command_mode(
            mode=command.mode,
            position_targets=position_targets,
            velocity_targets=velocity_targets,
            effort_targets=effort_targets,
        )

        all_names = set(position_targets.keys()) | set(velocity_targets.keys()) | set(effort_targets.keys())

        unknown_joints = sorted(name for name in all_names if name not in self.joint_name_to_index)

        position_targets = self._filter_known_joints(position_targets)
        velocity_targets = self._filter_known_joints(velocity_targets)
        effort_targets = self._filter_known_joints(effort_targets)

        applied = False

        if position_targets:
            self._apply_targets(
                articulation=articulation,
                kind=JointControlMode.POSITION,
                targets=position_targets,
            )
            applied = True

        if velocity_targets:
            self._apply_targets(
                articulation=articulation,
                kind=JointControlMode.VELOCITY,
                targets=velocity_targets,
            )
            applied = True

        if effort_targets:
            self._apply_targets(
                articulation=articulation,
                kind=JointControlMode.EFFORT,
                targets=effort_targets,
            )
            applied = True

        self.last_status = JointCommandStatus(
            stamp_s=now,
            prim_path=self.prim_path,
            mode=command.mode,
            applied=applied,
            unknown_joints=unknown_joints,
            n_position_targets=len(position_targets),
            n_velocity_targets=len(velocity_targets),
            n_effort_targets=len(effort_targets),
        )

        return applied

    def command_positions(self, targets: Dict[str, float]) -> bool:
        return self.apply_command(
            JointCommand(
                stamp_s=time.time(),
                mode=JointControlMode.POSITION,
                position_targets=targets,
            )
        )

    def command_velocities(self, targets: Dict[str, float]) -> bool:
        return self.apply_command(
            JointCommand(
                stamp_s=time.time(),
                mode=JointControlMode.VELOCITY,
                velocity_targets=targets,
            )
        )

    def command_efforts(self, targets: Dict[str, float]) -> bool:
        return self.apply_command(
            JointCommand(
                stamp_s=time.time(),
                mode=JointControlMode.EFFORT,
                effort_targets=targets,
            )
        )

    def command_mixed(
        self,
        position_targets: Dict[str, float] | None = None,
        velocity_targets: Dict[str, float] | None = None,
        effort_targets: Dict[str, float] | None = None,
    ) -> bool:
        return self.apply_command(
            JointCommand(
                stamp_s=time.time(),
                mode=JointControlMode.MIXED,
                position_targets=position_targets or {},
                velocity_targets=velocity_targets or {},
                effort_targets=effort_targets or {},
            )
        )

    def close(self) -> None:
        self.articulation = None

        self.joint_names = []
        self.joint_name_to_index = {}

        self._inited = False

        self.last_status = JointCommandStatus(
            stamp_s=0.0,
            prim_path=self.prim_path,
            mode=JointControlMode.MIXED,
            applied=False,
            unknown_joints=[],
            n_position_targets=0,
            n_velocity_targets=0,
            n_effort_targets=0,
        )

        self.log(f"ArticulationControl closed: {self.prim_path}")

    def _validate_command_mode(
        self,
        mode: JointControlMode,
        position_targets: Dict[str, float],
        velocity_targets: Dict[str, float],
        effort_targets: Dict[str, float],
    ) -> None:
        if mode == JointControlMode.POSITION:
            if velocity_targets or effort_targets:
                raise ValueError("POSITION command cannot contain velocity/effort targets")

        elif mode == JointControlMode.VELOCITY:
            if position_targets or effort_targets:
                raise ValueError("VELOCITY command cannot contain position/effort targets")

        elif mode == JointControlMode.EFFORT:
            if position_targets or velocity_targets:
                raise ValueError("EFFORT command cannot contain position/velocity targets")

        elif mode == JointControlMode.MIXED:
            return

        else:
            raise ValueError(f"Unsupported joint control mode: {mode}")

    def _filter_known_joints(self, targets: Dict[str, float]) -> Dict[str, float]:
        return {name: value for name, value in targets.items() if name in self.joint_name_to_index}

    def _apply_targets(
        self,
        articulation: SingleArticulation,
        kind: JointControlMode,
        targets: Dict[str, float],
    ) -> None:
        names = list(targets.keys())

        indices = np.asarray(
            [self.joint_name_to_index[name] for name in names],
            dtype=np.int64,
        )

        values = np.asarray(
            [targets[name] for name in names],
            dtype=np.float32,
        )

        if kind == JointControlMode.POSITION:
            action = ArticulationAction(
                joint_positions=values,
                joint_indices=indices,
            )

        elif kind == JointControlMode.VELOCITY:
            action = ArticulationAction(
                joint_velocities=values,
                joint_indices=indices,
            )

        elif kind == JointControlMode.EFFORT:
            action = ArticulationAction(
                joint_efforts=values,
                joint_indices=indices,
            )

        else:
            raise ValueError(f"Cannot apply target kind: {kind}")

        articulation.apply_action(action)
