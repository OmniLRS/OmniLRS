from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen"
__copyright__ = "Copyright 2025-26, JAOPS, AsteriaART/Artefacts"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.types import ArticulationAction

logger = logging.getLogger(__name__)


class JointControlMode(str, Enum):
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
    def __init__(
        self,
        prim_path: str,
        name: str = "articulation_control",
        init_retry_s: float = 0.5,
        apply_period_s: float = 0.0,
        logger=logger,
    ):
        self.prim_path = prim_path
        self.name = name
        self.init_retry_s = float(init_retry_s)
        self.apply_period_s = float(apply_period_s)

        self.log = logger.info

        self.stage = None
        self.articulation: Optional[SingleArticulation] = None

        self.joint_names: List[str] = []
        self.joint_name_to_index: Dict[str, int] = {}

        self._inited = False
        self._t_last_init_try = 0.0
        self._t_last_apply = 0.0

        self._pending_command: Optional[JointCommand] = None

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
        if self.is_ready:
            return True

        now = time.time()
        if now - self._t_last_init_try < self.init_retry_s:
            return False
        self._t_last_init_try = now

        try:
            self.stage = get_current_stage()

            self.articulation = SingleArticulation(
                prim_path=self.prim_path,
                name=self.name,
            )
            self.articulation.initialize()

            q = self._safe_array(self.articulation.get_joint_positions())
            dof_names = self.articulation.dof_names

            if dof_names is None:
                raise RuntimeError(f"No DOF names found for articulation {self.prim_path}")

            self.joint_names = list(dof_names)

            if q is not None and len(q) != len(self.joint_names):
                raise RuntimeError(
                    f"DOF mismatch for {self.prim_path}: " f"positions={len(q)} names={len(self.joint_names)}"
                )

            self.joint_name_to_index = {name: i for i, name in enumerate(self.joint_names)}

            self._inited = True

            self.log(f"ArticulationControl initialized: " f"prim={self.prim_path}, dofs={len(self.joint_names)}")
            self.log(f"DOF names: {self.joint_names}")
            return True

        except Exception as e:
            self.log(f"ArticulationControl init failed, retrying: {e}")
            self._inited = False
            self.articulation = None
            return False

    def set_command(self, command: JointCommand) -> None:
        self._pending_command = command

    def clear_command(self) -> None:
        self._pending_command = None

    def update(self) -> bool:
        if self._pending_command is None:
            return False

        return self.apply_command(self._pending_command)

    def apply_command(self, command: JointCommand) -> bool:
        if not self.is_ready:
            return False

        now = time.time()

        if self.apply_period_s > 0.0 and now - self._t_last_apply < self.apply_period_s:
            return False

        self._t_last_apply = now

        assert self.articulation is not None

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
                articulation=self.articulation,
                kind=JointControlMode.POSITION,
                targets=position_targets,
            )
            applied = True

        if velocity_targets:
            self._apply_targets(
                articulation=self.articulation,
                kind=JointControlMode.VELOCITY,
                targets=velocity_targets,
            )
            applied = True

        if effort_targets:
            self._apply_targets(
                articulation=self.articulation,
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
        self.stage = None
        self.articulation = None

        self.joint_names = []
        self.joint_name_to_index = {}

        self._pending_command = None
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

    @staticmethod
    def _safe_array(x):
        if x is None:
            return None
        try:
            return np.asarray(x)
        except Exception:
            return None
