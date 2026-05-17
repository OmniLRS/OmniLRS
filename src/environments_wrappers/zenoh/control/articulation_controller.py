from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.types import ArticulationAction

logger = logging.getLogger(__name__)


@dataclass
class JointCommand:
    stamp_s: float
    mode: str = "mixed"  # "position" | "velocity" | "mixed"
    targets: Dict[str, float] | None = None  # backward compatibility
    position_targets: Dict[str, float] | None = None
    velocity_targets: Dict[str, float] | None = None


class ArticulationController:
    def __init__(
        self,
        prim_path: str = "/Robots/husky/joints/*",
        logger=logger,
        init_retry_s: float = 0.5,
        apply_period_s: float = 0.0,  # 0 = every sim step
    ):
        self.prim_path = prim_path
        self.log = logger.info
        self.init_retry_s = float(init_retry_s)
        self.apply_period_s = float(apply_period_s)

        self.stage = None
        self.rover: Optional[SingleArticulation] = None
        self._inited = False
        self._t_last_init_try = 0.0
        self._t_last_apply = 0.0

        self.joint_names: list[str] = []
        self.joint_name_to_index: Dict[str, int] = {}

        self._cmd: Optional[JointCommand] = None

        self.last_status: Dict[str, Any] = {
            "stamp_s": None,
            "prim_path": self.prim_path,
            "mode": None,
            "applied": False,
            "unknown_joints": [],
        }

    def maybe_initialize(self) -> bool:
        if self._inited:
            return True

        now = time.time()
        if now - self._t_last_init_try < self.init_retry_s:
            return False
        self._t_last_init_try = now

        try:
            self.stage = get_current_stage()
            self.rover = SingleArticulation(prim_path=self.prim_path, name="rover_ctrl")
            self.rover.initialize()

            dof_names = self.rover.dof_names

            self.joint_names = list(dof_names)

            self.joint_name_to_index = {n: i for i, n in enumerate(self.joint_names)}

            self._inited = True
            self.log(f"initialized: dofs={len(self.joint_names)} prim={self.prim_path}")
            self.log(f"dof names: {self.joint_names}")
            return True

        except Exception as e:
            self.log(f"init failed (retrying): {e}")
            self._inited = False
            self.rover = None
            return False

    def set_command(self, cmd: JointCommand) -> None:
        self._cmd = cmd

    def clear_command(self) -> None:
        self._cmd = None

    def update(self) -> bool:
        if not self._inited or self.rover is None or self._cmd is None:
            return False

        now = time.time()
        if self.apply_period_s > 0 and (now - self._t_last_apply) < self.apply_period_s:
            return False
        self._t_last_apply = now

        cmd = self._cmd
        mode = (cmd.mode or "").lower()

        position_targets = dict(cmd.position_targets or {})
        velocity_targets = dict(cmd.velocity_targets or {})

        if cmd.targets:
            if mode == "position":
                position_targets.update(cmd.targets)
            elif mode == "velocity":
                velocity_targets.update(cmd.targets)

        all_names = set(position_targets.keys()) | set(velocity_targets.keys())
        unknown = [j for j in all_names if j not in self.joint_name_to_index]

        position_targets = {
            j: v for j, v in position_targets.items() if j in self.joint_name_to_index
        }
        velocity_targets = {
            j: v for j, v in velocity_targets.items() if j in self.joint_name_to_index
        }

        if not position_targets and not velocity_targets:
            self.last_status = {
                "stamp_s": now,
                "prim_path": self.prim_path,
                "mode": mode,
                "applied": False,
                "unknown_joints": unknown,
            }
            return False

        applied = False

        if position_targets:
            pos_idx = np.array(
                [self.joint_name_to_index[j] for j in position_targets.keys()],
                dtype=np.int64,
            )
            pos_vals = np.array(
                [position_targets[j] for j in position_targets.keys()],
                dtype=np.float32,
            )

            action = ArticulationAction(
                joint_positions=pos_vals,
                joint_indices=pos_idx,
            )
            self.rover.apply_action(action)
            applied = True

        if velocity_targets:
            vel_idx = np.array(
                [self.joint_name_to_index[j] for j in velocity_targets.keys()],
                dtype=np.int64,
            )
            vel_vals = np.array(
                [velocity_targets[j] for j in velocity_targets.keys()],
                dtype=np.float32,
            )

            action = ArticulationAction(
                joint_velocities=vel_vals,
                joint_indices=vel_idx,
            )
            self.rover.apply_action(action)
            applied = True

        self.last_status = {
            "stamp_s": now,
            "prim_path": self.prim_path,
            "mode": mode,
            "applied": applied,
            "unknown_joints": unknown,
            "n_position_targets": int(len(position_targets)),
            "n_velocity_targets": int(len(velocity_targets)),
        }

        return applied

    def close(self) -> None:
        self._inited = False
        self.rover = None
        self.stage = None
        self.joint_names = []
        self.joint_name_to_index = {}
        self._cmd = None
        self.last_status = {
            "stamp_s": None,
            "prim_path": self.prim_path,
            "mode": None,
            "applied": False,
            "unknown_joints": [],
        }
        self.log("closed")
