from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import get_current_stage
from motion_stack.lvl1.core import JStateBatch
from motion_stack.utils.joint_state import JState
from motion_stack.utils.time import Time

from src.environments_wrappers.zenoh.transport.factory import make_transports

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class JointSample:
    stamp_s: float
    q: float
    dq: float
    effort: Optional[float] = None


@dataclass(frozen=True)
class ContactSample:
    stamp_s: float
    wrench: np.ndarray


class JointForceBridge:
    def __init__(
        self,
        zenoh_cfg: dict,
        robot_name: str = "husky",
        robot_root_prim: str = "/Robots/husky/joints/*",
        publish_period_s: float = 0.1,
        logger=logger,
        init_retry_s: float = 0.5,
        not_ready_log_period_s: float = 1.0,
        keep_history: bool = False,
        history_len: int = 200,
        contact_force_threshold_n: float = 0.0,
        publish_json_frame: bool = False,
    ):
        self.zenoh_cfg = zenoh_cfg
        self.robot_name = robot_name
        self.robot_root_prim = robot_root_prim
        self.publish_period_s = self.zenoh_cfg.get("publish_period_s", float(publish_period_s))
        self.log = logger.info
        self.init_retry_s = self.zenoh_cfg.get("init_retry_s", float(init_retry_s))
        self.not_ready_log_period_s = self.zenoh_cfg.get("not_ready_log_period_s", float(not_ready_log_period_s))

        self.keep_history = self.zenoh_cfg.get("keep_history", bool(keep_history))
        self.history_len = self.zenoh_cfg.get("history_len", int(max(1, history_len)))
        self.contact_force_threshold_n = self.zenoh_cfg.get("contact_force_threshold_n", float(max(0.0, contact_force_threshold_n))) 
        self.publish_json_frame = self.zenoh_cfg.get("publish_json_frame", bool(publish_json_frame))

        self.rover: Optional[SingleArticulation] = None
        self.stage = None

        self.joint_names: List[str] = []
        self.joint_name_to_index: Dict[str, int] = {}

        self._inited = False
        self._forces_ready = False
        self._t_last_publish = 0.0
        self._t_last_init_try = 0.0
        self._t_last_not_ready_log = 0.0

        self.joints_state: Dict[str, JointSample] = {}
        self.contacts: Dict[str, ContactSample] = {}

        self.joints_history: Dict[str, List[JointSample]] = {}
        self.contacts_history: Dict[str, List[ContactSample]] = {}

        self.last_frame: Dict[str, Any] = {
            "stamp_s": None,
            "robot_root_prim": self.robot_root_prim,
            "joints": {},
            "contacts": {},
            "meta": {},
        }

        self.last_jstate_batch: JStateBatch = JStateBatch({})

        self.transports = []
        self._transports_started = False

    def make_transports(self):
        spec = {
            "type": "zenoh",
            "keyexpr": self.zenoh_cfg.get("keyexpr", "OmniLRS/{robot_name}/joint_telemetry").format(robot_name=self.robot_name)
        }
        self.transports = make_transports([spec])

    def maybe_initialize(self) -> bool:
        if self._inited:
            return True

        now = time.time()
        if now - self._t_last_init_try < self.init_retry_s:
            return False
        self._t_last_init_try = now

        try:
            self.make_transports()

            self.stage = get_current_stage()
            self.rover = SingleArticulation(
                prim_path=self.robot_root_prim,
                name="joint_force_probe",
            )
            self.rover.initialize()

            q0 = self._safe_array(self.rover.get_joint_positions())
            n_dofs = 0 if q0 is None else len(q0)

            dof_names = self.rover.dof_names
            if dof_names is not None and len(dof_names) == n_dofs:
                self.joint_names = list(dof_names)

            self.joint_name_to_index = {n: i for i, n in enumerate(self.joint_names)}

            if self.keep_history:
                self.joints_history = {n: [] for n in self.joint_names}
                self.contacts_history = {}

            self._inited = True
            self._forces_ready = False

            if not self._transports_started:
                for t in self.transports:
                    t.start()
                self._transports_started = True

            self.log(
                f"initialized: dofs={len(self.joint_names)} prim={self.robot_root_prim}"
            )
            self.log(f"dof names: {self.joint_names}")
            return True

        except Exception as e:
            self.log(f"init failed (retrying): {e}")
            self._inited = False
            self.rover = None
            return False

    def update(self) -> bool:
        if not self._inited or self.rover is None:
            return False

        now = time.time()
        if (now - self._t_last_publish) < self.publish_period_s:
            return False
        self._t_last_publish = now

        stamp = Time.from_parts(nano=time.time_ns())

        q = self._safe_array(self.rover.get_joint_positions())
        dq = self._safe_array(self.rover.get_joint_velocities())

        efforts = self.rover.get_measured_joint_efforts()
        efforts_arr = self._safe_array(efforts) if efforts is not None else None

        forces = self.rover.get_measured_joint_forces()
        forces_arr = self._safe_array(forces) if forces is not None else None

        forces_ready = (efforts_arr is not None) and (forces_arr is not None)
        self._forces_ready = forces_ready

        joints_dict: Dict[str, Dict[str, float]] = {}
        jstate_dict: Dict[str, JState] = {}

        for name, i in self.joint_name_to_index.items():
            qi = float(q[i]) if q is not None and i < len(q) else 0.0
            dqi = float(dq[i]) if dq is not None and i < len(dq) else 0.0
            ei = (
                float(efforts_arr[i])
                if efforts_arr is not None and i < len(efforts_arr)
                else 0.0
            )

            sample = JointSample(stamp_s=now, q=qi, dq=dqi, effort=ei)
            self.joints_state[name] = sample

            joints_dict[name] = {
                "q": qi,
                "dq": dqi,
                "effort": ei,
            }

            jstate_dict[name] = JState(
                name=name,
                time=stamp,
                position=qi,
                velocity=dqi,
                effort=ei,
            )

            if self.keep_history:
                self._append_hist(self.joints_history[name], sample, self.history_len)

        jsb = JStateBatch(jstate_dict)
        self.last_jstate_batch = jsb

        contacts_dict: Dict[str, Dict[str, Any]] = {}

        if forces_arr is None:
            if now - self._t_last_not_ready_log > self.not_ready_log_period_s:
                self.log(
                    f"forces/efforts not ready. "
                    f"q={None if q is None else q.shape} "
                    f"dq={None if dq is None else dq.shape}"
                )
                self._t_last_not_ready_log = now
        else:
            wrenches = self._reshape_wrenches(forces_arr)

            for link_i, wrench in enumerate(wrenches):
                wrench = wrench.astype(np.float32, copy=False)
                f_norm = float(np.linalg.norm(wrench[0:3]))

                if f_norm < self.contact_force_threshold_n:
                    continue

                key = f"link_{link_i}"
                cs = ContactSample(stamp_s=now, wrench=wrench.copy())
                self.contacts[key] = cs

                contacts_dict[key] = {
                    "wrench": wrench.tolist(),
                    "force_norm": f_norm,
                }

                if self.keep_history:
                    if key not in self.contacts_history:
                        self.contacts_history[key] = []
                    self._append_hist(self.contacts_history[key], cs, self.history_len)

        self.last_frame = {
            "stamp_s": now,
            "robot_root_prim": self.robot_root_prim,
            "joints": joints_dict,
            "contacts": contacts_dict,
            "meta": {
                "forces_ready": forces_ready,
                "n_joints": len(self.joint_names),
                "n_contacts": len(contacts_dict),
                "wire_format": "WireJStateBatch",
            },
        }

        self._publish(jsb, self.last_frame)
        return True

    def _publish(self, jsb: JStateBatch, frame: Dict[str, Any]) -> None:
        for t in self.transports:
            try:
                if hasattr(t, "publish_jstate_batch"):
                    t.publish_jstate_batch(jsb)
                elif self.publish_json_frame:
                    t.publish(frame)
                else:
                    self.log(
                        f"transport {type(t).__name__} has no publish_jstate_batch(); skipping"
                    )
            except Exception as e:
                self.log(f"transport publish error: {e}")

    def get_latest_frame(self) -> Dict[str, Any]:
        return self.last_frame

    def get_latest_jstate_batch(self) -> JStateBatch:
        return self.last_jstate_batch

    def close(self) -> None:
        self._inited = False
        self._forces_ready = False
        self.rover = None
        self.stage = None
        self.joint_names = []
        self.joint_name_to_index = {}
        self.joints_state.clear()
        self.contacts.clear()
        self.joints_history.clear()
        self.contacts_history.clear()

        self.last_frame = {
            "stamp_s": None,
            "robot_root_prim": self.robot_root_prim,
            "joints": {},
            "contacts": {},
            "meta": {},
        }
        self.last_jstate_batch = JStateBatch({})

        for t in self.transports:
            try:
                t.close()
            except Exception:
                pass

        self._transports_started = False
        self.log("closed")

    @staticmethod
    def _safe_array(x):
        if x is None:
            return None
        try:
            return np.asarray(x)
        except Exception:
            return None

    @staticmethod
    def _append_hist(buf: List[Any], item: Any, max_len: int) -> None:
        buf.append(item)
        if len(buf) > max_len:
            del buf[: (len(buf) - max_len)]

    @staticmethod
    def _reshape_wrenches(forces_arr: np.ndarray) -> np.ndarray:
        a = np.asarray(forces_arr)
        while a.ndim > 2 and a.shape[-1] == 1:
            a = a[..., 0]
        if a.ndim == 2 and a.shape[1] == 6:
            return a
        if a.ndim == 1 and (a.size % 6 == 0):
            return a.reshape((-1, 6))
        a = a.reshape((a.shape[0], -1))
        if a.shape[1] >= 6:
            return a[:, :6]
        return np.zeros((0, 6), dtype=np.float32)
