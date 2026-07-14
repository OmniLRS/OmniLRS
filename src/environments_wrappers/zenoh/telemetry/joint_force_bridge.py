from __future__ import annotations

import logging
import time
from typing import Any, Dict, List

from src.environments_wrappers.zenoh.transport.factory import make_transports
from src.robots.robot import RobotManager

logger = logging.getLogger(__name__)


class JointForceBridge:
    def __init__(
        self,
        zenoh_cfg: dict,
        RM: RobotManager,
        robot_name: str = "husky",
        robot_root_prim: str = "/Robots/husky",
        publish_period_s: float = 0.1,
        logger=logger,
        init_retry_s: float = 0.5,
        not_ready_log_period_s: float = 1.0,
        keep_history: bool = False,
        history_len: int = 200,
        contact_force_threshold_n: float = 0.0,
    ):
        self.zenoh_cfg = zenoh_cfg
        self.RM = RM
        self.robot_name = robot_name
        self.robot_root_prim = robot_root_prim

        self.publish_period_s = self.zenoh_cfg.get("publish_period_s", float(publish_period_s))

        self.log = logger.info
        self.init_retry_s = self.zenoh_cfg.get("init_retry_s", float(init_retry_s))
        self.not_ready_log_period_s = self.zenoh_cfg.get("not_ready_log_period_s", float(not_ready_log_period_s))

        self.keep_history = self.zenoh_cfg.get("keep_history", bool(keep_history))
        self.history_len = self.zenoh_cfg.get("history_len", int(max(1, history_len)))

        self._inited = False
        self._t_last_publish = 0.0
        self._t_last_init_try = 0.0
        self._t_last_not_ready_log = 0.0

        self.joints_state: Dict[str, Dict[str, float]] = {}
        self.joints_history: Dict[str, List[Dict[str, float]]] = {}

        self.last_frame: Dict[str, Any] = {
            "stamp_s": None,
            "robot_root_prim": self.robot_root_prim,
            "joints": {},
            "contacts": {},
            "meta": {},
        }

        self.last_joint_states: Dict[str, Dict[str, float]] = {}

        self.transports = []
        self._transports_started = False

    def make_transports(self) -> None:
        spec = {
            "type": "zenoh",
            "keyexpr": self.zenoh_cfg.get("keyexpr", "OmniLRS/{robot_name}/joint_telemetry").format(
                robot_name=self.robot_name
            ),
            "wire_format": self.zenoh_cfg.get("wire_format", "json"),
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

            if not self._transports_started:
                for t in self.transports:
                    t.start()
                self._transports_started = True

            self._inited = True

            self.log(
                "[joint_force_bridge] initialized using Robot cached articulation API "
                f"robot={self.robot_name} prim={self.robot_root_prim}"
            )

            return True

        except Exception as e:
            self.log(f"[joint_force_bridge] init failed, retrying: {e}")
            self._inited = False
            return False

    def update(self) -> bool:
        if not self._inited:
            return False

        now = time.time()
        if (now - self._t_last_publish) < self.publish_period_s:
            return False
        self._t_last_publish = now

        robot = self.RM.robot

        if robot is None:
            return False

        if not robot.is_articulation_api_ready():
            if now - self._t_last_not_ready_log > self.not_ready_log_period_s:
                self.log("[joint_force_bridge] articulation API is not ready")
                self._t_last_not_ready_log = now
            return False

        q = robot.get_joint_positions()
        dq = robot.get_joint_velocities()
        efforts = robot.get_joint_efforts()

        joint_names = robot.get_available_joint_names()

        joints_dict: Dict[str, Dict[str, float]] = {}

        for name in joint_names:
            qi = float(q.get(name, 0.0))
            dqi = float(dq.get(name, 0.0))
            ei = float(efforts.get(name, 0.0))

            sample = {
                "stamp_s": now,
                "q": qi,
                "dq": dqi,
                "effort": ei,
            }

            self.joints_state[name] = sample

            joints_dict[name] = {
                "q": qi,
                "dq": dqi,
                "effort": ei,
            }

            if self.keep_history:
                self.joints_history.setdefault(name, [])
                self._append_hist(self.joints_history[name], sample, self.history_len)

        self.last_joint_states = joints_dict

        self.last_frame = {
            "stamp_s": now,
            "robot_root_prim": self.robot_root_prim,
            "joints": joints_dict,
            "contacts": {},
            "meta": {
                "source": "Robot cached articulation API",
                "n_joints": len(joints_dict),
                "n_contacts": 0,
                "wire_format": self.zenoh_cfg.get("wire_format", "json"),
            },
        }

        self._publish(self.last_frame)
        return True

    def _publish(self, frame: Dict[str, Any]) -> None:
        for t in self.transports:
            try:
                t.publish(frame)
            except Exception as e:
                self.log(f"[joint_force_bridge] transport publish error: {e}")

    def get_latest_frame(self) -> Dict[str, Any]:
        return self.last_frame

    def get_latest_joint_states(self) -> Dict[str, Dict[str, float]]:
        return self.last_joint_states

    def close(self) -> None:
        self._inited = False
        self.joints_state.clear()
        self.joints_history.clear()

        self.last_frame = {
            "stamp_s": None,
            "robot_root_prim": self.robot_root_prim,
            "joints": {},
            "contacts": {},
            "meta": {},
        }
        self.last_joint_states = {}

        for t in self.transports:
            try:
                t.close()
            except Exception:
                pass

        self._transports_started = False
        self.log("[joint_force_bridge] closed")

    @staticmethod
    def _append_hist(buf: List[Any], item: Any, max_len: int) -> None:
        buf.append(item)
        if len(buf) > max_len:
            del buf[: len(buf) - max_len]
