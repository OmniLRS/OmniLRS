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
from typing import Any, Dict, List, Optional

import numpy as np
from isaacsim.core.prims import SingleArticulation

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class JointSample:
    stamp_s: float
    position: float
    velocity: float
    effort: float


@dataclass(frozen=True)
class ContactSample:
    stamp_s: float
    wrench: np.ndarray
    force_norm: float


@dataclass(frozen=True)
class ArticulationTelemetryFrame:
    stamp_s: float
    prim_path: str
    joint_names: List[str]
    joints: Dict[str, JointSample]
    contacts: Dict[str, ContactSample]
    forces_ready: bool


class ArticulationTelemetry:
    """
    OmniLRS wrapper for reading Isaac Sim articulation telemetry.

    This class owns a `SingleArticulation` view for one robot articulation and
    samples joint positions, joint velocities, measured efforts, and measured
    joint/link forces.

    Simulation managers should call `maybe_initialize()` and `sample()` from the
    simulation update loop after the world has stepped. Other code should access
    data through the cached getters such as `get_joint_positions()` and
    `get_contact_wrenches()` instead of directly reading Isaac state.

    If `keep_history` is enabled, recent joint/contact samples are stored in
    bounded buffers. `contact_force_threshold_n` filters out contact samples
    whose force norm is below the configured threshold.

    Isaac Sim articulation reference:
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/articulation_controller.html
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.core.prims/docs/index.html#isaacsim.core.prims.SingleArticulation
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "articulation_telemetry",
        init_retry_s: float = 0.5,
        sample_period_s: float = 0.0,
        contact_force_threshold_n: float = 0.0,
        keep_history: bool = False,
        history_len: int = 200,
        logger=logger,
    ):
        """
        Create an articulation telemetry reader.

        Args:
            prim_path:
                USD prim path of the articulation root to read.
            name:
                Name assigned to the Isaac `SingleArticulation` wrapper.
            init_retry_s:
                Minimum wall-clock time between initialization attempts.
            sample_period_s:
                Minimum wall-clock time between telemetry samples. A value of 0.0
                allows every call to `sample()` to read telemetry.
            contact_force_threshold_n:
                Minimum contact force norm required for a contact sample to be kept.
            keep_history:
                If True, keep bounded joint/contact sample histories.
            history_len:
                Maximum number of samples stored per history buffer.
            logger:
                Logger-like object used for status messages.
        """

        self.prim_path = prim_path
        self.name = name
        self.init_retry_s = float(init_retry_s)
        self.sample_period_s = float(sample_period_s)
        self.contact_force_threshold_n = float(max(0.0, contact_force_threshold_n))
        self.keep_history = bool(keep_history)
        self.history_len = int(max(1, history_len))

        self.log = logger.info

        self.articulation: Optional[SingleArticulation] = None

        self.joint_names: List[str] = []
        self.joint_name_to_index: Dict[str, int] = {}

        self._inited = False
        self._t_last_init_try = 0.0
        self._t_last_sample = 0.0

        self.joints_state: Dict[str, JointSample] = {}
        self.contacts_state: Dict[str, ContactSample] = {}

        self.joints_history: Dict[str, List[JointSample]] = {}
        self.contacts_history: Dict[str, List[ContactSample]] = {}

        self.last_frame = self._make_empty_frame()

    def _make_empty_frame(self) -> ArticulationTelemetryFrame:
        return ArticulationTelemetryFrame(
            stamp_s=time.time(),
            prim_path=self.prim_path,
            joint_names=[],
            joints={},
            contacts={},
            forces_ready=False,
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

            if self.keep_history:
                self.joints_history = {name: [] for name in self.joint_names}
                self.contacts_history = {}

            self._inited = True

            self.log(f"ArticulationTelemetry initialized: " f"prim={self.prim_path}, dofs={len(self.joint_names)}")
            self.log(f"DOF names: {self.joint_names}")
            return True

        except Exception as e:
            self.log(f"ArticulationTelemetry init failed, retrying in {self.init_retry_s} s. Exception was: {e}")
            self._inited = False
            self.articulation = None
            return False

    def sample(self, force: bool = False) -> Optional[ArticulationTelemetryFrame]:
        """
        Sample articulation telemetry from Isaac.

        This reads joint positions, velocities, measured efforts, and measured
        joint/link forces from the `SingleArticulation` view. The result is stored
        in `last_frame` and returned to the caller.

        Args:
            force:
                If True, ignore `sample_period_s` and sample immediately.

        Returns:
            A telemetry frame if sampling succeeded, otherwise None.
        """
        if not self.is_ready or self.articulation is None:
            return None

        now = time.time()

        if not force and self.sample_period_s > 0.0 and now - self._t_last_sample < self.sample_period_s:
            return None

        self._t_last_sample = now

        articulation = self.articulation

        q = self._safe_array(articulation.get_joint_positions())
        if q is None:
            return None  # instead of publishing all zeros

        dq = self._safe_array(articulation.get_joint_velocities())
        efforts_arr = self._safe_array(articulation.get_measured_joint_efforts())
        forces_arr = self._safe_array(articulation.get_measured_joint_forces())

        forces_ready = efforts_arr is not None and forces_arr is not None
        joints: Dict[str, JointSample] = {}

        for name, i in self.joint_name_to_index.items():
            position = float(q[i]) if q is not None and i < len(q) else 0.0
            velocity = float(dq[i]) if dq is not None and i < len(dq) else 0.0
            effort = float(efforts_arr[i]) if efforts_arr is not None and i < len(efforts_arr) else 0.0

            sample = JointSample(
                stamp_s=now,
                position=position,
                velocity=velocity,
                effort=effort,
            )

            joints[name] = sample
            self.joints_state[name] = sample

            if self.keep_history:
                self._append_history(self.joints_history[name], sample)

        contacts: Dict[str, ContactSample] = {}

        if forces_arr is not None:
            wrenches = self._reshape_wrenches(forces_arr)

            for link_i, wrench in enumerate(wrenches):
                wrench = wrench.astype(np.float32, copy=False)
                force_norm = float(np.linalg.norm(wrench[:3]))

                if force_norm < self.contact_force_threshold_n:
                    continue

                name = f"link_{link_i}"

                sample = ContactSample(
                    stamp_s=now,
                    wrench=wrench.copy(),
                    force_norm=force_norm,
                )

                contacts[name] = sample
                self.contacts_state[name] = sample

                if self.keep_history:
                    if name not in self.contacts_history:
                        self.contacts_history[name] = []
                    self._append_history(self.contacts_history[name], sample)

        frame = ArticulationTelemetryFrame(
            stamp_s=now,
            prim_path=self.prim_path,
            joint_names=list(self.joint_names),
            joints=joints,
            contacts=contacts,
            forces_ready=forces_ready,
        )

        self.last_frame = frame
        return frame

    def get_joint_positions(self) -> Dict[str, float]:
        return {name: sample.position for name, sample in self.last_frame.joints.items()}

    def get_joint_velocities(self) -> Dict[str, float]:
        return {name: sample.velocity for name, sample in self.last_frame.joints.items()}

    def get_joint_efforts(self) -> Dict[str, float]:
        return {name: sample.effort for name, sample in self.last_frame.joints.items()}

    def get_contact_wrenches(self) -> Dict[str, np.ndarray]:
        return {name: sample.wrench for name, sample in self.last_frame.contacts.items()}

    def close(self) -> None:
        self.articulation = None

        self.joint_names = []
        self.joint_name_to_index = {}

        self.joints_state.clear()
        self.contacts_state.clear()
        self.joints_history.clear()
        self.contacts_history.clear()

        self._inited = False

        self.last_frame = self._make_empty_frame()

        self.log(f"ArticulationTelemetry closed: {self.prim_path}")

    def _append_history(self, buf: List[Any], item: Any) -> None:
        buf.append(item)
        if len(buf) > self.history_len:
            del buf[: len(buf) - self.history_len]

    @staticmethod
    def _safe_array(x):
        if x is None:
            return None
        try:
            return np.asarray(x)
        except Exception:
            return None

    @staticmethod
    def _reshape_wrenches(forces_arr: np.ndarray) -> np.ndarray:
        a = np.asarray(forces_arr)

        while a.ndim > 2 and a.shape[-1] == 1:
            a = a[..., 0]

        if a.ndim == 2 and a.shape[1] == 6:
            return a

        if a.ndim == 1 and a.size % 6 == 0:
            return a.reshape((-1, 6))

        a = a.reshape((a.shape[0], -1))

        if a.shape[1] >= 6:
            return a[:, :6]

        return np.zeros((0, 6), dtype=np.float32)
