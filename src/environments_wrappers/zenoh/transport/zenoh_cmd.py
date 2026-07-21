from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import asyncio
import logging
from collections.abc import Mapping
from typing import Any

import asyncio_for_robotics.zenoh as afor

from src.robots.robot import RobotManager

from .wire import WireFormat, decode_payload, normalize_wire_format

logger = logging.getLogger(__name__)


class ZenohCommandReceiver:
    def __init__(
        self,
        RM: RobotManager,
        keyexpr: str = "joint_cmd",
        wire_format: str = "json",
        logger=logger,
        log_every_n: int = 5000,
    ):
        self.RM = RM
        self.keyexpr = keyexpr
        self.wire_format: WireFormat = normalize_wire_format(wire_format)
        self.log = logger.info
        self.log_every_n = int(max(1, log_every_n))

        self._task: asyncio.Task | None = None
        self._started = False
        self._count = 0

    def start(self) -> None:
        if self._started:
            return

        afor.auto_session()
        self._task = asyncio.ensure_future(self._listen())
        self._started = True

        self.log(f"[ZenohCommandReceiver] listening: {self.keyexpr} wire_format={self.wire_format}")

    @staticmethod
    def _joint_targets(
        message: Any,
    ) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
        if not isinstance(message, Mapping):
            raise TypeError("command payload must be a mapping")

        joints = message.get("joints", message)
        if not isinstance(joints, Mapping):
            raise TypeError("command payload 'joints' must be a mapping")

        position_targets: dict[str, float] = {}
        velocity_targets: dict[str, float] = {}
        effort_targets: dict[str, float] = {}

        for name, state in joints.items():
            if not isinstance(name, str) or not isinstance(state, Mapping):
                raise TypeError("each joint entry must map a string name to a mapping")

            position = state.get("position", state.get("q"))
            velocity = state.get("velocity", state.get("dq"))
            effort = state.get("effort")

            if position is not None:
                position_targets[name] = float(position)
            if velocity is not None:
                velocity_targets[name] = float(velocity)
            if effort is not None:
                effort_targets[name] = float(effort)

        return position_targets, velocity_targets, effort_targets

    async def _listen(self) -> None:
        sub = afor.Sub(self.keyexpr)

        try:
            async for msg in sub.listen_reliable():
                payload = bytes(msg.payload)

                try:
                    command = decode_payload(payload, self.wire_format)
                    position_targets, velocity_targets, effort_targets = self._joint_targets(command)
                except Exception as e:
                    self.log(f"[ZenohCommandReceiver] failed to decode {self.wire_format} command: {e}")
                    continue

                robot = self.RM.robot

                if robot is None:
                    self.log("[ZenohCommandReceiver] skipped command: robot is None")
                    continue

                if not robot.is_articulation_api_ready():
                    self.log("[ZenohCommandReceiver] skipped command: articulation API is not ready")
                    continue

                ok = robot.set_joint_targets(
                    position_targets=position_targets,
                    velocity_targets=velocity_targets,
                    effort_targets=effort_targets,
                )

                self._count += 1

                if self._count == 1 or self._count % self.log_every_n == 0:
                    self.log(
                        "[ZenohCommandReceiver] received cmd "
                        f"#{self._count}: "
                        f"ok={ok} "
                        f"position={len(position_targets)} "
                        f"velocity={len(velocity_targets)} "
                        f"effort={len(effort_targets)} "
                        f"preview_pos={list(position_targets.items())[:3]} "
                        f"preview_vel={list(velocity_targets.items())[:3]} "
                        f"preview_eff={list(effort_targets.items())[:3]}"
                    )

        except asyncio.CancelledError:
            raise

        except Exception as e:
            self.log(f"[ZenohCommandReceiver] listener crashed: {e}")

        finally:
            try:
                sub.close()
            except Exception:
                pass

    def close(self) -> None:
        if self._task is not None:
            self._task.cancel()

        self._task = None
        self._started = False
