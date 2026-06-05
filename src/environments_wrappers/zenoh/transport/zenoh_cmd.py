from __future__ import annotations

import asyncio
import logging
import time

import asyncio_for_robotics.zenoh as afor
from ms_zenoh_bridge.utils import wire_to_jsb

from src.environments_wrappers.zenoh.control.articulation_controller import (
    ArticulationController,
    JointCommand,
)

logger = logging.getLogger(__name__)


class ZenohCommandReceiver:
    def __init__(
        self,
        controller: ArticulationController,
        keyexpr: str = "joint_cmd",
        logger=logger,
        log_every_n: int = 5000,
    ):
        self.controller = controller
        self.keyexpr = keyexpr
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

        self.log(f"[ZenohCommandReceiver] listening: {self.keyexpr}")

    async def _listen(self) -> None:
        sub = afor.Sub(self.keyexpr)

        try:
            async for msg in sub.listen_reliable():
                payload = bytes(msg.payload)

                try:
                    jsb = wire_to_jsb(payload)
                except Exception as e:
                    self.log(
                        f"[ZenohCommandReceiver] failed to decode WireJStateBatch: {e}"
                    )
                    continue

                position_targets: dict[str, float] = {}
                velocity_targets: dict[str, float] = {}

                for name, j in jsb.items():
                    if j.position is not None:
                        position_targets[name] = float(j.position)

                    if j.velocity is not None:
                        velocity_targets[name] = float(j.velocity)

                cmd = JointCommand(
                    stamp_s=time.time(),
                    mode="mixed",
                    position_targets=position_targets,
                    velocity_targets=velocity_targets,
                )

                self.controller.set_command(cmd)

                self._count += 1
                if self._count == 1 or self._count % self.log_every_n == 0:
                    self.log(
                        "[ZenohCommandReceiver] received cmd "
                        f"#{self._count}: "
                        f"position={len(position_targets)} "
                        f"velocity={len(velocity_targets)} "
                        f"preview_pos={list(position_targets.items())[:3]} "
                        f"preview_vel={list(velocity_targets.items())[:3]}"
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
