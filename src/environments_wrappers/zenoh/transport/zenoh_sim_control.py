from __future__ import annotations

__author__ = "Louis Burtz"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import asyncio
import logging
from collections import deque

import asyncio_for_robotics.zenoh as afor

logger = logging.getLogger(__name__)


class ZenohSimulationControl:
    """
    Subscribes to the simulation control keys (play, pause, stop) and queues the requests.

    The subscribers run on Kit's asyncio loop, inside an app update, where the timeline must not be driven.
    The simulation manager applies the queued requests from its main loop with `pop_requests()`.

    Args:
        keyexprs (dict[str, str]): Key expression per action, e.g. {"play": "OmniLRS/sim/play", ...}.
    """

    ACTIONS = ("play", "pause", "stop")

    def __init__(self, keyexprs: dict[str, str]) -> None:
        unknown = set(keyexprs) - set(self.ACTIONS)
        if unknown:
            raise ValueError(f"Unknown simulation control actions {sorted(unknown)}; expected {self.ACTIONS}")
        self._keyexprs = keyexprs
        self._requests: deque[str] = deque()
        self._tasks: list[asyncio.Task] = []

    def start(self) -> None:
        if self._tasks:
            return

        afor.auto_session()
        for action, keyexpr in self._keyexprs.items():
            self._tasks.append(asyncio.ensure_future(self._listen(action, keyexpr)))
            logger.info(f"[ZenohSimulationControl] listening: {keyexpr} ({action})")

    async def _listen(self, action: str, keyexpr: str) -> None:
        sub = afor.Sub(keyexpr)
        try:
            async for _ in sub.listen_reliable():
                logger.info(f"[ZenohSimulationControl] received: {action}")
                self._requests.append(action)
        finally:
            sub.close()

    def pop_requests(self) -> list[str]:
        """Return the queued actions in the order received, and clear the queue."""
        requests = []
        while self._requests:
            requests.append(self._requests.popleft())
        return requests

    def close(self) -> None:
        for task in self._tasks:
            task.cancel()
        self._tasks = []
        self._requests.clear()
