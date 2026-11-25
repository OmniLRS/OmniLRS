from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
from typing import Any, Dict

import asyncio_for_robotics.zenoh as afor
import msgspec
import numpy as np

from .wire import WireFormat, encode_payload, normalize_wire_format

logger = logging.getLogger(__name__)


class WireNDArray(msgspec.Struct, array_like=True, kw_only=True):
    dtype: str
    shape: tuple[int, ...]
    data: memoryview

    @classmethod
    def pack(cls, arr: np.ndarray) -> "WireNDArray":
        arr = np.ascontiguousarray(arr)
        return cls(data=arr.data, dtype=str(arr.dtype), shape=arr.shape)

    def unpack(self) -> np.ndarray:
        return np.frombuffer(self.data, dtype=self.dtype).reshape(self.shape)


class ZenohPubTransport:
    def __init__(
        self,
        keyexpr: str,
        wire_format: str,
        is_logging: bool,
        log_every_n: int,
    ):
        self.keyexpr = keyexpr
        self.wire_format: WireFormat = normalize_wire_format(wire_format)
        self.is_logging = is_logging
        self.log_every_n = int(max(1, log_every_n))

        self._session = None
        self._pub = None

        self._array_encoder = msgspec.msgpack.Encoder()

        self._publish_count = 0

        logger.info(
            "ZenohPubTransport created: keyexpr=%s wire_format=%s",
            self.keyexpr,
            self.wire_format,
        )

    def start(self) -> None:
        if self._session is None:
            self._session = afor.auto_session()

        if self._pub is None:
            self._pub = self._session.declare_publisher(self.keyexpr)
            logger.info("publisher declared: %s", self.keyexpr)
        else:
            logger.info("publisher already exists: %s", self.keyexpr)

    def _check_pub(self, method: str) -> bool:
        if self._pub is None:
            logger.warning(
                "%s skipped: publisher is None. Did you call start()? keyexpr=%s",
                method,
                self.keyexpr,
            )
            return False
        return True

    def _log_publish(self, kind: str, payload_len: int, extra: str = "") -> None:
        self._publish_count += 1

        if self._publish_count == 1 or self._publish_count % self.log_every_n == 0:
            logger.info(
                "published %s #%d on %s payload_bytes=%d %s",
                kind,
                self._publish_count,
                self.keyexpr,
                payload_len,
                extra,
            )

    def publish(self, frame: Dict[str, Any]) -> None:
        if not self._check_pub("publish"):
            return

        try:
            payload = encode_payload(frame, self.wire_format)
            self._pub.put(payload)
            if self.is_logging:
                self._log_publish(self.wire_format, len(payload), extra=f"keys={list(frame.keys())}")

        except Exception:
            logger.exception("failed to publish %s on %s", self.wire_format, self.keyexpr)

    def publish_array(self, array: np.ndarray) -> None:
        if not self._check_pub("publish_array"):
            return

        try:
            payload = self._array_encoder.encode(WireNDArray.pack(array))
            self._pub.put(payload)
            if self.is_logging:
                self._log_publish(
                    "ndarray",
                    len(payload),
                    extra=f"shape={array.shape} dtype={array.dtype}",
                )

        except Exception:
            logger.exception("failed to publish ndarray on %s", self.keyexpr)

    def close(self) -> None:
        logger.info("closing ZenohPubTransport: %s", self.keyexpr)

        if self._pub:
            try:
                self._pub.undeclare()
                logger.info("publisher undeclared: %s", self.keyexpr)
            except Exception:
                logger.exception("failed to undeclare publisher: %s", self.keyexpr)

        if self._session:
            try:
                self._session.close()
                logger.info("zenoh session closed")
            except Exception:
                logger.exception("failed to close zenoh session")

        self._session = None
        self._pub = None
