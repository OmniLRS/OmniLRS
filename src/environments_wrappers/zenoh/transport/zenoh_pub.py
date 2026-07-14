from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
import time
from typing import Any, AsyncGenerator, Dict, Optional

import asyncio_for_robotics.zenoh as afor
import msgspec
import numpy as np
from asyncio_for_robotics.zenoh.sub import Sub

from .wire import WireFormat, decode_payload, encode_payload, normalize_wire_format

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
        keyexpr: str = "OmniLRS/Husky/**",
        wire_format: str = "json",
        log_every_n: int = 50,
    ):
        self.keyexpr = keyexpr
        self.wire_format: WireFormat = normalize_wire_format(wire_format)
        self.log_every_n = int(max(1, log_every_n))

        self._session = None
        self._pub = None
        self._sub: Optional[Sub] = None

        self._array_encoder = msgspec.msgpack.Encoder()
        self._array_decoder = msgspec.msgpack.Decoder(WireNDArray)

        self._publish_count = 0
        self._last_publish_t = 0.0

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
        pass

    def publish(self, frame: Dict[str, Any]) -> None:
        if not self._check_pub("publish"):
            return

        try:
            payload = encode_payload(frame, self.wire_format)
            self._pub.put(payload)
            self._log_publish(
                self.wire_format, len(payload), extra=f"keys={list(frame.keys())}"
            )

        except Exception:
            logger.exception(
                "failed to publish %s on %s", self.wire_format, self.keyexpr
            )

    def publish_array(self, array: np.ndarray) -> None:
        if not self._check_pub("publish_array"):
            return

        try:
            payload = self._array_encoder.encode(WireNDArray.pack(array))
            self._pub.put(payload)
            self._log_publish(
                "ndarray",
                len(payload),
                extra=f"shape={array.shape} dtype={array.dtype}",
            )

        except Exception:
            logger.exception("failed to publish ndarray on %s", self.keyexpr)

    async def subscribe_payload(self) -> AsyncGenerator[Any, None]:
        logger.info("subscribing %s: %s", self.wire_format, self.keyexpr)
        self._sub = Sub(self.keyexpr)

        count = 0
        try:
            async for sample in self._sub.listen_reliable():
                payload = bytes(sample.payload)
                count += 1

                if count == 1 or count % self.log_every_n == 0:
                    logger.info(
                        "received %s payload #%d on %s bytes=%d",
                        self.wire_format,
                        count,
                        self.keyexpr,
                        len(payload),
                    )

                try:
                    yield decode_payload(payload, self.wire_format)
                except Exception:
                    logger.exception(
                        "failed to decode %s payload on %s bytes=%d",
                        self.wire_format,
                        self.keyexpr,
                        len(payload),
                    )

        finally:
            logger.info("closing subscriber: %s", self.keyexpr)
            self._sub.close()
            self._sub = None

    async def subscribe_array(self) -> AsyncGenerator[np.ndarray, None]:
        logger.info("subscribing ndarray: %s", self.keyexpr)
        self._sub = Sub(self.keyexpr)

        count = 0
        try:
            async for sample in self._sub.listen_reliable():
                payload = bytes(sample.payload)
                count += 1

                if count == 1 or count % self.log_every_n == 0:
                    logger.info(
                        "received ndarray payload #%d on %s bytes=%d",
                        count,
                        self.keyexpr,
                        len(payload),
                    )

                wire_obj = self._array_decoder.decode(payload)
                yield wire_obj.unpack()

        finally:
            logger.info("closing ndarray subscriber: %s", self.keyexpr)
            self._sub.close()
            self._sub = None

    async def subscribe_json(self) -> AsyncGenerator[Dict[str, Any], None]:
        if self.wire_format != "json":
            raise ValueError("subscribe_json() requires wire_format='json'")
        async for value in self.subscribe_payload():
            yield value

    async def subscribe(self) -> AsyncGenerator[Any, None]:
        async for value in self.subscribe_payload():
            yield value

    def close(self) -> None:
        logger.info("closing ZenohPubTransport: %s", self.keyexpr)

        if self._pub:
            try:
                self._pub.undeclare()
                logger.info("publisher undeclared: %s", self.keyexpr)
            except Exception:
                logger.exception("failed to undeclare publisher: %s", self.keyexpr)

        if self._sub:
            try:
                self._sub.close()
                logger.info("subscriber closed: %s", self.keyexpr)
            except Exception:
                logger.exception("failed to close subscriber: %s", self.keyexpr)

        if self._session:
            try:
                self._session.close()
                logger.info("zenoh session closed")
            except Exception:
                logger.exception("failed to close zenoh session")

        self._session = None
        self._pub = None
        self._sub = None
