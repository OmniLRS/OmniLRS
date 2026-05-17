from __future__ import annotations

import json
import logging
import time
from typing import Any, AsyncGenerator, Dict, Optional

import asyncio_for_robotics.zenoh as afor
import msgspec
import numpy as np
from asyncio_for_robotics.zenoh.sub import Sub
from motion_stack.lvl1.core import JStateBatch
from ms_zenoh_bridge.utils import jsb_to_wire, wire_to_jsb

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
        json_compact: bool = True,
        log_every_n: int = 50,
    ):
        self.keyexpr = keyexpr
        self.json_compact = json_compact
        self.log_every_n = int(max(1, log_every_n))

        self._session = None
        self._pub = None
        self._sub: Optional[Sub] = None

        self._array_encoder = msgspec.msgpack.Encoder()
        self._array_decoder = msgspec.msgpack.Decoder(WireNDArray)

        self._publish_count = 0
        self._last_publish_t = 0.0

        logger.info(
            "ZenohPubTransport created: keyexpr=%s json_compact=%s",
            self.keyexpr,
            self.json_compact,
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
        return
        self._publish_count += 1
        self._last_publish_t = time.time()

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
            payload = (
                json.dumps(frame, separators=(",", ":"), ensure_ascii=False)
                if self.json_compact
                else json.dumps(frame, ensure_ascii=False)
            )
            self._pub.put(payload)
            self._log_publish("json", len(payload), extra=f"keys={list(frame.keys())}")

        except Exception:
            logger.exception("failed to publish json on %s", self.keyexpr)

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

    def publish_jstate_batch(self, batch: JStateBatch) -> None:
        """
        Publish using the exact wire format expected by ms_zenoh_bridge.
        """
        if not self._check_pub("publish_jstate_batch"):
            return

        try:
            payload = jsb_to_wire(batch.values())
            self._pub.put(payload)

            joint_names = list(batch.keys())
            preview = joint_names[:5]
            self._log_publish(
                "jstate_batch",
                len(payload),
                extra=f"n_joints={len(joint_names)} preview={preview}",
            )

        except Exception:
            logger.exception("failed to publish JStateBatch on %s", self.keyexpr)

    async def subscribe_jstate_batch(self) -> AsyncGenerator[JStateBatch, None]:
        logger.info("subscribing jstate_batch: %s", self.keyexpr)
        self._sub = Sub(self.keyexpr)

        count = 0
        try:
            async for sample in self._sub.listen_reliable():
                payload = bytes(sample.payload)
                count += 1

                if count == 1 or count % self.log_every_n == 0:
                    logger.info(
                        "received jstate payload #%d on %s bytes=%d",
                        count,
                        self.keyexpr,
                        len(payload),
                    )

                try:
                    yield wire_to_jsb(payload)
                except Exception:
                    logger.exception(
                        "failed to decode jstate payload on %s bytes=%d",
                        self.keyexpr,
                        len(payload),
                    )

        finally:
            logger.info("closing jstate subscriber: %s", self.keyexpr)
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
        logger.info("subscribing json: %s", self.keyexpr)
        self._sub = Sub(self.keyexpr)

        count = 0
        try:
            async for sample in self._sub.listen_reliable():
                payload = bytes(sample.payload)
                count += 1

                if count == 1 or count % self.log_every_n == 0:
                    logger.info(
                        "received json payload #%d on %s bytes=%d",
                        count,
                        self.keyexpr,
                        len(payload),
                    )

                yield json.loads(payload.decode("utf-8"))

        finally:
            logger.info("closing json subscriber: %s", self.keyexpr)
            self._sub.close()
            self._sub = None

    async def subscribe(self) -> AsyncGenerator[np.ndarray, None]:
        async for arr in self.subscribe_array():
            yield arr

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
