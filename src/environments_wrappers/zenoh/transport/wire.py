from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json
from typing import Any, Literal, TypeAlias, cast

import msgspec

WireFormat: TypeAlias = Literal["json", "msgspec"]


def normalize_wire_format(wire_format: str) -> WireFormat:
    normalized = wire_format.strip().lower()
    if normalized not in ("json", "msgspec"):
        raise ValueError(f"unsupported wire format {wire_format!r}; expected 'json' or 'msgspec'")
    return cast(WireFormat, normalized)


def encode_payload(value: Any, wire_format: WireFormat) -> bytes:
    if wire_format == "json":
        return json.dumps(value, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return msgspec.msgpack.encode(value)


def decode_payload(payload: bytes, wire_format: WireFormat) -> Any:
    if wire_format == "json":
        return json.loads(payload.decode("utf-8"))
    return msgspec.msgpack.decode(payload)
