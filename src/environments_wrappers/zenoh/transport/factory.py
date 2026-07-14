from __future__ import annotations

from typing import Any, Dict, List, Optional, Union

from src.environments_wrappers.zenoh.transport.base import Transport

from .zenoh_pub import ZenohPubTransport


### TODO: should we make this transports factory to be interface for all modes in future? (i.e: zenoh, ros2, zmq, ..)
### -> if so, may also consider moving these "transport", "control", "telemetry" outside of zenoh wrapper, to be used by other modes
def make_transports(
    spec: Optional[Union[str, Dict[str, Any], List[Dict[str, Any]]]],
) -> List[Transport]:
    """
    spec examples:
      None -> []
      "zmq" -> [ZmqPubTransport()]
      {"type":"zmq","bind":"tcp://127.0.0.1:7777"} -> [...]
      [{"type":"zmq",...}, {"type":"zenoh",...}] -> [...]
    """
    if spec is None:
        return []
    if isinstance(spec, str):
        spec = [{"type": spec}]
    if isinstance(spec, dict):
        spec = [spec]

    out = []
    for s in spec:
        t = s.get("type", "").lower()
        if t == "zenoh":
            out.append(
                ZenohPubTransport(
                    keyexpr=s.get("keyexpr", "joint_telemetry"),
                    wire_format=s.get("wire_format", "json"),
                )
            )
        else:
            raise ValueError(f"Unknown transport type: {t}")
    return out
