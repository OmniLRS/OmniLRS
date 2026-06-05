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
        # if : == "zmq":
        #     out.append(
        #         ZmqPubTransport(
        #             bind=s.get("bind", "tcp://127.0.0.1:7777"),
        #             topic=s.get("topic", ""),
        #             json_compact=s.get("json_compact", True),
        #         )
        #     )
        if t == "zenoh":
            out.append(
                ZenohPubTransport(
                    keyexpr=s.get("keyexpr", "joint_telemetry"),
                )
            )
        # elif t == "ros_js":
        #     out.append(
        #         RosJointStatePubTransport(
        #             topic=s.get("topic", "/joint_states"),
        #             frame_id=s.get("frame_id", ""),
        #         )
        #     )
        # elif t == "ros_imu":
        #     out.append(
        #         RosImuPubTransport(
        #             topic=s.get("topic", "/imu"),
        #             frame_id=s.get("frame_id", ""),
        #         )
        #     )

        else:
            raise ValueError(f"Unknown transport type: {t}")
    return out
