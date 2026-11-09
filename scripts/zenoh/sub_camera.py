__author__ = "Bach Nguyen, Shamistan Karimov"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import asyncio

import cv2
import msgspec
from asyncio_for_robotics.zenoh.sub import Sub

from environments_wrappers.zenoh.transport.zenoh_pub import WireNDArray


async def main(keyexpr: str):
    sub = Sub(keyexpr)

    try:
        async for sample in sub.listen_reliable():
            data = bytes(sample.payload)
            im = msgspec.msgpack.decode(data, type=WireNDArray).unpack()
            im = cv2.resize(im, (0, 0), fx=0.5, fy=0.5)

            cv2.imshow("", im)
            cv2.waitKey(1)
    finally:
        sub.close()


if __name__ == "__main__":
    # Each rgba camera of the robot yaml is published per resolution: OmniLRS/<robot>/camera/<camera>/<resolution>
    parser = argparse.ArgumentParser(description="View a robot camera published over Zenoh.")
    parser.add_argument("--robot", default="husky")
    parser.add_argument("--camera", default="cam_color", help="name of an rgba camera in cfg/robot/<robot>.yaml")
    parser.add_argument("--resolution", default="high")
    args = parser.parse_args()

    asyncio.run(main(f"OmniLRS/{args.robot}/camera/{args.camera}/{args.resolution}"))
