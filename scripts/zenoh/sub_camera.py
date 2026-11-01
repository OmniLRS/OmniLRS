__author__ = "Bach Nguyen, Shamistan Karimov"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import asyncio

import cv2
import msgspec
from asyncio_for_robotics.zenoh.sub import Sub

from environments_wrappers.zenoh.transport.zenoh_pub import WireNDArray


async def main(robot_name: str, camera_name: str):
    sub = Sub(f"OmniLRS/{robot_name}/camera/{camera_name}/high")

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
    parser = argparse.ArgumentParser(description="View camera stream over Zenoh.")
    parser.add_argument("robot_name", nargs="?", default="husky")
    parser.add_argument(
        "camera_name",
        nargs="?",
        default="cam_color",
    )
    args = parser.parse_args()

    asyncio.run(main(args.robot_name, args.camera_name))
