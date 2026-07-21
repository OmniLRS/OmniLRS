import asyncio

import cv2
import msgspec
from asyncio_for_robotics.zenoh.sub import Sub

from environments_wrappers.zenoh.transport.zenoh_pub import WireNDArray


async def main():
    sub = Sub("OmniLRS/husky/camera/cam_color")

    try:
        async for sample in sub.listen_reliable():
            data = bytes(sample.payload)
            im = msgspec.msgpack.decode(data, type=WireNDArray).unpack()
            im = cv2.resize(im, (0, 0), fx=0.5, fy=0.5)

            cv2.imshow("", im)
            cv2.waitKey(1)
    finally:
        sub.close()


asyncio.run(main())
