import asyncio
from asyncio_for_robotics.zenoh.sub import Sub
import numpy as np
import cv2
import msgspec


class WireNDArray(msgspec.Struct, array_like=True, kw_only=True):
    dtype: str
    shape: tuple[int, ...]
    data: memoryview

    @classmethod
    def pack(cls, arr: np.ndarray):
        return cls(data=arr.data, dtype=str(arr.dtype), shape=arr.shape)
    
    def unpack(self) -> np.ndarray:
        return np.frombuffer(self.data, dtype=self.dtype).reshape(self.shape)

async def main():
    sub = Sub("OmniLRS/husky/camera/cam_color")

    try:
        async for sample in sub.listen_reliable():
            data = bytes(sample.payload)
            im = msgspec.msgpack.decode(data, type=WireNDArray).unpack()
            #breakpoint()
            #if im is not None:
            im = cv2.resize(im, (0,0), fx=0.5, fy=0.5)
            
            cv2.imshow("", im); 
            cv2.waitKey(1)
    finally:
        sub.close()

asyncio.run(main())
