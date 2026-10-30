__author__ = "Louis Burtz"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from typing import Sequence

import numpy as np
import warp as wp
from isaacsim.sensors.experimental.rtx import CameraSensor

RGBA = "rgba"
DEPTH = "distance_to_image_plane"


class RtxCamera:
    """
    Numpy front-end for an Isaac Sim RTX camera (isaacsim.sensors.experimental.rtx.CameraSensor).

    Frames are copied to host memory. While the renderer warms up, getters return an empty array.

    Args:
        prim_path (str): Path to an existing USD Camera prim (created if missing).
        resolution (Sequence[int]): Image size as (width, height).
        depth (bool): Also attach the distance-to-image-plane annotator.
    """

    def __init__(self, prim_path: str, resolution: Sequence[int], depth: bool = False) -> None:
        width, height = int(resolution[0]), int(resolution[1])
        self._shape = (height, width)
        annotators = [RGBA, DEPTH] if depth else [RGBA]
        self._sensor = CameraSensor(prim_path, resolution=self._shape, annotators=annotators)

    def _get(self, annotator: str, channels: int, dtype) -> np.ndarray:
        out = wp.empty((*self._shape, channels), dtype=dtype, device="cpu")
        data, _ = self._sensor.get_data(annotator, out=out)
        if data is None:
            return np.empty((0,), dtype=wp.dtype_to_numpy(dtype))
        return out.numpy()

    def get_rgba(self) -> np.ndarray:
        """Returns the RGBA image as a (H, W, 4) uint8 array."""
        return self._get(RGBA, 4, wp.uint8)

    def get_rgb(self) -> np.ndarray:
        """Returns the RGB image as a (H, W, 3) uint8 array."""
        rgba = self.get_rgba()
        return rgba[:, :, :3] if rgba.size else rgba

    def get_depth(self) -> np.ndarray:
        """Returns the depth (distance to image plane, in meters) as a (H, W) float32 array."""
        depth = self._get(DEPTH, 1, wp.float32)
        return depth[:, :, 0] if depth.size else depth
