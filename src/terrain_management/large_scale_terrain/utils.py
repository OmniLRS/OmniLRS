__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from scipy.interpolate import CubicSpline
from typing import Tuple
import dataclasses
import numpy as np
import threading
import logging
import time
import zfpy

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")


@dataclasses.dataclass
class BoundingBox:
    x_min: float = 0
    x_max: float = 0
    y_min: float = 0
    y_max: float = 0

    def get_area(self) -> float:
        return (self.x_max - self.x_min) * (self.y_max - self.y_min)


@dataclasses.dataclass
class RockBlockData:
    coordinates: np.ndarray = dataclasses.field(default_factory=np.ndarray)
    quaternion: np.ndarray = dataclasses.field(default_factory=np.ndarray)
    scale: np.ndarray = dataclasses.field(default_factory=np.ndarray)
    ids: np.ndarray = dataclasses.field(default_factory=np.ndarray)

    def __sizeof__(self) -> int:
        return self.coordinates.nbytes + self.quaternion.nbytes + self.scale.nbytes + self.ids.nbytes

    def compress(self):
        coordinates = zfpy.compress_numpy(self.coordinates, tolerance=1e-3)
        quaternion = zfpy.compress_numpy(self.quaternion, tolerance=1e-3)
        scale = zfpy.compress_numpy(self.scale, tolerance=1e-3)
        ids = zfpy.compress_numpy(self.ids)
        return CompressedRockBlockData(
            coordinates=coordinates,
            quaternion=quaternion,
            scale=scale,
            ids=ids,
        )


@dataclasses.dataclass
class CompressedRockBlockData:
    coordinates: bytes = dataclasses.field(default_factory=bytes)
    quaternion: bytes = dataclasses.field(default_factory=bytes)
    scale: bytes = dataclasses.field(default_factory=bytes)
    ids: bytes = dataclasses.field(default_factory=bytes)

    def decompress(self) -> RockBlockData:
        coordinates = zfpy.decompress_numpy(self.coordinates)
        quaternion = zfpy.decompress_numpy(self.quaternion)
        scale = zfpy.decompress_numpy(self.scale)
        ids = zfpy.decompress_numpy(self.ids)
        return RockBlockData(
            coordinates=coordinates,
            quaternion=quaternion,
            scale=scale,
            ids=ids,
        )

    def __sizeof__(self) -> int:
        return (
            self.coordinates.__sizeof__()
            + self.quaternion.__sizeof__()
            + self.scale.__sizeof__()
            + self.ids.__sizeof__()
        )


# TODO (antoine.richard): Add a memory footprint method to the CraterMetadata class
# TODO (antoine.richard): Find a way to compress a list of CraterMetadata objects.


@dataclasses.dataclass
class CraterMetadata:
    radius: float = 0.0
    coordinates: Tuple[int, int] = (0, 0)
    deformation_spline_id: CubicSpline = None
    marks_spline_id: CubicSpline = None
    marks_intensity: float = 0
    crater_profile_id: int = 0
    xy_deformation_factor: Tuple[float, float] = (0, 0)
    rotation: float = 0

    def get_memory_footprint(self) -> int:
        return -1


class ScopedTimer:
    _thread_local_data = threading.local()

    def __init__(self, name, active=True, argb_color=None, unit="s"):
        self.name = name
        self.active = active
        self.argb_color = argb_color
        assert unit in ["s", "ms", "us"]
        self.unit_multiplier = {"s": 1, "ms": 1e3, "us": 1e6}[unit]
        self.unit = unit

        if argb_color:
            self.rgb_color = self.argb_to_rgb(argb_color)
            self.ansi_color = self.rgb_to_ansi(self.rgb_color)
        else:
            self.ansi_color = ""
        self.indent = 2  # Number of spaces to indent per nesting level

    def argb_to_rgb(self, argb):
        rgb = (argb >> 16) & 0xFFFFFF
        return (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF

    def rgb_to_ansi(self, rgb):
        return f"\033[38;2;{rgb[0]};{rgb[1]};{rgb[2]}m"

    def __enter__(self):
        if self.active:
            if not hasattr(self._thread_local_data, "nesting_level"):
                self._thread_local_data.nesting_level = 0
            if not hasattr(self._thread_local_data, "messages"):
                self._thread_local_data.messages = []

            self._thread_local_data.nesting_level += 1
            self.start_time = time.time()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.active:
            self.end_time = time.time()
            elapsed_time = (self.end_time - self.start_time) * self.unit_multiplier
            reset_color = "\033[0m"
            indentation = " " * (self._thread_local_data.nesting_level - 1) * self.indent
            message = f"{self.ansi_color}{indentation}{self.name} took: {elapsed_time:.4f} {self.unit}{reset_color}"

            # Insert the message at the beginning of the list to ensure the outermost message is printed first
            self._thread_local_data.messages.insert(0, message)

            self._thread_local_data.nesting_level -= 1

            # If we are back to the outermost level, print all accumulated messages
            if self._thread_local_data.nesting_level == 0:
                for msg in self._thread_local_data.messages:
                    logger.info(msg)
                # Clear the message stack
                self._thread_local_data.messages.clear()
