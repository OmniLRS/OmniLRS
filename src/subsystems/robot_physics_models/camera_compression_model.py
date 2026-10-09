from fractions import Fraction
from typing import Any, Dict

import av  # PyAV library for handling video encoding and decoding
import numpy as np
from PIL import Image

from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel


class CameraCompressionModel(RobotPhysicsModel):
    """H.264-compress sequential camera images and return their decoded frames.

    Uses a fixed bitrate by default; `crf` overrides it. Resolution is fixed after the first frame.
    """

    def initialize(
        self,
        fps: int = 1,  # Assumed frame arrival rate; sets timestamps and the bitrate budget, `dt` is ignored.
        bitrate: int = 20_000,  # Bits per second; low by default for strong compression.
        keyframe_interval: int = 10,  # Group of pictures size: number of frames between keyframes.
        crf: int | None = None,  # Currently implemented, but not recommended for real-time streaming, since it may introduce variable bitrate (it depends on the complexity of the content).
    ) -> None:
        # CRF: 0-51, lower means higher quality. None uses bitrate instead.
        if crf is not None and not 0 <= crf <= 51:
            raise ValueError("crf must be between 0 and 51, or None")
        if fps <= 0:
            raise ValueError("fps must be positive")
        if keyframe_interval <= 0:
            raise ValueError("keyframe_interval must be positive")
        if bitrate <= 0:
            raise ValueError("bitrate must be positive")

        self._image: Image.Image | None = None
        self._compressed_image: Image.Image | None = None
        self._resolution: tuple[int, int] | None = None
        self._fps = fps
        self._bitrate = bitrate
        self._keyframe_interval = keyframe_interval
        self._crf = crf
        self._frame_index = 0
        self._last_frame: Image.Image | None = None
        self._encoder = None
        # Persistent H.264 decoder
        self._decoder = av.CodecContext.create("h264", "r")
        self._decoder.open()

    def set_inputs(self, image: Image.Image) -> None:
        self._image = image

    def compute(self, dt: float) -> None:
        if self._image is None:
            self._compressed_image = None
            return

        if self._encoder is None:
            self._initialize_codecs(self._image.size)
        elif self._image.size != self._resolution:
            raise ValueError("Image resolution cannot change after H.264 encoding starts")

        image_rgb = np.asarray(self._image.convert("RGB"))
        # Convert NumPy RGB image to an FFmpeg frame.
        frame = av.VideoFrame.from_ndarray(image_rgb, format="rgb24")
        frame.pts = self._frame_index
        frame.time_base = self._encoder.time_base
        self._frame_index += 1

        # Compress one frame, then decode the resulting packets.
        # TODO: Handle potential packet loss or reordering in real-time streaming scenarios.
        decoded_frames = []
        for packet in self._encoder.encode(frame):
            decoded_frames.extend(self._decoder.decode(packet))

        if decoded_frames:
            decoded_rgb = decoded_frames[-1].to_ndarray(format="rgb24")
            self._compressed_image = Image.fromarray(decoded_rgb)
            self._last_frame = self._compressed_image
        elif self._last_frame is not None:
            # Application-level frame concealment.
            self._compressed_image = self._last_frame.copy()
        else:
            raise RuntimeError("H.264 encoder produced no decodable frame and none is available to conceal with")

    def _initialize_codecs(self, resolution: tuple[int, int]) -> None:
        width, height = resolution
        # yuv420p subsamples chroma 2x2, so it needs even width and height; yuv444p keeps full chroma.
        pixel_format = "yuv420p" if width % 2 == 0 and height % 2 == 0 else "yuv444p"

        # Persistent H.264 encoder
        self._encoder = av.CodecContext.create("libx264", "w")
        self._encoder.width = width
        self._encoder.height = height
        self._encoder.pix_fmt = pixel_format  # Pixel format for H.264 encoding, commonly used.
        self._encoder.time_base = Fraction(1, self._fps)
        self._encoder.framerate = Fraction(self._fps, 1)
        if self._crf is None:
            self._encoder.bit_rate = self._bitrate
        self._encoder.gop_size = self._keyframe_interval
        self._encoder.max_b_frames = 0
        self._encoder.options = {
            "preset": "veryfast",  # Encoding speed/efficiency tradeoff, slower presets give better compression.
            "tune": "zerolatency",  # Minimize latency for real-time applications.
            "x264-params": (
                f"keyint={self._keyframe_interval}:"  # Maximum interval between keyframes.
                f"min-keyint={self._keyframe_interval}:"  # Minimum interval between keyframes.
                "scenecut=0:"
                "repeat-headers=1"
            ),
        }
        if self._crf is None:
            self._encoder.options["x264-params"] += (
                f":vbv-maxrate={self._bitrate // 1000}:"
                f"vbv-bufsize={self._bitrate // 1000}"
            )
        else:
            # Quality mode: bitrate and its VBV limits are not applied.
            self._encoder.options["crf"] = str(self._crf)
        self._encoder.open()
        self._resolution = resolution

    def get_outputs(self) -> Dict[str, Any]:
        return {
            "resolution": self._resolution,
            "fps": self._fps,
            "compressed_image": self._compressed_image,
        }