__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import os
import csv
import json
import time
import numpy as np
from typing import Callable, Dict, List, Optional, Tuple


class SLAMRecorder:
    """
    Utility class for recording SLAM-appropriate data from OmniLRS simulations.

    Handles synchronized data capture from multiple sensors, timestamp management,
    ground truth pose recording, and data serialization in standard formats.

    Attributes:
        output_dir (str): Root directory for recorded data.
        sensors (dict): Registered sensor callbacks keyed by sensor name.
        ground_truth_poses (list): Accumulated ground truth poses.
        sensor_data (dict): Accumulated sensor data keyed by sensor name.
        timestamps (list): Accumulated simulation timestamps.
        _frame_index (int): Current frame counter.
        _recording (bool): Whether recording is active.
    """

    def __init__(self, output_dir: str, sensors: Optional[Dict[str, Callable]] = None) -> None:
        """
        Initializes the SLAMRecorder.

        Args:
            output_dir (str): Path to the directory where data will be saved.
            sensors (dict, optional): Dictionary mapping sensor names to data-fetching callables.
        """
        self.output_dir = output_dir
        self.sensors: Dict[str, Callable] = sensors if sensors is not None else {}
        self.ground_truth_poses: List[Dict] = []
        self.sensor_data: Dict[str, List] = {name: [] for name in self.sensors}
        self.timestamps: List[float] = []
        self._frame_index: int = 0
        self._recording: bool = False
        self._start_time: Optional[float] = None

    def register_sensor(self, name: str, callback: Callable) -> None:
        """
        Registers a sensor with the recorder.

        Args:
            name (str): Unique sensor identifier.
            callback (Callable): Function that returns the sensor's current data.
        """
        if name in self.sensors:
            raise ValueError(f"Sensor '{name}' is already registered.")
        self.sensors[name] = callback
        self.sensor_data[name] = []

    def unregister_sensor(self, name: str) -> None:
        """
        Unregisters a previously registered sensor.

        Args:
            name (str): Sensor identifier to remove.
        """
        if name not in self.sensors:
            raise KeyError(f"Sensor '{name}' is not registered.")
        del self.sensors[name]
        del self.sensor_data[name]

    def start(self) -> None:
        """
        Begins a recording session.
        """
        if self._recording:
            raise RuntimeError("Recording is already in progress.")
        self._recording = True
        self._start_time = time.time()
        self.ground_truth_poses = []
        self.timestamps = []
        self.sensor_data = {name: [] for name in self.sensors}
        self._frame_index = 0

    def stop(self) -> None:
        """
        Ends the current recording session.
        """
        if not self._recording:
            raise RuntimeError("No recording is in progress.")
        self._recording = False

    def record_frame(
        self,
        pose: np.ndarray,
        timestamp: Optional[float] = None,
    ) -> None:
        """
        Records a single synchronized frame from all registered sensors and the ground truth pose.

        Args:
            pose (np.ndarray): 4x4 homogeneous transformation matrix representing the ground truth pose.
            timestamp (float, optional): Simulation timestamp in seconds. Defaults to wall-clock time.
        """
        if not self._recording:
            raise RuntimeError("Cannot record frame: recording has not been started.")
        if pose.shape != (4, 4):
            raise ValueError(f"Expected pose shape (4, 4), got {pose.shape}.")

        ts = timestamp if timestamp is not None else time.time() - self._start_time
        self.timestamps.append(ts)

        self.ground_truth_poses.append(
            {
                "frame": self._frame_index,
                "timestamp": ts,
                "pose": pose.tolist(),
            }
        )

        for name, callback in self.sensors.items():
            data = callback()
            self.sensor_data[name].append(
                {
                    "frame": self._frame_index,
                    "timestamp": ts,
                    "data": data,
                }
            )

        self._frame_index += 1

    def save(self) -> None:
        """
        Serializes all recorded data to disk under output_dir.

        Directory layout::

            output_dir/
                ground_truth.csv
                ground_truth.json
                sensors/
                    <sensor_name>/
                        frames.json
        """
        if self._recording:
            raise RuntimeError("Stop recording before saving.")
        os.makedirs(self.output_dir, exist_ok=True)
        self._save_ground_truth()
        self._save_sensor_data()

    def _save_ground_truth(self) -> None:
        csv_path = os.path.join(self.output_dir, "ground_truth.csv")
        json_path = os.path.join(self.output_dir, "ground_truth.json")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["frame", "timestamp", "tx", "ty", "tz", "r00", "r01", "r02", "r10", "r11", "r12", "r20", "r21", "r22"])
            for entry in self.ground_truth_poses:
                pose = np.array(entry["pose"])
                tx, ty, tz = pose[0, 3], pose[1, 3], pose[2, 3]
                r = pose[:3, :3]
                writer.writerow(
                    [
                        entry["frame"],
                        entry["timestamp"],
                        tx, ty, tz,
                        r[0, 0], r[0, 1], r[0, 2],
                        r[1, 0], r[1, 1], r[1, 2],
                        r[2, 0], r[2, 1], r[2, 2],
                    ]
                )

        with open(json_path, "w") as f:
            json.dump(self.ground_truth_poses, f, indent=2)

    def _save_sensor_data(self) -> None:
        sensors_dir = os.path.join(self.output_dir, "sensors")
        for name, frames in self.sensor_data.items():
            sensor_dir = os.path.join(sensors_dir, name)
            os.makedirs(sensor_dir, exist_ok=True)
            frames_path = os.path.join(sensor_dir, "frames.json")
            serializable_frames = []
            for frame in frames:
                entry = {
                    "frame": frame["frame"],
                    "timestamp": frame["timestamp"],
                }
                data = frame["data"]
                if isinstance(data, np.ndarray):
                    entry["data"] = data.tolist()
                else:
                    entry["data"] = data
                serializable_frames.append(entry)
            with open(frames_path, "w") as f:
                json.dump(serializable_frames, f, indent=2)

    def get_ground_truth_poses(self) -> List[Dict]:
        """
        Returns the accumulated ground truth poses.

        Returns:
            list: List of pose dictionaries with keys 'frame', 'timestamp', 'pose'.
        """
        return list(self.ground_truth_poses)

    def get_timestamps(self) -> List[float]:
        """
        Returns the accumulated timestamps.

        Returns:
            list: List of float timestamps.
        """
        return list(self.timestamps)

    def get_sensor_frames(self, name: str) -> List[Dict]:
        """
        Returns the accumulated frames for a given sensor.

        Args:
            name (str): Sensor identifier.

        Returns:
            list: List of frame dictionaries.
        """
        if name not in self.sensor_data:
            raise KeyError(f"Sensor '{name}' is not registered.")
        return list(self.sensor_data[name])

    @property
    def frame_count(self) -> int:
        """
        Returns the number of recorded frames.
        """
        return self._frame_index

    @property
    def is_recording(self) -> bool:
        """
        Returns whether a recording session is active.
        """
        return self._recording
