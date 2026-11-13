__author__ = "Louis Burtz"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


import math
import os
import traceback

import cv2
import numpy as np
import omni.timeline
import omni.usd
from isaacsim.sensors.experimental.rtx import CameraSensor
from pxr import Gf, UsdGeom

from src.tmtc.commands_handler import RECEIVED_COMMANDS

ENV_VAR = "OMNILRS_ROBOT_VIDEO"


class RobotVideoRecorder:
    """
    Opt-in video of the robot, to follow a test run: a top view and a side view that follow base_link.

    Set the environment variable OMNILRS_ROBOT_VIDEO to the path of an .mp4 file to record it (the Yamcs sanity tests do).
    The top view looks down from TOP_HEIGHT_M above base_link, with north (+y) up in the image. The side view looks at the
    left side of the robot from SIDE_DISTANCE_M, and turns with the robot. Each frame shows the simulation time and the
    base_link pose. The video is a fragmented MP4, so it stays readable if the simulation stops without closing it.

    At the bottom left, each frame shows the commanded and measured speed of each wheel, one column per wheel. At the top
    center, each frame shows the test sequence: the current test step (the last line of the steps file
    <video name>.steps.log, which the test writes) and the last COMMAND_LINES Yamcs commands with the simulation time at
    which the simulation received them. The newest command is yellow for NEW_COMMAND_S.
    """

    WIDTH, HEIGHT = 480, 270  # pixels of each view; the video is two views side by side
    PERIOD_S = 0.2  # simulation time between frames
    FPS = 10  # the video plays at 2 times the simulation time
    TOP_HEIGHT_M = 3.5
    SIDE_DISTANCE_M = 3.0
    SIDE_HEIGHT_M = 0.8
    ROOT = "/RobotVideo"
    COMMAND_LINES = 3
    WHEEL_MISMATCH = 0.2  # rad/s between the commanded and the measured speed of a wheel
    NEW_COMMAND_S = 3.0

    @classmethod
    def from_env(cls, robot_rigid_group, robot=None) -> "RobotVideoRecorder | None":
        path = os.environ.get(ENV_VAR)
        return cls(path, robot_rigid_group, robot) if path else None

    def __init__(self, path: str, robot_rigid_group, robot=None) -> None:
        import av  # only when recording

        self._path = path
        self._robot = robot_rigid_group
        self._robot_joints = robot  # Robot, for the wheel speeds (optional)
        self._timeline = omni.timeline.get_timeline_interface()
        stage = omni.usd.get_context().get_stage()
        self._cameras = {}
        for name in ("top", "side"):
            camera = UsdGeom.Camera.Define(stage, f"{self.ROOT}/{name}")
            camera.CreateFocalLengthAttr(18.0)
            camera.CreateHorizontalApertureAttr(20.955)
            camera.CreateVerticalApertureAttr(20.955 * self.HEIGHT / self.WIDTH)
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
            sensor = CameraSensor(str(camera.GetPath()), resolution=(self.HEIGHT, self.WIDTH), annotators=["rgba"])
            # CameraSensor replaces the xform ops of the prim by translate, orient and scale ops: use these.
            ops = {op.GetOpName(): op for op in UsdGeom.Xformable(camera).GetOrderedXformOps()}
            self._cameras[name] = ((ops["xformOp:translate"], ops["xformOp:orient"]), sensor)
        self._container = av.open(path, "w", options={"movflags": "frag_keyframe+empty_moov", "flush_packets": "1"})
        codec = "libx264" if "libx264" in av.codecs_available else "mpeg4"
        # A keyframe (and an MP4 fragment) each second of video and no frame held back by the encoder: a simulation
        # that stops without close() loses at most 1 s of video.
        options = {"g": str(self.FPS), "tune": "zerolatency"} if codec == "libx264" else {"g": str(self.FPS)}
        self._stream = self._container.add_stream(codec, rate=self.FPS, options=options)
        self._stream.width, self._stream.height, self._stream.pix_fmt = 2 * self.WIDTH, self.HEIGHT, "yuv420p"
        self._next_time = 0.0
        self._frames = 0
        self._failed = False
        self._steps_path = os.path.splitext(path)[0] + ".steps.log"
        self._step = ""
        self._steps_mtime = None
        self._last_command = max((number for number, _, _ in list(RECEIVED_COMMANDS)), default=0)
        self._commands = []  # (simulation time of receipt, text), oldest first
        print(f"[RobotVideoRecorder] Recording {path} ({codec})")

    def update(self) -> None:
        """
        Moves the cameras with the robot; every PERIOD_S of simulation time, adds a frame to the video. A failure stops
        the recording (with one message), never the simulation.
        """
        if self._failed:
            return
        try:
            self._update()
        except Exception:
            self._failed = True
            print("[RobotVideoRecorder] Recording stopped after an error:")
            traceback.print_exc()

    def _update(self) -> None:
        position, orientation = self._robot.get_pose_of_base_link()
        x, y, z = (float(v) for v in position)
        w, qx, qy, qz = (float(v) for v in orientation)
        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        target = Gf.Vec3d(x, y, z)
        self._place("top", target + Gf.Vec3d(0, 0, self.TOP_HEIGHT_M), target, Gf.Vec3d(0, 1, 0))
        left = Gf.Vec3d(-math.sin(yaw), math.cos(yaw), 0)
        side_eye = target + left * self.SIDE_DISTANCE_M + Gf.Vec3d(0, 0, self.SIDE_HEIGHT_M)
        self._place("side", side_eye, target, Gf.Vec3d(0, 0, 1))

        now = self._timeline.get_current_time()
        self._read_commands(now)
        if now < self._next_time - 2 * self.PERIOD_S:  # timeline restarted
            self._next_time = now
        if now < self._next_time:
            return
        views = []
        for name, (_, sensor) in self._cameras.items():
            rgba, _ = sensor.get_data("rgba")
            if rgba is None:
                return  # the cameras render from the next frame on
            view = np.ascontiguousarray(rgba.numpy()[..., :3])
            cv2.putText(view, name, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)
            views.append(view)
        frame = np.concatenate(views, axis=1)
        text = f"t={now:7.1f} s  x={x:7.2f} y={y:7.2f} z={z:6.2f} m  yaw={math.degrees(yaw):7.1f} deg"
        cv2.putText(frame, text, (8, self.HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
        self._draw_wheels(frame)
        self._draw_sequence(frame, now)
        self._encode(frame)
        self._next_time = now + self.PERIOD_S

    def _wheel_speeds(self):
        """(wheel labels, commanded speeds, measured speeds) in rad/s, left wheels then right wheels, or None."""
        robot = self._robot_joints
        if robot is None or not robot.control.is_ready:
            return None
        articulation = robot.control.articulation
        left, right = robot.get_wheel_joint_names("left"), robot.get_wheel_joint_names("right")
        if not left + right:
            return None
        indices = articulation.get_dof_indices(left + right)
        targets = np.asarray(articulation.get_dof_velocity_targets(dof_indices=indices).numpy()).reshape(-1)
        speeds = np.asarray(articulation.get_dof_velocities(dof_indices=indices).numpy()).reshape(-1)
        labels = [name.split("_")[-1] for name in left + right]  # wheel_fl -> fl
        return labels, targets, speeds, len(left)

    def _draw_wheels(self, frame: np.ndarray) -> None:
        """
        Table at the bottom left: one column per wheel (left wheels, then right wheels), the commanded speed on one row
        and the measured speed right below it. A measured speed more than WHEEL_MISMATCH away from its command is orange.
        """
        data = self._wheel_speeds()
        if data is None:
            return
        labels, targets, speeds, n_left = data
        font, scale, row = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 15
        label_width, column = 78, 50  # pixels
        top = self.HEIGHT - 26 - 3 * row
        right_edge = 8 + label_width + column * len(labels) + (12 if n_left else 0) + 44
        box = frame[top - 4 : top + 3 * row + 2, 0:right_edge]
        box[:] = (box * 0.4).astype(np.uint8)

        def text_right(text, x_right, y, color):
            width = cv2.getTextSize(text, font, scale, 1)[0][0]
            cv2.putText(frame, text, (x_right - width, y), font, scale, color, 1, cv2.LINE_AA)

        rows = [("rad/s", labels, None), ("command", targets, None), ("measured", speeds, targets)]
        for r, (name, values, reference) in enumerate(rows):
            y = top + row * (r + 1) - 3
            cv2.putText(frame, name, (8, y), font, scale, (200, 200, 200), 1, cv2.LINE_AA)
            for i, value in enumerate(values):
                x_right = 8 + label_width + column * (i + 1) + (12 if i >= n_left else 0)
                if r == 0:
                    text_right(value, x_right, y, (200, 200, 200))
                    continue
                mismatch = reference is not None and abs(value - reference[i]) > self.WHEEL_MISMATCH
                color = (255, 140, 0) if mismatch else (0, 255, 255)
                text_right(f"{value:.2f}", x_right, y, color)
            if n_left:
                separator_x = 8 + label_width + column * n_left + 9
                cv2.putText(frame, "|", (separator_x, y), font, scale, (200, 200, 200), 1, cv2.LINE_AA)

    def _read_commands(self, now: float) -> None:
        """Stamps the commands received since the last call with the simulation time `now`."""
        for number, name, arguments in list(RECEIVED_COMMANDS):
            if number <= self._last_command:
                continue
            self._last_command = number
            short = name.split("/", 2)[-1] if name.count("/") >= 2 else name  # /Rover/motor/x -> motor/x
            values = " ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in arguments.items())
            self._commands = (self._commands + [(now, f"{short} {values}".strip())])[-self.COMMAND_LINES :]

    def _draw_sequence(self, frame: np.ndarray, now: float) -> None:
        """Draws the current test step and the last commands at the top center of the frame."""
        try:
            mtime = os.path.getmtime(self._steps_path)
            if mtime != self._steps_mtime:
                self._steps_mtime = mtime
                with open(self._steps_path) as steps:
                    lines = [line.strip() for line in steps if line.strip()]
                self._step = lines[-1] if lines else ""
        except OSError:
            pass
        lines = [(self._step, (255, 255, 255))] if self._step else []
        for i, (received, text) in enumerate(self._commands):
            newest = i == len(self._commands) - 1 and now - received < self.NEW_COMMAND_S
            lines.append((f"t={received:6.1f} s  {text}", (255, 230, 0) if newest else (170, 170, 170)))
        if not lines:
            return
        font, scale, line_height = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 17
        width = max(cv2.getTextSize(text, font, scale, 1)[0][0] for text, _ in lines) + 16
        left = max(0, (frame.shape[1] - width) // 2)
        bottom = 6 + line_height * len(lines)
        box = frame[2:bottom, left : left + width]
        box[:] = (box * 0.35).astype(np.uint8)  # darkened background for readability
        for i, (text, color) in enumerate(lines):
            cv2.putText(frame, text, (left + 8, 2 + line_height * (i + 1)), font, scale, color, 1, cv2.LINE_AA)

    def close(self) -> None:
        for packet in self._stream.encode(None):
            self._container.mux(packet)
        self._container.close()
        print(f"[RobotVideoRecorder] Wrote {self._frames} frames to {self._path}")

    def _place(self, name: str, eye: Gf.Vec3d, target: Gf.Vec3d, up: Gf.Vec3d) -> None:
        (translate_op, orient_op), _ = self._cameras[name]
        pose = Gf.Transform(Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse())
        translate_op.Set(Gf.Vec3d(pose.GetTranslation()))
        quat = pose.GetRotation().GetQuat()
        orient_op.Set(quat if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf(quat))

    def _encode(self, frame: np.ndarray) -> None:
        import av

        for packet in self._stream.encode(av.VideoFrame.from_ndarray(frame, format="rgb24")):
            self._container.mux(packet)
        self._frames += 1
