__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.environments.utils import transform_orientation_into_xyz
from yamcs.client import YamcsClient, CommandHistory
import time
import math
import omni.timeline
import omni.kit.app
from PIL import Image
import numpy as np
import os
from enum import Enum

class IntervalName(Enum):
    CAMERA_STREAMING = "camera_streaming"
    POSE_OF_BASE_LINK = "pose_of_base_link"
    STOP_ROBOT = "stop_robot"

class YamcsTMTC:
    """
    YamcsTMTC class.
    It allows to control a robot instance, by receiving TCs from Yamcs and sending TM to Yamcs.
    """ 

    def __init__(
        self,
        yamcs_conf,
        robot_name,
        robot_RG,
        robot,
    ) -> None:
        self._yamcs_client = YamcsClient(yamcs_conf["address"])
        self._yamcs_processor = self._yamcs_client.get_processor(instance=yamcs_conf["instance"], processor=yamcs_conf["processor"])
        self._robot_name = robot_name
        self._robots_RG = robot_RG
        self._yamcs_conf = yamcs_conf
        self._time_of_last_command = 0
        self._robot = robot
        self._yamcs_processor.create_command_history_subscription(on_data=self._command_callback)
        self._camera_handler = CameraViewTransmitHandler(self._yamcs_processor, self._robot, yamcs_conf["address"])
        self._intervals_handler = IntervalsHandler()

    def _command_callback(self, command:CommandHistory):
        # CommandHistory info is available at: https://docs.yamcs.org/python-yamcs-client/tmtc/model/#yamcs.client.CommandHistory
        #NOTE: it happens that the subscriber receives the same instance of command multiple times in a very short period of time
        # however, desired behavior is to execute the command only once
        # since commands are executed by human operators, waiting for a small period of time (such as 0.5s) is enough to counter this issue
        if time.time() - self._time_of_last_command < 0.5:
            return
        
        self._time_of_last_command = time.time()

        name = command.name
        arguments = command.all_assignments
        print(name)
        print(arguments)
        if name == self._yamcs_conf["commands"]["drive_straight"]:
            self._drive_robot_straight(arguments["linear_velocity"], arguments["distance"]) 
        elif name == self._yamcs_conf["commands"]["drive_turn"]:
            self._drive_robot_turn(arguments["angular_velocity"], arguments["angle"])
        elif name == self._yamcs_conf["commands"]["camera_capture_high"]:
            self._camera_handler.transmit_camera_view(CameraViewTransmitHandler.BUCKET_IMAGES_ONCOMMAND, "high")
        elif name == self._yamcs_conf["commands"]["camera_streaming_on_off"]:
            self._set_activity_of_camera_streaming(arguments["action"])
        # here add reactions to other commands
        else:
            print("Unknown command:", name)

    def _drive_robot_straight(self, linear_velocity, distance):
        if linear_velocity == 0:
            self._stop_robot()
            return
        
        self._robot.drive_straight(linear_velocity)
        drive_time = distance / abs(linear_velocity)
        self._stop_robot_after_time(drive_time)

    def _drive_robot_turn(self, angular_velocity, angle):
        if angular_velocity == 0:
            self._stop_robot()
            return
        # through experiment it was observed that the robot requires ~10% more time to turn to the desired angle 
        # (maybe this number has to be set depending on a specific robot)
        turn_time_adjustment_coef = 1.11
        # similar for 2
        wheel_speed_adjustment_coef = 2
        robot_width = 2.85 * 2 # also 2 helped here
        radians = math.radians(angular_velocity) 
        wheel_speed = radians * (robot_width / 2)
        turn_time = angle / abs(angular_velocity)
        self._robot.drive_turn(wheel_speed * wheel_speed_adjustment_coef)
        self._stop_robot_after_time(turn_time * turn_time_adjustment_coef)

    def _stop_robot_after_time(self, travel_time):
        if self._intervals_handler.does_exist(IntervalName.STOP_ROBOT.value):
            self._intervals_handler.update_next_time(IntervalName.STOP_ROBOT.value, travel_time)
        else:
            self._intervals_handler.add_new_interval(name=IntervalName.STOP_ROBOT.value, seconds=travel_time, is_repeating=False, execute_immediately=False,
                                                 function=self._stop_robot)

    def _stop_robot(self):
        self._robot.stop_drive()
        self._intervals_handler.remove_interval(IntervalName.STOP_ROBOT.value)

    def start_streaming_data(self):
        self._intervals_handler.add_new_interval(name=IntervalName.POSE_OF_BASE_LINK.value, seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_pose_of_base_link)
        self._intervals_handler.add_new_interval(name=IntervalName.CAMERA_STREAMING.value, seconds=self._yamcs_conf["intervals"]["camera_streaming"], is_repeating=True, execute_immediately=True,
                                                 function=self._camera_handler.transmit_camera_view, f_args=(CameraViewTransmitHandler.BUCKET_IMAGES_STREAMING, "low"))
        # here add further intervals and their functionalities

    def _transmit_pose_of_base_link(self):
        position, orientation = self._robots_RG[str(self._robot_name)].get_pose_of_base_link()
        # euler_orient = transform_orientation_into_xyz(orientation)
        position = position.tolist()
        orientation = orientation.tolist()
        pose_of_base_link = {"position": {"x":position[0], "y":position[1], "z":position[2]}, 
                                "orientation":{"w":orientation[0],"x":orientation[1], "y":orientation[2], "z":orientation[3] }}
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["pose_of_base_link"], pose_of_base_link)

    def _set_activity_of_camera_streaming(self, action:str):
        if action == "STOP":
            self._intervals_handler.remove_interval(IntervalName.CAMERA_STREAMING.value)
        elif action == "START":
            if not self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value):
                self._intervals_handler.add_new_interval(name=IntervalName.CAMERA_STREAMING.value, seconds=self._yamcs_conf["intervals"]["camera_streaming"], is_repeating=True, execute_immediately=True,
                                                 function=self._camera_handler.transmit_camera_view, f_args=(CameraViewTransmitHandler.BUCKET_IMAGES_STREAMING, "low"))
        else:
            print("Unknown action:", action)

class IntervalsHandler:
    def __init__(self):
        self._intervals = {}
        self.timeline = omni.timeline.get_timeline_interface()
        self._update_stream = omni.kit.app.get_app().get_update_event_stream()

    def add_new_interval(self, *, name: str, seconds: int, is_repeating: bool, execute_immediately: bool, function, f_args=()):
        if name in self._intervals:
            raise ValueError(f"Interval '{name}' already exists")

        interval = {
            "seconds": seconds,
            "next_time": self.timeline.get_current_time() + seconds,
            "repeat": is_repeating,
            "execute_immediately": execute_immediately,
            "func": function,
            "args": f_args,
            "sub": None,
        }

        def callback(e, _interval=interval, _name=name):
            now = self.timeline.get_current_time()
            if now < interval["next_time"]:
                return

            _interval["func"](*_interval["args"])

            if _interval["repeat"]:
                _interval["next_time"] = now + _interval["seconds"]
            else:
                if _interval["sub"] is not None:
                    _interval["sub"].unsubscribe()
                self._intervals.pop(_name, None)

        interval["sub"] = self._update_stream.create_subscription_to_pop(
                callback,
                name= name + "_callback", 
            )

        self._intervals[name] = interval

        if interval["execute_immediately"]: # makes sense for repeating intervals, as not to have to wait for the first interval to pass before executing func
            interval["func"](*interval["args"])

    def update_next_time(self, interval_name, new_interval_time=None):
        if interval_name not in self._intervals:
            raise ValueError(f"Interval '{interval_name}' does not exist")

        interval = self._intervals[interval_name]
        now = self.timeline.get_current_time()

        if new_interval_time is None:
            interval["next_time"] = now + interval["seconds"]
        else:
            interval["next_time"] = now + new_interval_time

    def does_exist(self, interval_name):
        return interval_name in self._intervals
    
    def remove_interval(self, interval_name):
        if interval_name not in self._intervals:
            return
        
        interval = self._intervals[interval_name]

        if interval["sub"] is not None:
            interval["sub"].unsubscribe()

        self._intervals.pop(interval_name, None)

class CameraViewTransmitHandler:
    BUCKET_IMAGES_ONCOMMAND = "images_oncommand"
    BUCKET_IMAGES_STREAMING = "images_streaming"

    def __init__(self, yamcs_processor, robot, yamcs_address) -> None:
        self._yamcs_processor = yamcs_processor
        self._robot = robot
        self._yamcs_address = yamcs_address
        self._counter = {
            self.BUCKET_IMAGES_STREAMING:0,
            self.BUCKET_IMAGES_ONCOMMAND:0,
        }

    def transmit_camera_view(self, bucket:str, resolution:str):
        camera_view:Image = self._snap_camera_view_rgb(resolution)
        image_name = self._save_image_locally(camera_view, bucket)
        self._inform_yamcs(image_name, bucket)
        self._counter[bucket] += 1

    def _snap_camera_view_rgb(self, resolution:str) -> Image:
        rgba_frame = self._robot.get_rgba_camera_view(resolution)
        rgba_uint8 = rgba_frame.astype(np.uint8)
        camera_view = Image.fromarray(rgba_uint8, "RGBA")

        return camera_view
    
    def _save_image_locally(self, image, bucket) -> str:
        image_name = f"{bucket}_{self._counter[bucket]:04d}.png"
        IMG_DIR = f"/tmp/{bucket}"
        os.makedirs(IMG_DIR, exist_ok=True)   # creates directory if missing
        img_path = f"{IMG_DIR}/{image_name}" 
        image.save(img_path)

        return image_name

    def _inform_yamcs(self, image_name, bucket):
        url_storage = f"/storage/buckets/{bucket}/objects/{image_name}"
        url_full = "http://" + self._yamcs_address + f"/api{url_storage}"
        url_full_nginx = "https://52.69.177.254/yamcs" + f"/api{url_storage}"  # @TODO hardcoded nginx address for now
        self._yamcs_processor.set_parameter_values({
            f"/{bucket}/number": self._counter[bucket],
            f"/{bucket}/name": image_name,
            f"/{bucket}/url_storage": url_storage,
            f"/{bucket}/url_full": url_full,
            f"/{bucket}/url_full_nginx": url_full_nginx,
        })
