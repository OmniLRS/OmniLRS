__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.environments.utils import transform_orientation_into_xyz
from yamcs.client import YamcsClient, CommandHistory
import threading
import time
import math
import omni.timeline
import omni.kit.app
from PIL import Image
import numpy as np
import os

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
        self.timeline = omni.timeline.get_timeline_interface()
        self._update_stream = omni.kit.app.get_app().get_update_event_stream()
        self._drive_callback_sub = None
        self._stop_time = None
        self._yamcs_processor.create_command_history_subscription(on_data=self._command_callback)
        self._camera_handler = CameraViewTransmitHandler(self._yamcs_processor, self._robot, yamcs_conf["address"])

    def _command_callback(self, command:CommandHistory):
        # CommandHistory info is available at: https://docs.yamcs.org/python-yamcs-client/tmtc/model/#yamcs.client.CommandHistory
        #NOTE: it happens that the subscriber receives the same instance of command multiple times in a very short period of time
        # however, desired behaviour is to execute the command only once
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
            self._turn_rover(arguments["angular_velocity"], arguments["angle"])
        # here add reactions to other commands
        else:
            print("Unknown comand.")

    def _drive_robot_straight(self, linear_velocity, distance):
        if linear_velocity == 0:
            self._stop_robot()
            return
        
        self._robot.drive_straight(linear_velocity)
        drive_time = distance / abs(linear_velocity)
        self._stop_rover_after_time(drive_time)

    def _turn_rover(self, angular_velocity, angle):
        if angular_velocity == 0:
            self._stop_robot()
            return
        # through experiment it was observed that the robot requries ~10% more time to turn to the desired angle 
        # (maybe this number has to be set depending on a specific robot)
        turn_time_adjustment_coef = 1.11
        # similar for 2
        wheel_speed_adjustment_coef = 2
        robot_width = 2.85 * 2 # also 2 helped here
        radians = math.radians(angular_velocity) 
        wheel_speed = radians * (robot_width / 2)
        turn_time = angle / abs(angular_velocity)
        self._robot.drive_turn(wheel_speed * wheel_speed_adjustment_coef)
        self._stop_rover_after_time(turn_time * turn_time_adjustment_coef)

    def _stop_rover_after_time(self, travel_time):
        start_time = self.timeline.get_current_time()
        self._stop_time = start_time + travel_time
        
        if self._drive_callback_sub is None:
            # the callback subscription gets overriten which enables continuous control over the robot
            # (issuing new command, before the previous one is completed with no issues)
            self._drive_callback_sub = self._update_stream.create_subscription_to_pop(
                self._stop_robot_callback,
                name="RobotDriveStopCallback", 
            )

    def _stop_robot_callback(self, e):
        # callback function for stopping the robot
        # checks every frame if simulaiton has reached the self._stop_time (calculated in the caller function)
        if self._stop_time is None:
            return

        current_time = self.timeline.get_current_time()

        if current_time >= self._stop_time:
            self._stop_robot()

    def _stop_robot(self):
        self._robot.stop_drive()
        self._stop_time = None

        if self._drive_callback_sub is not None:
            self._drive_callback_sub.unsubscribe()
            self._drive_callback_sub = None

    def start(self):
        # initially inteded to be in a for robot in robots loop, thus to have one thread for each robot
        # however, for the workshop use-case, the code was simplified to assume use of only one robot 
        t = threading.Thread(
            target=self._yamcs_transmitter,
            args=(self._robot_name, self._yamcs_conf["interval_s"]),
            name="yamcs-TMTC-" + self._robot_name,
            daemon=True,
        )   
        t.start()

    def _yamcs_transmitter(self, robot_name, interval_s):
        print("started TMTC for: " + robot_name)
        try:
            while True:
                self._transmit_base_link_pose()
                self._camera_handler.transmit_camera_view("images_streaming")
                # add here further commands
                time.sleep(interval_s) #TODO: change into simulation secs
        finally:
            print("ended transmitter for: " + robot_name)

    def _transmit_base_link_pose(self):
        position, orientation = self._robots_RG[str(self._robot_name)].get_base_link_pose()
        # euler_orient = transform_orientation_into_xyz(orientation)
        position = position.tolist()
        orientation = orientation.tolist()
        pose_of_base_link = {"position": {"x":position[0], "y":position[1], "z":position[2]}, 
                                "orientation":{"w":orientation[0],"x":orientation[1], "y":orientation[2], "z":orientation[3] }}
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["pose_of_base_link"], pose_of_base_link)

class CameraViewTransmitHandler:
    def __init__(self, yamcs_processor, robot, yamcs_address) -> None:
        self._yamcs_processor = yamcs_processor
        self._robot = robot
        self._yamcs_address = yamcs_address
        self._counter = {
            "images_streaming":0,
            "images_commanding":0,
        }

    def transmit_camera_view(self, bucket:str):
        camera_view:Image = self._snap_camera_view_rgb()
        image_name = self._save_image_locally(camera_view, bucket)
        self._inform_yamcs(image_name, bucket)
        self._counter[bucket] += 1

    def _snap_camera_view_rgb(self) -> Image:
        rgba_frame = self._robot.get_rgba_camera_view()
        rgba_uint8 = rgba_frame.astype(np.uint8)
        camera_view = Image.fromarray(rgba_uint8, "RGBA")

        return camera_view
    
    def _save_image_locally(self, image, bucket) -> str:
        n = self._counter[bucket]
        image_name = f"{bucket}_{n:04d}.png"
        IMG_DIR = f"/tmp/{bucket}"
        os.makedirs(IMG_DIR, exist_ok=True)   # creates directory if missing
        img_path = f"{IMG_DIR}/{image_name}" 
        image.save(img_path)

        return image_name

    def _inform_yamcs(self, image_name, bucket):
        url_storage = f"/storage/buckets/{bucket}/objects/{image_name}"
        url_full = "http://" + self._yamcs_address + f"/api{url_storage}"
        self._yamcs_processor.set_parameter_values({
            f"/{bucket}/number": self._counter[bucket],
            f"/{bucket}/name": image_name,
            f"/{bucket}/url_storage": url_storage,
            f"/{bucket}/url_full": url_full,
        })