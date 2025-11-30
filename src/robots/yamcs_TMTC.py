__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.environments.utils import transform_orientation_into_xyz
from src.robots.subsystems_manager import GoNogoState, ObcState, PowerState, SolarPanelState
from yamcs.client import YamcsClient, CommandHistory
import time
import math
import omni.timeline
import omni.kit.app
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import os
from enum import Enum
from scipy.spatial.transform import Rotation as R
from pathlib import Path


class IntervalName(Enum):
    # use for intervals that are repeatedly created or removed
    CAMERA_STREAMING = "camera_streaming"
    STOP_ROBOT = "stop_robot"
    CAMERA_STREAMING_STATE = "camera_streaming_state"
    OBC_STATE = "obc_state"

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
        self._helper = HandlerHelper(self._yamcs_processor, yamcs_conf["address"])
        self._camera_handler = CameraViewTransmitHandler(self._yamcs_processor, self._robot, yamcs_conf["address"], self._helper)
        self._intervals_handler = IntervalsHandler()
        self._payload_handler = PayloadHandler(self._helper)

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
            self.handle_high_res_capture()
        elif name == self._yamcs_conf["commands"]["camera_streaming_on_off"]:
            self._set_activity_of_camera_streaming(arguments["action"])
        elif name == self._yamcs_conf["commands"]["camera_capture_depth"]:
            self.handle_depth_capture()
        elif name == self._yamcs_conf["commands"]["power_electronics"]:
            self._handle_electronics(arguments["subsystem_id"], arguments["power_state"])
        elif name == self._yamcs_conf["commands"]["solar_panel"]:
            self._handle_solar_panel(arguments["deployment"])
        elif name == self._yamcs_conf["commands"]["go_nogo"]:
            self._handle_go_nogo(arguments["decision"])
        elif name == self._yamcs_conf["commands"]["capture_apxs"]:
            self._payload_handler._snap_apxs()
        elif name == self._yamcs_conf["commands"]["admin_batter_percentage"]:
            self._handle_batter_perc_change(arguments["battery_percentage"])
        # here add reactions to other commands
        else:
            print("Unknown command:", name)

    def _handle_batter_perc_change(self, battery_percentage:int):
        self._robot.subsystems.set_battery_perc(battery_percentage)

    def handle_high_res_capture(self):
        self._camera_handler.transmit_camera_view(CameraViewTransmitHandler.BUCKET_IMAGES_ONCOMMAND, "high", "rgb")
        self._set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self):
        self._camera_handler.transmit_camera_view(CameraViewTransmitHandler.BUCKET_IMAGES_DEPTH, "high", "depth")
        self._set_obc_state(ObcState.CAMERA, 10)

    def _set_obc_state(self, state:ObcState, set_to_idle_after=0):
        if self._intervals_handler.does_exist(IntervalName.OBC_STATE.value):
            self._intervals_handler.remove_interval(IntervalName.OBC_STATE.value)

        self._robot.subsystems.set_obc_state(state)

        if set_to_idle_after != 0:
            self._intervals_handler.add_new_interval(name=IntervalName.OBC_STATE.value, seconds=set_to_idle_after, is_repeating=False, execute_immediately=False,
                                                 function=self._robot.subsystems.set_obc_state, f_args=[ObcState.IDLE])

    def _handle_solar_panel(self, new_state:SolarPanelState):
        if new_state == SolarPanelState.STOWED:
            self._robot.subsystems.stow_solar()
        elif new_state == SolarPanelState.DEPLOYED:
            self._robot.subsystems.deploy_solar()
        else:
            print("New state for solar panel is unknown:", new_state)

    def _handle_electronics(self, electronics:str, new_state:PowerState):
        if new_state not in [PowerState.ON.value, PowerState.OFF.value]:
            print("New decision for PowerState of electronics is unknown:", new_state)
            return
        
        new_state = PowerState[new_state]
        self._robot.subsystems.set_electronics_state(electronics, new_state)

    def _handle_go_nogo(self, decision:str):
        if decision not in [GoNogoState.GO.name, GoNogoState.NOGO.name]:
            print("New decision for GO / NOGO is unknown:", decision)
            return
        
        decision = GoNogoState[decision]
        self._robot.subsystems.set_go_nogo_state(decision)

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
        self._set_obc_state(ObcState.MOTOR, travel_time)
        if self._intervals_handler.does_exist(IntervalName.STOP_ROBOT.value):
            self._intervals_handler.update_next_time(IntervalName.STOP_ROBOT.value, travel_time)
        else:
            self._intervals_handler.add_new_interval(name=IntervalName.STOP_ROBOT.value, seconds=travel_time, is_repeating=False, execute_immediately=False,
                                                 function=self._stop_robot)

    def _stop_robot(self):
        self._robot.stop_drive()
        self._set_obc_state(ObcState.IDLE)
        self._intervals_handler.remove_interval(IntervalName.STOP_ROBOT.value)

    def start_streaming_data(self):
        self._intervals_handler.add_new_interval(name="Pose of base link", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_pose_of_base_link)
        self._intervals_handler.add_new_interval(name=IntervalName.CAMERA_STREAMING.value, seconds=self._yamcs_conf["intervals"]["camera_streaming"], is_repeating=True, execute_immediately=True,
                                                 function=self._camera_handler.transmit_camera_view, f_args=(CameraViewTransmitHandler.BUCKET_IMAGES_STREAMING, "low"))
        self._is_camera_streaming_on = True
        # TODO add this when parameter path fixed
        # self._intervals_handler.add_new_interval(name=IntervalName.CAMERA_STREAMING_STATE.value, seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
        #                                          function=self._transmit_camera_streaming_state)
        self._intervals_handler.add_new_interval(name="GO_NOGO", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_go_nogo)
        self._intervals_handler.add_new_interval(name="IMU readings", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_imu_readings)
        self._intervals_handler.add_new_interval(name="OBC state", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_obc_state)
        self._intervals_handler.add_new_interval(name="Radio rssi", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_radio_signal_info)
        self._intervals_handler.add_new_interval(name="Thermal info", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_thermal_info, f_args=[self._yamcs_conf["intervals"]["robot_stats"]])
        self._intervals_handler.add_new_interval(name="Power status", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_power_info, f_args=[self._yamcs_conf["intervals"]["robot_stats"]])
        self._intervals_handler.add_new_interval(name="Neutron count", seconds=self._yamcs_conf["intervals"]["robot_stats"], is_repeating=True, execute_immediately=True,
                                                 function=self._transmit_neutroun_count, f_args=[self._yamcs_conf["intervals"]["robot_stats"]])
        # here add further intervals and their functionalities

    def _transmit_radio_signal_info(self):
        robot_position, orientation = self._robots_RG[str(self._robot_name)].get_pose_of_base_link()
        rssi = self._robot.subsystems.calculate_rssi(robot_position)
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["rssi"], int(rssi))

    def _transmit_thermal_info(self, interval_s):
        robot_position, orientation = self._robots_RG[str(self._robot_name)].get_pose_of_base_link()
        temperatures = self._robot.subsystems.calculate_temperature(robot_position, interval_s)

        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_front"], temperatures['+X'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_back"], temperatures['-X'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_left"], temperatures['+Y'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_right"], temperatures['-Y'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_top"], temperatures['+Z'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_bottom"], temperatures['-Z'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["temperature_elec_box"], temperatures['interior'])

    def _transmit_power_info(self, interval_s):
        robot_position, orientation = self._robots_RG[str(self._robot_name)].get_pose_of_base_link()
        obc_state = self._robot.subsystems.get_obc_state()
        power_status = self._robot.subsystems.calculate_power_status(robot_position, interval_s, obc_state)
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["battery_charge"], int(power_status['battery_percentage_measured']))
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["battery_voltage"], power_status['battery_voltage_measured'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["total_current_in"], power_status['solar_input_current_measured'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_obc"], power_status["device_currents_measured"]['current_draw_obc'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_motor_controller"], power_status["device_currents_measured"]['current_draw_motor_controller'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_neutron_spectrometer"], power_status["device_currents_measured"]['current_draw_neutron_spectrometer'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_apxs"], power_status["device_currents_measured"]['current_draw_apxs'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_camera"], power_status["device_currents_measured"]['current_draw_camera'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_radio"], power_status["device_currents_measured"]['current_draw_radio'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["current_draw_eps"], power_status["device_currents_measured"]['current_draw_eps'])
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["motor_current"], power_status['motor_currents_measured'])

    def _transmit_neutroun_count(self, interval_s):
        neutron_counts = self._robot.subsystems.get_neutron_count(interval_s)
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["neutron_counts"], neutron_counts)

    def _transmit_obc_state(self):
        obc_state = self._robot.subsystems.get_obc_state()
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["obc_state"], obc_state.value)

    def _transmit_imu_readings(self):
        imu_accelerometer, imu_gyroscope, orientation = self._robot.get_imu_readings()
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["imu_accelerometer"], imu_accelerometer)
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["imu_gyroscope"], imu_gyroscope)
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["imu_orientation"], orientation)
        
    def _transmit_camera_streaming_state(self):
        #TODO update this depending on the param format
        is_camera_streaming = self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value)
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["camera_streaming_state"], is_camera_streaming)

    def _transmit_go_nogo(self):
        go_nogo_state =  self._robot.subsystems.get_go_nogo_state().value
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["go_nogo"], go_nogo_state)

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
    BUCKET_IMAGES_STREAMING = "images_streaming"
    BUCKET_IMAGES_ONCOMMAND = "images_oncommand"
    BUCKET_IMAGES_DEPTH = "images_depth"

    def __init__(self, yamcs_processor, robot, yamcs_address, helper) -> None:
        self._yamcs_processor = yamcs_processor
        self._robot = robot
        self._yamcs_address = yamcs_address
        self._counter = {
            self.BUCKET_IMAGES_STREAMING:0,
            self.BUCKET_IMAGES_ONCOMMAND:0,
            self.BUCKET_IMAGES_DEPTH:0,
        }
        self._helper = helper

    def transmit_camera_view(self, bucket:str, resolution:str, type:str="rgb"):
        camera_view:Image = None

        if type == "depth":
            camera_view:Image = self._snap_camera_view_depth(resolution)
        elif type == "rgb":
            camera_view:Image = self._snap_camera_view_rgb(resolution)
        else:
            print("in transmit_camera_view: unknown type:", type)
            return

        image_name = self._helper.save_image_locally(camera_view, bucket, self._counter[bucket])
        print(image_name)
        self._helper.inform_yamcs(image_name, "camera", bucket, self._counter[bucket])
        self._counter[bucket] += 1

    def _snap_camera_view_rgb(self, resolution:str) -> Image:
        frame = self._robot.get_rgba_camera_view(resolution)
        frame_uint8 = frame.astype(np.uint8)
        camera_view = Image.fromarray(frame_uint8, "RGBA")

        return camera_view
    
    def _snap_camera_view_depth(self, resolution:str) -> Image:
        frame = self._robot.get_depth_camera_view(resolution)
        #NOTE this gives non-human readable png, while the below uncommented code transfers this into grayscale png
        # camera_view = Image.fromarray(frame, "L")
        # return camera_view
    
        depth = np.nan_to_num(frame, nan=0.0, posinf=0.0, neginf=0.0)

        valid = depth > 0
        if not np.any(valid):
            return Image.fromarray(np.zeros_like(depth, dtype=np.uint8), "L")

        near = np.percentile(depth[valid], 1)
        far  = np.percentile(depth[valid], 20)

        d = np.clip(depth, near, far)
        d = (d - near) / (far - near + 1e-8)
        d = (1.0 - d) * 255.0 
        d_uint8 = d.astype(np.uint8)

        return Image.fromarray(d_uint8, mode="L")
    
class HandlerHelper:
    URL_FULL_NGINX = "https://52.69.177.254/yamcs"

    def __init__(self, yamcs_processor, yamcs_address):
        self._yamcs_processor = yamcs_processor
        self._yamcs_address = yamcs_address

    def save_image_locally(self, image, bucket, counter_number) -> str:
        image_name = f"{bucket}_{counter_number:04d}.png"
        IMG_DIR = f"/tmp/{bucket}"
        os.makedirs(IMG_DIR, exist_ok=True)   # creates directory if missing
        img_path = f"{IMG_DIR}/{image_name}" 
        image.save(img_path)

        return image_name

    def inform_yamcs(self, image_name, path_prefix, bucket, counter_number):
        url_storage = f"/storage/buckets/{bucket}/objects/{image_name}"
        url_full = "http://" + self._yamcs_address + f"/api{url_storage}"
        url_full_nginx = self.URL_FULL_NGINX + f"/api{url_storage}"  # @TODO hardcoded nginx address for now
        self._yamcs_processor.set_parameter_values({
            f"/Rover/{path_prefix}/{bucket}/number": counter_number,
            f"/Rover/{path_prefix}/{bucket}/name": image_name,
            f"/Rover/{path_prefix}/{bucket}/url_storage": url_storage,
            f"/Rover/{path_prefix}/{bucket}/url_full": url_full,
            f"/Rover/{path_prefix}/{bucket}/url_full_nginx": url_full_nginx,
        })

class PayloadHandler:

    BUCKET_IMAGES_APXS = "images_apxs"
    APXS_WIDTH = 1440
    APXS_HEIGHT = 1080
    APXS_SAMPLES_DIR = "/workspace/omnilrs/assets/images/"
    APXS_FONT = ImageFont.load_default(APXS_HEIGHT//15) 

    def __init__(self, helper:HandlerHelper):
        self._helper = helper
        self._counter = {
            self.BUCKET_IMAGES_APXS:0,
        }

    def _snap_apxs(self):
        image_name = f"{self.BUCKET_IMAGES_APXS}_{self._counter[self.BUCKET_IMAGES_APXS]}.png"

        if self._counter[self.BUCKET_IMAGES_APXS] == 0:
            apxs_background_path = os.path.join(self.APXS_SAMPLES_DIR, 'APXS_nodata.png')
        else:
            apxs_background_path = os.path.join(self.APXS_SAMPLES_DIR, 'APXS_measurement.png')

        # if os.path.exists(apxs_background_path):
        #     print("File exists:", apxs_background_path)
        # else:
        #     print("FILE NOT FOUND:", apxs_background_path)

        print(apxs_background_path)
        img = Image.open(apxs_background_path) #.convert('RGB').resize((self.APXS_WIDTH, self.APXS_HEIGHT))
        # draw = ImageDraw.Draw(img)
        # self._draw_text(draw, text=image_name, fill="black", position='top-right')
        image_name = self._helper.save_image_locally(img, self.BUCKET_IMAGES_APXS, self._counter[self.BUCKET_IMAGES_APXS])
        print("image name", image_name)

        saved_image = f"/tmp/{self.BUCKET_IMAGES_APXS}/{image_name}"
        if os.path.exists(saved_image):
            print("File exists:", saved_image)
        else:
            print("FILE NOT FOUND:", saved_image)
            
        self._helper.inform_yamcs(image_name, "payload", self.BUCKET_IMAGES_APXS, self._counter[self.BUCKET_IMAGES_APXS])
        self._counter[self.BUCKET_IMAGES_APXS] += 1
    
    def _draw_text(self, draw: ImageDraw.ImageDraw, text: str, fill, position: str = "center"):
        bbox = draw.textbbox((0, 0), text, font=self.APXS_FONT)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        if position == "center":
            x = (self.APXS_WIDTH - text_width) // 2
            y = (self.APXS_HEIGHT - text_height) // 2
        elif position == "top-right":
            margin = 40
            x = self.APXS_WIDTH - text_width - margin
            y = margin
        else:
            x, y = position

        draw.text((x, y), text, fill=fill, font=self.APXS_FONT)