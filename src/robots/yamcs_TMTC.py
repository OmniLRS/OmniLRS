__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.environments.utils import transform_orientation_into_xyz
from yamcs.client import YamcsClient, CommandHistory
import threading
import time
import numpy as np
from pxr import Gf, UsdGeom, Usd, UsdPhysics, Sdf
import math

class YamcsTMTC:
    """
    YamcsTMTC class.
    It allows to control a robot instance, by subscribing and reacting to TMTC instructions coming from Yamcs client.
    It also publishes relevant robot information to Yamcs client.
    """

    def __init__(
        self,
        yamcs_conf,
        robot_name,
        robot_RG,
        robot,
    ) -> None:
        self._yamsc_client = YamcsClient(yamcs_conf["address"])
        self._yamsc_processor = self._yamsc_client.get_processor(instance=yamcs_conf["instance"], processor=yamcs_conf["processor"])
        # self._yamsc_processor.commands.subscribe(self._command_callback)
        self._mdb = self._yamsc_client.get_mdb(instance=yamcs_conf["instance"])  #NOTE https://docs.yamcs.org/python-yamcs-client/mdb/client/#yamcs.client.MDBClient
        self._robot_name = robot_name
        self._robots_RG = robot_RG
        self._yamcs_conf = yamcs_conf
        self._time_of_last_command = 0
        self._robot = robot

        self._yamsc_processor.create_command_history_subscription(on_data=self._command_callback)

    def _command_callback(self, command:CommandHistory):
        # CommandHistory info is available at: https://docs.yamcs.org/python-yamcs-client/tmtc/model/#yamcs.client.CommandHistory
        #NOTE: it happens that the subscriber receives the same instance of command multiple times in a very short period of time
        # however, desired behaviour is to execute the command only once
        # since commands are executed by human operators, waiting for a small period of time (such as 0.5s) is enough to counter this issue
        if time.time() - self._time_of_last_command < 0.5:
            return
        
        name = command.name
        arguments = command.all_assignments
        if name == "/Rover/motor/drive_straight":
            self._move_rover_straight(arguments["linear_velocity"], arguments["distance"]) 
        # here add reactions to other commands
        else:
            print("Unknown comand.")

    def _move_rover_straight(self, linear_velocity, distance):
        self._robot.drive(linear_velocity, distance)

    def _turn_rover(self, angular_velocity, distance):
        pass

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
                self._transmit_ground_pose()
                # add here further commands
                time.sleep(interval_s) #TODO: change into simulation secs
        finally:
            print("ended transmitter for: " + robot_name)

    def _transmit_ground_pose(self):
        position, orientation = self._robots_RG[str(self._robot_name)].get_base_pose()
        # euler_orient = transform_orientation_into_xyz(orientation)
        position = [round(x, 1) for x in position.tolist()]         # position = position.tolist()
        orientation = [round(x, 1) for x in orientation.tolist()]   # orientation = orientation.tolist()
        ground_pose_truth = {"position": {"x":position[0], "y":position[1], "z":position[2]}, 
                                "orientation":{"w":orientation[0],"x":orientation[1], "y":orientation[2], "z":orientation[3] }}
        self._yamsc_processor.set_parameter_value(self._yamcs_conf["topics"]["ground_pose_truth"], ground_pose_truth)

