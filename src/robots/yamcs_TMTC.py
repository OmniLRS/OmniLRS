__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.environments.utils import transform_orientation_into_xyz
from yamcs.client import YamcsClient
import threading
import time
import numpy as np

class YamcsTMTC:
    """
    YamcsTMTC class.
    It allows to control a robot instance, by receiving TCs from Yamcs and sending TM to Yamcs.
    """ 

    def __init__(
        self,
        yamcs_conf,
        robot_name,
        robot_RG
    ) -> None:
        self._yamcs_client = YamcsClient(yamcs_conf["address"])
        self._yamcs_processor = self._yamcs_client.get_processor(instance=yamcs_conf["instance"], processor=yamcs_conf["processor"])
        self._robot_name = robot_name
        self._robots_RG = robot_RG
        self._yamcs_conf = yamcs_conf

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
                # add here further commands
                time.sleep(interval_s) #TODO: change into simulation secs
        finally:
            print("ended transmitter for: " + robot_name)

    def _transmit_base_link_pose(self):
        position, orientation = self._robots_RG[str(self._robot_name)].get_base_link_pose()
        # euler_orient = transform_orientation_into_xyz(orientation)
        position = [round(x, 1) for x in position.tolist()]         # position = position.tolist()
        orientation = [round(x, 1) for x in orientation.tolist()]   # orientation = orientation.tolist()
        pose_of_base_link = {"position": {"x":position[0], "y":position[1], "z":position[2]}, 
                                "orientation":{"w":orientation[0],"x":orientation[1], "y":orientation[2], "z":orientation[3] }}
        self._yamcs_processor.set_parameter_value(self._yamcs_conf["parameters"]["pose_of_base_link"], pose_of_base_link)

