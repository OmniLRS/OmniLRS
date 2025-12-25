__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.tmtc.commands_handler import CommandsHandler
from src.tmtc.drive_handler import DriveHandler
from src.tmtc.images_handler import ImagesHandler
from src.tmtc.intervals_handler import IntervalsHandler
from yamcs.client import YamcsClient
import omni.kit.app
from abc import ABC, abstractmethod

class YamcsTMTC(ABC):
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
        yamcs_client = YamcsClient(yamcs_conf["address"])
        self._yamcs_processor = yamcs_client.get_processor(instance=yamcs_conf["instance"], processor=yamcs_conf["processor"])
        self._robot_name = robot_name
        self._robots_RG = robot_RG
        self._yamcs_conf = yamcs_conf
        self._robot = robot
        self._intervals_handler = IntervalsHandler()
        self._drive_handler = DriveHandler(self._robot, self._intervals_handler)
        self._commands_handler:CommandsHandler = CommandsHandler(self._yamcs_processor)
        self._images_handler = ImagesHandler(self._yamcs_processor, yamcs_conf["address"], yamcs_conf["images"], yamcs_conf["url_full_nginx"])
        
    @abstractmethod
    def _setup_command_callbacks(self, commands_conf):
        """Register command callbacks for the robot."""
        pass

    @abstractmethod
    def start_streaming_data(self):
        """Register command callbacks for the robot."""
        pass
