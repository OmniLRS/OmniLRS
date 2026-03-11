__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.tmtc.commands_handler import CommandsHandler
from src.tmtc.drive_handler import DriveHandler
from src.tmtc.images_handler import ImagesHandler
from src.tmtc.intervals_handler import IntervalsHandler
from yamcs.client import YamcsClient
import omni.kit.app
from abc import ABC, abstractmethod
from src.tmtc.obc_handler import ObcHandler

class YamcsTMTC(ABC):
    """
    YamcsTMTC is implementation of Yamcs-TMTC framework. 
    It allows to control a robot instance, by receiving TCs from Yamcs and sending TM to Yamcs.

    It is not intended to be used standalone, but as a class that is inherited by a rover controller.
    Take a loot at PragyaanController for reference implementation of how YamcsTMTC should be utilized for a specific rover context.

    The class provides handlers that enable working with general concepts of Yamcs-TMTC framework, that would be used by all rovers.
    YamcsTMTC is not constrained by specific rover implementation and functionalities, but is implemented on a high level. 
    Specific rover controller classes are intended to adhere to the required rover functionalities by building upon the concepts 
    provided by YamcsTMTC.

    Description of handlers:
        IntervalsHandler - allows for easy handling of intervals, thus enables scheduling (non-)repeating tasks in intervals.
        ObcHandler - facilitates work with rover's OBC, such as updating the rover's state from a central point.
        DriveHandler - handles commanding of rover's drive mechanichs by transfering high level commands in term of speed and distance 
                            into low level wheel movements.
        CommandsHandler - handles addition and execution of commands coming from Yamcs by conducing mapping between the predefined commands and 
                            specific functions (functions are implemented inside specific rover controllers that will utilize the CommandsHandler) 
        ImagesHandler - facilitates working with Yamcs bucket storages for saving the images created inside the simulator

    The class also implements (two) abstract methods that are required to be implemented by the inheriting controllers:
        _setup_command_callbacks(commands_conf) - utilizes CommandsHandler to construct the mapping between high level Yamcs commands, 
                                                    and desired functionalities (check PragyaanController for ref. impl.)
        start_streaming_data - utilizes IntervalsHandler to setup and trigger update of rover's parameters in Yamcs (check PragyaanController for ref. impl.)

    """ 
    def __init__(
        self,
        yamcs_instance_conf,
        yamcs_conf,
        robot_name,
        robot_RG,
        robot,
    ) -> None:
        yamcs_client = YamcsClient(yamcs_instance_conf["address"])
        self._yamcs_processor = yamcs_client.get_processor(instance=yamcs_instance_conf["instance"], processor=yamcs_instance_conf["processor"])
        self._robot_name = robot_name
        self._robots_RG = robot_RG
        self._yamcs_conf = yamcs_conf
        self._robot = robot
        #creating the five generic handlers for the main functions of the robot
        self._intervals_handler:IntervalsHandler = IntervalsHandler()
        self._obc_handler:ObcHandler = ObcHandler(self._robot, self._intervals_handler)
        self._drive_handler:DriveHandler = DriveHandler(self._robot, self._intervals_handler, self._obc_handler)
        self._commands_handler:CommandsHandler = CommandsHandler(self._yamcs_processor, yamcs_instance_conf)
        self._images_handler:ImagesHandler = ImagesHandler(self._yamcs_processor, yamcs_instance_conf["address"], yamcs_conf["images"], yamcs_instance_conf["url_full_nginx"])

    @abstractmethod
    def _setup_command_callbacks(self, commands_conf):
        """Register command callbacks for the robot."""
        pass

    @abstractmethod
    def start_streaming_data(self):
        """Register command callbacks for the robot."""
        pass
