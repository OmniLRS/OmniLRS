__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import logging
from abc import ABC, abstractmethod
from typing import Callable

from yamcs.client import YamcsClient

from src.tmtc.commands_handler import CommandsHandler
from src.tmtc.drive_handler import DriveHandler
from src.tmtc.images_handler import ImagesHandler
from src.tmtc.intervals_handler import IntervalsHandler
from src.tmtc.mdb_parsing_service import MdbParsingService, ServerMdb
from src.tmtc.obc_handler import ObcHandler
from src.tmtc.yamcs_contract import check_contract

logging.getLogger("urllib3").setLevel(logging.WARNING)


class YamcsTMTC(ABC):
    """
    YamcsTMTC is implementation of Yamcs-TMTC framework.
    It allows to control a robot instance, by receiving TCs from Yamcs and sending TM to Yamcs.

    It is not intended to be used standalone, but as a class that is inherited by a rover controller.
    Take a look at mission_specific/pragyaan/tmtct/PragyaanController for reference implementation of how YamcsTMTC should be utilized for a specific rover context.

    The class provides handlers that enable working with general concepts of Yamcs-TMTC framework, that would be used by any rover.
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

    At construction the MDB (TM/TC definitions) is fetched from the Yamcs server for the space systems listed in
    `mdb_space_systems`. start() then runs the startup in order: check_yamcs_server() checks the MDB and the storage
    buckets against what this robot and controller send and handle (src/tmtc/yamcs_contract.py) and aborts if
    something is missing; old images are cleared if configured; commands are registered; telemetry starts.

    Inheriting controllers implement two abstract methods (check PragyaanController for ref. impl.):
        command_handlers() - commands-yaml key -> (handler, argument names it takes). setup_command_callbacks registers
                             each handler for its command with the CommandsHandler, and check_yamcs_server verifies
                             the command and its arguments exist in the server MDB.
        start_streaming_data - utilizes IntervalsHandler to setup and trigger update of rover's parameters in Yamcs

    """

    # Streaming telemetry the controller's transmitter sends per rgba camera (STREAMING_FIELDS of
    # src/mission_specific/<robot>/tmtc/enums.py); the startup check expects them in the server MDB.
    CAMERA_STREAMING_FIELDS: tuple[str, ...] = ()

    def __init__(
        self,
        yamcs_instance_conf,
        yamcs_conf,
        robot_name,
        robot_RG,
        robot,
    ) -> None:
        yamcs_client = YamcsClient(yamcs_instance_conf["address"])
        self._yamcs_processor = yamcs_client.get_processor(
            instance=yamcs_instance_conf["instance"], processor=yamcs_instance_conf["processor"]
        )
        self._robot_name = robot_name
        self._robot_RG = robot_RG
        self._yamcs_conf = yamcs_conf
        self._robot = robot
        self._storage = yamcs_client.get_storage_client()
        self._mdb: ServerMdb = MdbParsingService.load_from_server(
            yamcs_client.get_mdb(yamcs_instance_conf["instance"]), yamcs_conf["mdb_space_systems"]
        )
        # creating the five generic handlers for the main functions of the robot
        self._intervals_handler: IntervalsHandler = IntervalsHandler()
        self._obc_handler: ObcHandler = ObcHandler(self._robot, self._intervals_handler)
        self._drive_handler: DriveHandler = DriveHandler(self._robot, self._intervals_handler, self._obc_handler)
        self._commands_handler: CommandsHandler = CommandsHandler(
            self._yamcs_processor, yamcs_instance_conf, self._mdb.registry
        )
        self._images_handler: ImagesHandler = ImagesHandler(
            self._storage,
            self._yamcs_processor,
            yamcs_instance_conf["address"],
            yamcs_conf["images"],
            yamcs_instance_conf["url_full_nginx"],
            robot.cameras,
        )

    def start(self, commands_conf) -> None:
        """Startup in Yamcs mode, once the controller is built."""
        self.check_yamcs_server()
        self._images_handler.prepare_buckets()
        self.setup_command_callbacks(commands_conf)
        self.start_streaming_data()

    def check_yamcs_server(self) -> None:
        """Check the Yamcs server MDB and buckets against this robot and controller; raise if anything OmniLRS needs
        is missing."""
        report = check_contract(
            mdb=self._mdb,
            buckets={bucket.name for bucket in self._storage.list_buckets()},
            cameras=self._robot.cameras,
            streaming_fields=self.CAMERA_STREAMING_FIELDS,
            yamcs_conf=self._yamcs_conf,
            command_args={key: args for key, (_, args) in self.command_handlers().items()},
        )
        print(report.summary(self._yamcs_conf["mdb_space_systems"]), flush=True)
        if not report.ok:
            raise RuntimeError(
                f"The Yamcs server is missing {len(report.missing)} items OmniLRS needs (see the summary above). "
                "Regenerate the MDB and buckets from this OmniLRS checkout and restart Yamcs."
            )

    @abstractmethod
    def command_handlers(self) -> dict[str, tuple[Callable, list[str]]]:
        """Commands the robot handles: commands-yaml key -> (handler, argument names it takes, in call order)."""
        pass

    def setup_command_callbacks(self, commands_conf):
        """Register each handler of command_handlers() for its command path in the controller yaml `commands`."""
        for key, (handler, args) in self.command_handlers().items():
            self._commands_handler.add_command(commands_conf[key], handler, args=args)

    @abstractmethod
    def start_streaming_data(self):
        """Start streaming data from the robot to Yamcs."""
        pass

    def transmit_to_yamcs(self, param_name, param_value):
        self._yamcs_processor.set_parameter_value(param_name, param_value)
