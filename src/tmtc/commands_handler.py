__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import socket
import threading
import time
import traceback

from src.tmtc import yamcs_naming
from src.tmtc.mdb_parsing_service import MdbParsingService


def check_camera_argument(func, command_path: str, arg_names: list[str], cameras):
    """Handler of a camera command (yamcs_naming.CAMERA_COMMAND_TYPE) that only runs with a camera_name of the robot
    cameras of the command's camera type; any other value is logged and the command dropped. The server MDB may list
    cameras this robot does not have (a startup check warning). Other commands' handlers are returned unchanged."""
    command = command_path.rsplit("/", 1)[-1]
    if command not in yamcs_naming.CAMERA_COMMAND_TYPE:
        return func
    if yamcs_naming.CAMERA_NAME_ARG not in arg_names:
        raise ValueError(f"handler of camera command {command_path} must take {yamcs_naming.CAMERA_NAME_ARG!r}")
    index = arg_names.index(yamcs_naming.CAMERA_NAME_ARG)
    allowed = yamcs_naming.cameras_for_command(cameras, command)

    def checked(*args):
        if args[index] not in allowed:
            print(
                f"{command_path}: camera {args[index]!r} is not one of this robot's cameras {allowed}, command dropped"
            )
            return
        func(*args)

    return checked


class CommandsHandler:
    UDP_RECV_MAX = 4096
    SOCKET_TIMEOUT_SEC = 2.0
    HEARTBEAT_EVERY_SEC = 10.0  # how often to log "waiting..." when idle

    def __init__(self, yamcs_processor, yamcs_instance_conf, registry: dict[bytes, dict]):
        """registry: wire command name -> command, from the server MDB (ServerMdb.registry)."""
        self._yamcs_processor = yamcs_processor
        self._commands_catalogue = {}
        self._registry = registry
        self._config_tc_socket(yamcs_instance_conf)
        self._start_tc_listener()

    def _config_tc_socket(self, yamcs_instance_conf):
        self._tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tc_socket.bind((yamcs_instance_conf["tc_receive_address"], yamcs_instance_conf["tc_receive_port"]))
        self._tc_socket.settimeout(self.SOCKET_TIMEOUT_SEC)
        print("UDP bound to:", self._tc_socket.getsockname())

    def _start_tc_listener(self):
        self._tc_stop_event = threading.Event()
        self._tc_thread = threading.Thread(
            target=self._tc_listener_loop,
            name="tc-listener",
            daemon=True,
        )
        self._tc_thread.start()

    def _tc_listener_loop(self):
        last_heartbeat = 0.0
        while not self._tc_stop_event.is_set():  # while True:
            try:
                tc_data, addr = self._tc_socket.recvfrom(self.UDP_RECV_MAX)
            except socket.timeout:
                now = time.time()
                if now - last_heartbeat >= self.HEARTBEAT_EVERY_SEC:
                    print("Heartbeat: waiting for TC on", self._tc_socket.getsockname())
                    last_heartbeat = now
                continue

            try:
                self._handle_tc(tc_data, addr)
            except Exception:
                # one failing command must not stop the listener (every later TC would be ignored)
                print(f"TC from {addr} failed: {tc_data[:40]!r}")
                traceback.print_exc()

    def _handle_tc(self, tc_data, addr):
        decoded = MdbParsingService.decode_tc_payload(tc_data, self._registry)
        if decoded is None:
            print(f"TC payload from {addr} matches no command of the server MDB: {tc_data[:40]!r}")
            return

        command = self._commands_catalogue.get(decoded["full_name"])
        if command is None:
            print(f"Command '{decoded['full_name']}' not found in catalogue")
        else:
            self._execute(command, decoded["arguments"])

    def _execute(self, command, received_arguments):
        arg_names = command["args"]
        func = command["func"]

        if arg_names == []:
            func()
        else:
            args = [received_arguments[name] for name in arg_names]
            func(*args)

    def add_command(self, command_name: str, func, args: list = []):
        # Can be called from inside _init_commands_catalogue or externally.
        if command_name in self._commands_catalogue:
            raise Exception("Command named", str(command_name), "already exists")
        elif command_name == "":
            raise Exception("Command name can not be an empty string.")

        self._commands_catalogue[command_name] = {"func": func, "args": args}
