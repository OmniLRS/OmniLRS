__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"
import time
import omni.kit.app

import socket
import time
import threading

from src.tmtc.mdb_parsing_service import MdbParsingService

class CommandsHandler():
    UDP_RECV_MAX        = 4096  
    SOCKET_TIMEOUT_SEC  = 2.0 
    HEARTBEAT_EVERY_SEC = 10.0  # how often to log "waiting..." when idle


    def __init__(self, yamcs_processor, yamcs_instance_conf, mdb_files:list[str]):
        self._yamcs_processor = yamcs_processor
        self._commands_catalogue = {}
        self._registry = MdbParsingService.load_mdb_registry(mdb_files) 
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
                    print("Waiting for TC on", self._tc_socket.getsockname())
                    last_heartbeat = now
                continue

            decoded = MdbParsingService.decode_tc_payload(tc_data, self._registry)
            if decoded is None:
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

    def add_command(self, command_name:str, func, args:list=[]):
        # Can be called from inside _init_commands_catalogue or externally.
        if command_name in self._commands_catalogue:
            raise Exception("Command named", str(command_name), "already exists")
        elif command_name == "":
            raise Exception("Command name can not be an empty string.")

        self._commands_catalogue[command_name] = {"func":func, "args":args}