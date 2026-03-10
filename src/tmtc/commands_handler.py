__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"
from yamcs.client import  CommandHistory
import time
import omni.kit.app

import socket
import time
import threading

from src.tmtc.mdb_register import decode_tc_payload, load_mdb_registry

class CommandsHandler():

    TC_RECEIVE_ADDRESS = "127.0.0.1"
    TC_RECEIVE_PORT    = 10025

    UDP_RECV_MAX        = 4096  
    SOCKET_TIMEOUT_SEC  = 2.0 
    HEARTBEAT_EVERY_SEC = 10.0  # how often to log "waiting..." when idle

    def __init__(self, yamcs_processor):
        self._yamcs_processor = yamcs_processor
        self._commands_catalogue = {}

        self._registry = load_mdb_registry("cfg/mdb")

        self._tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tc_socket.bind((self.TC_RECEIVE_ADDRESS, self.TC_RECEIVE_PORT))
        self._tc_socket.settimeout(self.SOCKET_TIMEOUT_SEC)
        print("UDP bound to:", self._tc_socket.getsockname())

        self._stop_event = threading.Event()
        # self._start_listening_to_TC()
        self._tc_thread = threading.Thread(
            target=self._start_listening_to_TC,
            name="tc-listener",
            daemon=True,
        )
        self._tc_thread.start()

    def _start_listening_to_TC(self):
        print("OBS software running ...")
        last_heartbeat = 0.0
        try:
            # while True:
            while not self._stop_event.is_set():
                try:
                    tc_data, addr = self._tc_socket.recvfrom(self.UDP_RECV_MAX)
                except socket.timeout:
                    now = time.time()
                    if now - last_heartbeat >= self.HEARTBEAT_EVERY_SEC:
                        print("Waiting for TC on", self._tc_socket.getsockname())
                        last_heartbeat = now
                    continue

                print("tc_data:",tc_data)
                print()

                decoded = decode_tc_payload(tc_data, self._registry)
                print("decoded",decoded)
                print()

                if decoded is None:
                    return

                command = self._commands_catalogue.get(decoded["full_name"])
                if command is None:
                    raise Exception(f"Command '{decoded['full_name']}' not found in catalogue")

                self._execute(command, decoded["arguments"])

        except KeyboardInterrupt:
            print("\nStopping OBS...")
    
    def _execute(self, command, received_arguments):
        arg_names = command["args"]
        func = command["func"]

        print(func)
        print(arg_names)

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