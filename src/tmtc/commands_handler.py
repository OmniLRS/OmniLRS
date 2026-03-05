__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"
from yamcs.client import  CommandHistory
import time
import omni.kit.app

class CommandsHandler():

    def __init__(self, yamcs_processor):
        self._yamcs_processor = yamcs_processor
        self._yamcs_processor.create_command_history_subscription(on_data=self._command_callback) # immeditally starts listening for commands once created
        self._time_of_last_command = 0
        self._commands_catalogue = {}

    def _command_callback(self, received_command:CommandHistory):
        # CommandHistory info is available at: https://docs.yamcs.org/python-yamcs-client/tmtc/model/#yamcs.client.CommandHistory
        #NOTE: it happens that the subscriber receives the same instance of command multiple times in a very short period of time
        # however, desired behavior is to execute the command only once
        # since commands are executed by human operators, waiting for a small period of time (such as 0.5s) is enough to counter this issue
        if time.time() - self._time_of_last_command < 0.5:
            return
        
        self._time_of_last_command = time.time()

        name = received_command.name
        received_arguments = received_command.all_assignments
        print(name)
        print(received_arguments)
        command = self._commands_catalogue.get(name)

        if command is None:
            raise Exception(f"Command '{name}' not found in catalogue")

        self._execute(command, received_arguments)
    
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