__author__ = "Louis Burtz, Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import random
import time

from src.subsystems.robot_enums import ObcState
from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel


class ObcMetricsModel(RobotPhysicsModel):
    """ Simulates OBC metrics like CPU, RAM, Disk usage and Uptime.
    Usage: 
        Instantiate the class, then call input_obc_state() to share the state of the OBC with this simulator.
        Then, call the get_*() methods to get the respective metrics.
    """
    def __init__(self):
        self._state = ObcState.IDLE
        self._boot_ts = time.monotonic()
    
    def initialize(self):
        pass

    def set_inputs(self, state:ObcState):
        self._input_obc_state(state)

    def compute(self):
        pass

    def get_outputs(self):
        return {
            "cpu_usage": self._get_obc_cpu_usage(),
            "ram_usage": self._get_obc_ram_usage(),
            "disk_usage": self._get_obc_disk_usage(),
            "uptime": self._get_obc_uptime(),
        }
    
    def _input_obc_state(self, state:ObcState):
        # First, check if need to reset uptime (whenever the controller transitions through OFF)
        if state == ObcState.OFF or (self._state == ObcState.OFF and state != ObcState.OFF):
            self._boot_ts = time.monotonic()
        # keep track of the obc state internally:
        self._state = state

    def _get_obc_cpu_usage(self):
        base = self._select_by_state(25.0, 50.0, 75.0)
        return self._usage_with_noise(base)

    def _get_obc_ram_usage(self):
        base = self._select_by_state(40.0, 50.0, 60.0)
        return self._usage_with_noise(base)

    def _get_obc_disk_usage(self):
        base = self._select_by_state(10.0, 25.0, 75.0)
        return self._usage_with_noise(base)

    def _get_obc_uptime(self):
        elapsed = int(time.monotonic() - self._boot_ts)
        return elapsed % (2 ** 32)

    def _select_by_state(self, low, medium, high):
        if self._state == ObcState.CAMERA:
            return high
        if self._state == ObcState.MOTOR:
            return medium
        return low

    def _usage_with_noise(self, base, noise=2.0):
        noisy_value = base + random.uniform(-noise, noise)
        return max(0.0, min(100.0, noisy_value))
    



