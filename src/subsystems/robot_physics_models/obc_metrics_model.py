__author__ = "Louis Burtz, Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import random
import time
from enum import Enum

from src.subsystems.robot_enums import ObcState
from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel


class CpuUsageLevel(float, Enum):
    LOW = 25.0
    MEDIUM = 50.0
    HIGH = 75.0

class RamUsageLevel(float, Enum):
    LOW = 40.0
    MEDIUM = 50.0
    HIGH = 60.0

class DiskUsageLevel(float, Enum):
    LOW = 10.0
    MEDIUM = 25.0
    HIGH = 75.0


class ObcMetricsModel(RobotPhysicsModel):
    """ Simulates OBC metrics like CPU, RAM, Disk usage and Uptime.
    Basic model that can be extended/overridden by specific missions
    
    - easily change the CPU/RAM/Disk usage levels to match the mission-specific parameters (see PragyaanObcMetricsModel for example).
    - override the initialize/set_inputs/compute/get_outputs methods to implement the logic for computing the OBC metrics
    """

    CPU_LEVELS = CpuUsageLevel
    RAM_LEVELS = RamUsageLevel
    DISK_LEVELS = DiskUsageLevel

    def __init__(self):
        self._state = ObcState.IDLE
        self._boot_ts = time.monotonic()
    
    def initialize(self):
        pass

    def set_inputs(self, state:ObcState):
        self._input_obc_state(state)

    def compute(self):
         self._obc_metrics = {
            "cpu_usage": self._get_obc_cpu_usage(),
            "ram_usage": self._get_obc_ram_usage(),
            "disk_usage": self._get_obc_disk_usage(),
            "uptime": self._get_obc_uptime(),
        }
        
    def get_outputs(self):
        return self._obc_metrics
    
    def _input_obc_state(self, state:ObcState):
        # First, check if need to reset uptime (whenever the controller transitions through OFF)
        if state == ObcState.OFF or (self._state == ObcState.OFF and state != ObcState.OFF):
            self._boot_ts = time.monotonic()
        # keep track of the obc state internally:
        self._state = state

    def _get_obc_cpu_usage(self):
        base = self._select_by_state(self.CPU_LEVELS)
        return self._usage_with_noise(base)

    def _get_obc_ram_usage(self):
        base = self._select_by_state(self.RAM_LEVELS)
        return self._usage_with_noise(base)

    def _get_obc_disk_usage(self):
        base = self._select_by_state(self.DISK_LEVELS)
        return self._usage_with_noise(base)

    def _get_obc_uptime(self):
        elapsed = int(time.monotonic() - self._boot_ts)
        return elapsed % (2 ** 32)

    def _select_by_state(self, levels):
        if self._state == ObcState.CAMERA:
            return levels.HIGH.value
        if self._state == ObcState.MOTOR:
            return levels.MEDIUM.value
        return levels.LOW.value

    def _usage_with_noise(self, base, noise=2.0):
        noisy_value = base + random.uniform(-noise, noise)
        return max(0.0, min(100.0, noisy_value))
    



