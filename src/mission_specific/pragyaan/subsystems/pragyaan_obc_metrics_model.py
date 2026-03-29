__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from enum import Enum

from src.subsystems.robot_physics_models.obc_metrics_model import ObcMetricsModel

# Mission Specific overrides of OBC metrics model. Modify values here to tune Pragyaan's OBC metrics behavior.
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


class PragyaanObcMetricsModel(ObcMetricsModel):
    """Pragyaan-specific OBC metrics model. Overrides the base CPU/RAM/Disk usage levels
    with values from pragyaan_robot_enums to match mission-specific parameters."""

    CPU_LEVELS = CpuUsageLevel
    RAM_LEVELS = RamUsageLevel
    DISK_LEVELS = DiskUsageLevel
