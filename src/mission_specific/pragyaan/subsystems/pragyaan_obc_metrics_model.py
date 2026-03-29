__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from src.mission_specific.pragyaan.subsystems.pragyaan_robot_enums import (
    CpuUsageLevel, RamUsageLevel, DiskUsageLevel
)
from src.subsystems.robot_physics_models.obc_metrics_model import ObcMetricsModel


class PragyaanObcMetricsModel(ObcMetricsModel):
    """Pragyaan-specific OBC metrics model. Overrides the base CPU/RAM/Disk usage levels
    with values from pragyaan_robot_enums to match mission-specific parameters."""

    CPU_LEVELS = CpuUsageLevel
    RAM_LEVELS = RamUsageLevel
    DISK_LEVELS = DiskUsageLevel
