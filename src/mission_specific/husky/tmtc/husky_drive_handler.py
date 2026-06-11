__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from src.tmtc.drive_handler import DriveHandler


class HuskyDriveHandler(DriveHandler):
    """
    Husky-specific drive handler.

    Overrides the drive gate so that the Husky can always drive without
    requiring a GO command, motor controller power-on, or health checks.
    """

    def _is_robot_able_to_drive(self) -> bool:
        return True