__author__ = "Amaan Javed"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from src.subsystems.robot_enums import GoNogoState
from src.tmtc.drive_handler import DriveHandler


class HuskyDriveHandler(DriveHandler):
    """
    Husky-specific drive handler.

    The Husky drives while GO (its startup state), without the motor controller power and health checks: it has no
    power commands.
    """

    def _is_robot_able_to_drive(self) -> bool:
        return self._robot.subsystems.get_go_nogo_state() == GoNogoState.GO
