__author__ = "Aleksa Stanivuk"
__copyright__ = "Copyright 2025-26, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import omni.kit.app
from src.robots.robot_enums import ObcState
from src.tmtc.intervals_handler import IntervalName

"""
Handles all general OBC-related tasks
"""
class ObcHandler:

    def __init__(
        self,
        robot,
        intervals_handler
    ) -> None:
        self._robot = robot
        self._intervals_handler = intervals_handler

    def set_obc_state(self, state:ObcState, set_to_idle_after=0):
        if self._intervals_handler.does_exist(IntervalName.OBC_STATE.value):
            self._intervals_handler.remove_interval(IntervalName.OBC_STATE.value)

        self._robot.subsystems.set_obc_state(state)

        if set_to_idle_after != 0:
            # for states that should switch back to idle after a duration
            self._intervals_handler.add_new_interval(name=IntervalName.OBC_STATE.value, seconds=set_to_idle_after, is_repeating=False, execute_immediately=False,
                                                 function=self._robot.subsystems.set_obc_state, f_args=[ObcState.IDLE])
