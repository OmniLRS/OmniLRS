__author__ = "Antoine Richard, Junnosuke Kamohara, Aleksa Stanivuk"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import time

class Rate:
    """
    Creates a rate object that enables to sleep for a minimum amount
    of time between two iterations of a loop. If freq and dt are
    passed, the object will only use the information provided by dt.
    """

    def __init__(self, freq: float = None, dt: float = None, is_disabled: bool = False) -> None:
        """
        Args:
          freq (float): The frequency at which the loop should be executed.
          dt (float): The delta of time to be kept between two loop iterations.
        """

        self.is_disabled = is_disabled

        if not self.is_disabled:
            if dt is None:
                if freq is None:
                    raise ValueError("You must provide either a frequency or a delta time.")
                else:
                    self.dt = 1.0 / freq
            else:
                self.dt = dt

            self.last_check = time.time()

    def reset(self) -> None:
        """
        Resets the timer.
        """
        if not self.is_disabled:
            self.last_check = time.time()

    def sleep(self) -> None:
        """
        Wait for a minimum amount of time between two iterations of a loop.
        """

        if not self.is_disabled:
            now = time.time()
            delta = now - self.last_check
            # If time delta is too low sleep, else carry on.
            if delta < self.dt:
                to_sleep = self.dt - delta
                time.sleep(to_sleep)