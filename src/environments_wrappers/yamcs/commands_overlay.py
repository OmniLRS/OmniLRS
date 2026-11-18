__author__ = "Louis Burtz"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


from src.tmtc.commands_handler import RECEIVED_COMMANDS


class YamcsCommandsOverlay:
    """
    Text overlay of the robot video in Yamcs mode (text_overlay of src/environments_wrappers/robot_video_recorder.py):
    the last LINES Yamcs commands, each with the simulation time at which the simulation received it. The newest command
    is highlighted for NEW_S. Commands received before the overlay is created are not shown.
    """

    LINES = 3
    NEW_S = 3.0  # simulation time

    def __init__(self) -> None:
        self._last_number = max((number for number, _, _ in list(RECEIVED_COMMANDS)), default=0)
        self._commands = []  # (simulation time of receipt, text), oldest first

    def __call__(self, sim_time: float) -> list[tuple[str, bool]]:
        """Stamps the commands received since the last call with `sim_time`; returns the (text, highlighted) lines."""
        for number, name, arguments in list(RECEIVED_COMMANDS):
            if number <= self._last_number:
                continue
            self._last_number = number
            short = name.split("/", 2)[-1] if name.count("/") >= 2 else name  # /Rover/motor/x -> motor/x
            values = " ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in arguments.items())
            self._commands = (self._commands + [(sim_time, f"{short} {values}".strip())])[-self.LINES :]
        newest = len(self._commands) - 1
        return [
            (f"t={received:6.1f} s  {text}", i == newest and sim_time - received < self.NEW_S)
            for i, (received, text) in enumerate(self._commands)
        ]
