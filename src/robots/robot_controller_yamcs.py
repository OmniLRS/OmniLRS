from src.robots.robot import Robot


class RobotControllerYamcs:
    """
    RobotControllerYamcs class.
    It allows to control a robot instance, by subscribing and reacting to telecomands coming from Yamcs client.
    It also publishes relevant robot information to Yamcs client.
    """

    def __init__(
        self,
        robot: Robot,
    ) -> None:
        self._robot = robot

    def start_transmitting_to_yamcs(self):
        pass

