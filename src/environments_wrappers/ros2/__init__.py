__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"


def enable_ros2(simulation_app, bridge_name="foxy", **kwargs) -> None:
    """
    Enables ROS2 in the simulation.

    Args:
        simulation_app (SimulationApp): SimulationApp instance.
        **kwargs: Additional keyword arguments."""

    from isaacsim.core.utils.extensions import enable_extension

    if bridge_name == "foxy":
        enable_extension("isaacsim.ros2.bridge")
    elif bridge_name == "humble":
        enable_extension("isaacsim.ros2.bridge") # replaced omni.isaac.ros2_bridge_humble
    else:
        raise ValueError("Bridge not supported, please choose between foxy and humble.")
    enable_extension("omni.kit.viewport.actions")

    simulation_app.update()
