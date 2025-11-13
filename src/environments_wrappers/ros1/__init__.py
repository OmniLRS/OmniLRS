__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"


def enable_ros1(simulation_app, **kwargs) -> None:
    """
    Enables ROS1 in the simulation.

    Args:
        simulation_app (SimulationApp): SimulationApp instance.
        **kwargs: Additional keyword arguments."""

    # Enables this ROS1 extension
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.isaac.ros_bridge")
    enable_extension("omni.kit.viewport.actions")
    simulation_app.update()

    # Checks that the master is running
    import rosgraph
    import carb

    if not rosgraph.is_master_online():
        carb.log_error("Please run roscore before executing this script")
        simulation_app.close()
        exit()
