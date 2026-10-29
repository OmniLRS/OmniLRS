__author__ = "Antoine Richard"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

SUPPORTED_ROS_DISTROS = ("jazzy", "humble")


def enable_ros2(simulation_app, bridge_name="jazzy", **kwargs) -> None:
    """
    Enables ROS2 in the simulation.

    The ROS 2 libraries bundled with Isaac Sim (isaacsim.ros2.core) are selected through the ROS_DISTRO
    environment variable, which must match `bridge_name`.

    Args:
        simulation_app (SimulationApp): SimulationApp instance.
        bridge_name (str): ROS 2 distribution, "jazzy" or "humble".
        **kwargs: Additional keyword arguments."""

    import os

    from isaacsim.core.utils.extensions import enable_extension

    if bridge_name not in SUPPORTED_ROS_DISTROS:
        raise ValueError(f"Bridge not supported, please choose between {', '.join(SUPPORTED_ROS_DISTROS)}.")
    ros_distro = os.environ.setdefault("ROS_DISTRO", bridge_name)
    if ros_distro != bridge_name:
        raise ValueError(f"ROS_DISTRO is '{ros_distro}' but the ROS2 mode requests '{bridge_name}'.")

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("omni.kit.viewport.actions")

    simulation_app.update()
