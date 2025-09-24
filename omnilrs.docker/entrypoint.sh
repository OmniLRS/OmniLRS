#!/bin/bash
set -e

## removed because it overrides the LD_LIBRARY_PATH set in the Dockerfile that loads the isaacsim builtin ros2 librairies (python 11)
# setup ros2 environment
# source "/opt/ros/$ROS_DISTRO/setup.bash" --
## to use ROS2 installed from apt (python 3.10) in the docker, unset LD_LIBRARY_PATH and source it manually (use alias 'humble')
## see references of this mess in the issacsim docs
## https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_ros.html#isaac-sim-app-no-system-installed-ros

cd /workspace/omnilrs
exec "$@"