#!/usr/bin/env bash

# Expose the ROS 2 libraries bundled with Isaac Sim (isaacsim.ros2.core) for ${ROS_DISTRO}.
py_ver="$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || echo 3.12)"
bridge_lib="${CONDA_PREFIX}/lib/python${py_ver}/site-packages/isaacsim/exts/isaacsim.ros2.core/${ROS_DISTRO:-jazzy}/lib"

if [ -d "$bridge_lib" ]; then
    case ":${LD_LIBRARY_PATH:-}:" in
        *":${bridge_lib}:"*) ;;
        *)
            if [ -n "${LD_LIBRARY_PATH:-}" ]; then
                export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${bridge_lib}"
            else
                export LD_LIBRARY_PATH="${bridge_lib}"
            fi
            ;;
    esac
fi
