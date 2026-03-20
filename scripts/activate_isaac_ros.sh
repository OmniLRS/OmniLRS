#!/usr/bin/env bash

bridge_lib="${CONDA_PREFIX}/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"

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
