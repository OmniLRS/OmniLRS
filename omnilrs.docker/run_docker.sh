#!/bin/bash
xhost +
docker run --name isaac-sim-omnilrs-container -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --ipc=host \
-v $HOME/.Xauthority:/root/.Xauthority \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v ${PWD}:/workspace/omnilrs \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v /tmp/images_streaming:/tmp/images_streaming:rw \
-v /tmp/images_oncommand:/tmp/images_oncommand:rw \
-v /tmp/images_apxs:/tmp/images_apxs:rw \
-v /tmp/images_depth:/tmp/images_depth:rw \
-v bash_command_history:/commandhistory \
isaac-sim-omnilrs:latest
