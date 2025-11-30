FROM osrf/space-ros:humble-2024.10.0

RUN mkdir -p "${HOME_DIR}/spaceros/demo/src"
WORKDIR "${HOME_DIR}/spaceros/demo/src"
RUN git clone https://github.com/ros2/teleop_twist_keyboard.git
WORKDIR "${HOME_DIR}/spaceros/demo"

SHELL ["/bin/bash", "-c"]

## Install the demo as Space ROS package
ARG DEMO_SPACEROS_WS_PATH="${HOME_DIR}/spaceros/demo"
# hadolint ignore=SC1091
RUN source /ros_entrypoint.sh -- && \
    colcon build --merge-install --symlink-install \
    --build-base "${DEMO_SPACEROS_WS_PATH}/build" \
    --install-base "${DEMO_SPACEROS_WS_PATH}/install" && \
    rm -rf ./log

WORKDIR "${HOME_DIR}"

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]