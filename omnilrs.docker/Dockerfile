FROM nvcr.io/nvidia/isaac-sim:2023.1.1

RUN /isaac-sim/python.sh -m pip install opencv-python omegaconf hydra-core

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -q -y --no-install-recommends tzdata apt-utils && \
  rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
  dirmngr \
  gnupg2 \
  curl \
  libglib2.0-0 \
  && rm -rf /var/lib/apt/lists/*


# setup keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros2 packages
RUN apt-get update && apt-get install -y \
  ros-humble-ros-core=0.10.0-1* \
  gdal-bin \
  libgdal-dev \
  ros-humble-vision-msgs \
  ros-humble-tf2-msgs \
  g++ \
  wget \
  && rm -rf /var/lib/apt/lists/*

RUN /isaac-sim/python.sh -m pip install --upgrade pip
RUN /isaac-sim/python.sh -m pip install \
  skyfield \
  zfpy \
  numba \
  gdown \
  em \
  lark \
  pandas
RUN /isaac-sim/python.sh -m pip install gdal==$(gdal-config --version)

# isaac ros2_bridge config
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
