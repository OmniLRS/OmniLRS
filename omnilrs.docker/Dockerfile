FROM nvcr.io/nvidia/isaac-sim:5.0.0

RUN /isaac-sim/python.sh -m pip install --upgrade pip
RUN /isaac-sim/python.sh -m pip install \
  opencv-python \ 
  omegaconf \ 
  hydra-core

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
  tmux \
  vim \
  nano \
  && rm -rf /var/lib/apt/lists/*


# setup keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=humble

# install ros2 packages  
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-ros-core \
  gdal-bin \
  libgdal-dev \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-tf2-msgs \  
  g++ \
  wget \
  && rm -rf /var/lib/apt/lists/*
 
RUN /isaac-sim/python.sh -m pip install \
  skyfield \
  zfpy \
  numba \
  gdown \
  em \
  lark

RUN /isaac-sim/python.sh -m pip install gdal==$(gdal-config --version)

# isaac ros2_bridge config  
ENV isaac_sim_package_path=/isaac-sim
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib

## For convenience:
RUN echo "alias kk='clear'" >> /root/.bashrc
RUN echo "alias tt='tmux new'" >> /root/.bashrc
RUN echo "alias isaacpy='/isaac-sim/python.sh'" >> /root/.bashrc
RUN echo "alias isaacapp='/isaac-sim/runapp.sh'" >> /root/.bashrc
RUN echo "alias humble='unset LD_LIBRARY_PATH && source /opt/ros/$ROS_DISTRO/setup.bash'" >> /root/.bashrc
# Save the bash history immediately on each execution
RUN echo "export PROMPT_COMMAND='history -a' " >> "/root/.bashrc"
RUN echo "export HISTFILE=/commandhistory/.bash_history" >> "/root/.bashrc"

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
