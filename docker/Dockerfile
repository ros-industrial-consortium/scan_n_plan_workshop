# syntax = docker/dockerfile:1.2

ARG BASE_IMAGE="ubuntu:20.04"
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
  && apt upgrade -y \
  && apt install -y --no-install-recommends \
    apt-utils \
    build-essential \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    git \
    gpg-agent \
    locales \
    software-properties-common \
    sudo \
  && add-apt-repository universe \
  && rm -rf /var/lib/apt/lists/* \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG-en_US.UTF-8

# Add the ROS aptitude sources and GPG key so we can install it from debians
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
  && sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt update \
  && sudo apt upgrade -y \
  && sudo apt install -y \
      python3-colcon-common-extensions \
      
      ros-foxy-ros-base \
      
      python3-catkin-tools \
      python3-rospkg \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rosmsg \
      ros-noetic-actionlib \
      ros-noetic-diagnostic-msgs \

      ros-noetic-actionlib-msgs \
      ros-foxy-actionlib-msgs \
      ros-noetic-std-msgs \
      ros-foxy-std-msgs \
      ros-noetic-sensor-msgs \
      ros-foxy-sensor-msgs \
      ros-noetic-control-msgs \
      ros-foxy-control-msgs \
      ros-noetic-rosgraph-msgs \
      ros-foxy-rosgraph-msgs \
      ros-noetic-trajectory-msgs \
      ros-foxy-trajectory-msgs \
      ros-noetic-tf2-msgs \
      ros-foxy-tf2-msgs \
      ros-noetic-geometry-msgs \
      ros-foxy-geometry-msgs


RUN cd /opt \
 && git clone https://github.com/DavidSpielman/ros1_bridge.git \
 && cd ros1_bridge \
 && . /opt/ros/foxy/setup.sh \
 && . /opt/ros/noetic/setup.sh \
 && colcon build --symlink-install

COPY --chmod=755 launch_rosbridge.sh /opt/launch_rosbridge.sh

ENTRYPOINT ["/opt/launch_rosbridge.sh"]
