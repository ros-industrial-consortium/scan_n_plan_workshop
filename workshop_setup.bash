#!/bin/bash

# Install ROS?

# Install ROS tools - colcon, rosdep, vcstool etc
sudo apt-get install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Clone workshop repo
cd /home/user
git clone 
https://github.com/ros-industrial-consortium/roscon_workshop_2021

# Move to and vcstool dep_ws
cd /home/user/roscon_workshop_2021/dep_ws/src
vcs import < deps.rosinstall

# Move to, vcstool, and rosdep ros1_ws
cd /home/user/roscon_workshop_2021/ros1_ws/src
vcs import < ros1.rosinstall
cd /home/user/roscon_workshop_2021/ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y

# Move to, vcstool, and rosdep ros2_ws
cd /home/user/roscon_workshop_2021/ros2_ws/src
vcs import < ros2.rosinstall
cd /home/user/roscon_workshop_2021/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -r -y

# Move to, vcstool, and rosdep bridge_ws
cd /home/user/roscon_workshop_2021/bridge_ws/src
vcs import < bridge.rosinstall
cd /home/user/roscon_workshop_2021/bridge_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -r -y

