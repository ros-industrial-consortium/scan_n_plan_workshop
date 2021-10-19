#!/bin/bash

# Get Taskflow and misc deps
sudo add-apt-repository -y ppa:ros-industrial/ppa
sudo apt-get update
sudo apt-get install -y taskflow libompl-dev

# Clone workshop repo
cd $HOME
git clone https://github.com/ros-industrial-consortium/roscon_workshop_2021.git workshop/

# Move to and vcstool deps_ws
cd ~/workshop/deps_ws
vcs import --shallow src/ < src/deps.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y

# Move to, vcstool, and rosdep ros1_ws
cd ~/workshop/ros1_ws
vcs import --shallow src/ < src/ros1.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y

# Move to, vcstool, and rosdep ros2_ws
cd ~/workshop/ros2_ws
vcs import --shallow src/ < src/ros2.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y

# Move to, vcstool, and rosdep bridge_ws
cd ~/workshop/bridge_ws
vcs import --shallow src/ < src/bridge.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y

# Build deps_ws
cd ~/workshop/deps_ws
colcon build --merge-install
