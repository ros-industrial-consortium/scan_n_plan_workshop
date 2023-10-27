#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/ros1_bridge/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

