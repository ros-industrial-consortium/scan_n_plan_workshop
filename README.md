# ROSCon2021

This repository contains three workspaces: a ROS1 catkin workspace, a ROS2 colcon workspace, and a colcon workspace for building the ROS1-ROS2 bridge.  This means that dependencies need to be cloned into the `src` directories two levels down in this workspace.  To avoid git zaniness, add the directories for each source dependency to the `.gitignore` file of the `src` directory into which it gets cloned.
