A package to bridge actions between ROS1 and ROS2. 
   
**NOTE:**   
- Currently supports forwarding goals from ROS1 (melodic) action client to ROS2 (crystal) action server  
- As an example, implemented interfaces for the action bridge for FibonacciAction   
  and FollowJointTrajectoryAction  

**Prerequisites:**  

(*rosdep does not work properly with mixed ROS1 and ROS2 dependencies*)

```
sudo apt install ros-melodic-actionlib ros-melodic-actionlib-tutorials ros-melodic-control-msgs ros-melodic-roscpp ros-crystal-control-msgs ros-crystal-example-interfaces ros-crystal-rclcpp ros-crystal-rclcpp-action
```

**How to build:**  
  
Clone the repository in the `src` folder of your ROS2 workspace.
```
git clone git@github.com:ipa-hsd/action_bridge.git
```
Since the package provides a bridge for `FollowJointTrajectoryAction`, which is part of the `control_msgs`, clone the package modified for ROS2.
```
git clone -b crystal-devel git@github.com:ros-controls/control_msgs.git
```

Since `action_bridge` package depends on both ROS1 and ROS2, source both workspaces.
```
source /opt/ros/melodic/local_setup.bash
source /opt/ros/crystal/local_setup.bash
colcon build
```
Now you are ready to run the `action_bridge`!  
Source this workspace to use the executeables built in the previous step. 
```
source <path-to-workspace>/install/local_setup.bash
```
2 example executables are available: 
- `action_bridge_fibonacci_node` and 
- `action_bridge_follow_joint_trajectory_node`  
You can start one of these nodes in the following manner:
```
ros2 run action_bridge action_bridge_fibonacci_node
```
OR
```
ros2 run action_bridge action_bridge_follow_joint_trajectory_node
```









