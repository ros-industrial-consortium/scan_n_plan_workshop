# Build
- First build deps_ws
  - Open new termial with clean environment
  - cd deps_ws
  - colcon build --cmake-args -G 'CodeBlocks - Unix Makefiles' -DCMAKE_BUILD_TYPE=Release
  - cd ros1_ws
  - source /opt/ros/noetic/setup.bash
  - source ../deps_ws/install/setup.bash
  - colcon build --cmake-args -G 'CodeBlocks - Unix Makefiles' -DCMAKE_BUILD_TYPE=Release
  
# Launch with hardware
``` bash
roslaunch snp_bringup_ros1 start.launch sim_robot:=false sim_sensor:=false
```

# Launch without hardware
``` bash
roslaunch snp_bringup_ros1 start.launch sim_robot:=true sim_sensor:=true
```
