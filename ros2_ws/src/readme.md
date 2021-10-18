# Build
- First build deps_ws
  - Open new termial with clean environment
  - cd deps_ws
  - colcon build --cmake-args -G 'CodeBlocks - Unix Makefiles' -DCMAKE_BUILD_TYPE=Release
  - cd ros2_ws
  - source /opt/ros/galactic/setup.bash
  - source ../deps_ws/install/setup.bash
  - colcon build --cmake-args -G 'CodeBlocks - Unix Makefiles' -DCMAKE_BUILD_TYPE=Release

# Launch with hardware
``` bash
ros2 launch snp_bringup_ros2 start.launch sim_robot:=false sim_sensor:=false
```

# Launch without hardware
``` bash
ros2 launch snp_bringup_ros2 start.launch sim_robot:=true sim_sensor:=true
```
