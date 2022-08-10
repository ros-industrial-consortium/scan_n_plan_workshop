# Scan 'N Plan Workshop

## Install

1. Install the prerequisite packages:
    - `taskflow` (from the ROS-I PPA)
      ```bash
      sudo add-apt-repository ppa:ros-industrial/ppa
      sudo apt-get update
      sudo apt-get install taskflow
        ```

1. Install the ROS2 dependencies
    ```bash
    cd <snpd_workspace>
    vcs import < src/scan_n_plan_workshop/dependencies_tesseract.repos
    vcs import < src/scan_n_plan_workshop/dependencies.repos
    vcs import < src/scan_n_plan_workshop/dependencies_hardware.repos
    rosdep install --from-paths src --ignore-src -r -y
    ```

1. Build
    ```bash
    colcon build --cmake-args -DTESSERACT_BUILD_FCL=OFF
    ```

1. Build the ros1_bridge
    - Create a new workspace, clone this branch of the ros1_bridge repo
    ```bash
    git clone -b action_bridge https://github.com/ipa-hsd/ros1_bridge.git
    ```
    - Source the both ROS distros
    ```bash
    source /opt/ros/foxy/setup.bash
    source /opt/ros/noetic/setup.bash
    ```
    - Build the bridge
    ```bash
    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
    ```

1. Build the ROS1 workspace
    - Create a new workspace, clone this commit of the motoman repo
    ```bash
    git clone -b 63c94ec https://github.com/ros-industrial/motoman.git
    ```
    - Source the ROS1 installation
    ```bash
    source /opt/ros/noetic/setup.bash
    ```
    - Build the repo
    ```bash
    catkin build
    ```

## Running the system

1. Start the ROS1 launch file
    - ```bash
      roslaunch motoman_hc10_support robot_interface_streaming_hc10.launch robot_ip:=192.168.1.31 controller:=yrc1000
        ```
1. Source both ROS distros and run the bridge
    - ```bash
      source /opt/ros/noetic/setup.bash
      source install/setup.bash
      ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
        ```
1. Start the ROS2 launch file
    - ```bash
      ros2 launch snp_bringup_ros2 start.launch.xml sim_robot:=false
        ```
