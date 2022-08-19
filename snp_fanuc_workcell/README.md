# Scan 'N Plan Workshop - FANUC Demo

## Install

1. Install the prerequisite packages:
    - `taskflow` (from the ROS-I PPA)
      ```bash
      sudo add-apt-repository ppa:ros-industrial/ppa
      sudo apt-get update
      sudo apt-get install taskflow
        ```

    - `xmlrpcpp`
      ```bash
      sudo apt install libxmlrpcpp-dev
      ```

    - `pip3` (install this if you haven't already) 
      ```bash
      sudo apt-get install python3-pip
      ```
    - `open3d` 
      ```bash
      pip3 install open3d
        ```
    - `pyquaternion` 
      ```bash
      pip3 install pyquaternion
        ```

2. Setup ROS2 workspaces
    <br/>Create workspace project folder (e.g.):
    ```bash
    mkdir fanuc_demo
    ```
    Inside fanuc_demo, create snptools, snp, bridge, and ros1 workspaces (e.g.):
    ```bash
    mkdir -p snptools_ws/src
    mkdir -p snp_ws/src
    mkdir -p ros_bridge_ws/src
    mkdir -p ros1_ws/src
    ```
    Clone scan-n-plan repository inside `snp_ws/src`
    ```bash
    git clone -b feature/automate2022_rebase https://github.com/Yolnan/scan_n_plan_workshop.git
    ```

3. Install snptools dependencies (e.g.):
  <br/>Inside `snptools_ws`:
    ```bash
    vcs import src < ../snp_ws/src/scan_n_plan_workshop/snp_fanuc_workcell/dependencies_hardware.repos
    vcs import src < ../snp_ws/src/scan_n_plan_workshop/dependencies_tesseract.repos
    vcs import src < ../snp_ws/src/scan_n_plan_workshop/dependencies.repos
    source /opt/ros/foxy/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build snptools workspace
    ```bash
    colcon build --cmake-args -DTESSERACT_BUILD_FCL=OFF
    ```

5. Install snp workspace dependencies (e.g.): 
    <br/>In a new terminal inside `snp_ws`:
    ```bash
    source ../snptools_ws/install/setup.bash 
    rosdep install --from-paths src --ignore-src -r -y
    ```

6. Build snp workspace
    ```bash
    colcon build
    ```

7. Build the ros1_bridge
    <br/>In a new terminal inside `ros_bridge_ws`:
    - Clone ros1_bridge repository inside `ros_bridge_ws/src`
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

8. Build the ROS1 workspace
    - Inside `ros1_ws`:
    ```bash
    vcs import src < ../snp_ws/src/scan_n_plan_workshop/snp_fanuc_workcell/dependencies_ros1.repos
    source /opt/ros/noetic/setup.bash
    rosdep update
    rosdep install --from-paths src/ --ignore-src --rosdistro noetic
    catkin build
    ```

## Running the system

1. Start the ROS1 launch file
    ```bash
    source devel/setup.bash
    roslaunch fanuc_lrmate200ic_support robot_interface_streaming_lrmate200ic.launch robot_ip:=192.168.0.1
    ```
2. Run the ros bridge
     ```bash
    source install/setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
    ```
3. Start the ROS2 launch file
    ```bash
    source install/setup.bash
    ros2 launch snp_fanuc_workcell start.launch.xml sim_robot:=false sim_vision:=false
    ```

4. Start End-effector motor control nodes
    ```bash
    roslaunch ee_motor_io ee_motor_io.launch
    ```