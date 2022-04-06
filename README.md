# Scan And Plan Demo

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
    vcs import < src/roscon_2021_demo/dependencies.rosinstall
    rosdep install --from-paths src --ignore-src -r -y
    ```

1. Build
    ```bash
    colcon build
    ```