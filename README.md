# Scan 'N Plan Workshop

Framework for developing and operating simple Scan 'N Plan applications in which a robot:
- Executes a scan path with a depth camera
- Reconstructs the surface of the objects observed by the camera
- Plans a tool path on the reconstructed surface
- Plans robot motions to execute the tool path
- Executes the process path

## Build Setup

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
    rosdep install --from-paths src --ignore-src -r -y
    ```

1. Follow the ROS2 build setup instructions for the application-specific implementations
    - Alternatively, add `COLCON_IGNORE` files to the application-specific implementation packages and skip their builds

## Build

```bash
colcon build --cmake-args -DTESSERACT_BUILD_FCL=OFF
```

## Application-specific Implementations
- [Automate 2022](snp_automate_2022)
