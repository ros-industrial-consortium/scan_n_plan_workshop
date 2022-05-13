cmake_minimum_required(VERSION 3.5)
project(snp_application)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(open3d_interface_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(
  roscon_app
  src/rosconwindow.ui
  include/rosconwindow.h
  src/rosconwindow.cpp
  src/main.cpp)
target_link_libraries(roscon_app Qt5::Widgets)
ament_target_dependencies(
  roscon_app
  open3d_interface_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  snp_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs)

install(TARGETS roscon_app DESTINATION lib/${PROJECT_NAME}/)

ament_package()
