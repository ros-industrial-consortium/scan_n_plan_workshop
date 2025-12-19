cmake_minimum_required(VERSION 3.5)
project(snp_application)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(ROS2_DEPS
    behaviortree_cpp
    behaviortree_ros2
    control_msgs
    Eigen3
    geometry_msgs
    industrial_reconstruction_msgs
    noether_gui
    noether_ros
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_components
    rviz_common
    sensor_msgs
    snp_msgs
    std_srvs
    trajectory_msgs
    trajectory_preview
    visualization_msgs)

find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}_bt_plugins SHARED
  src/bt/button_approval_node.cpp
  src/bt/button_monitor_node.cpp
  src/bt/create_joint_state_message.cpp
  src/bt/extract_approach_process_departure_trajectories_node.cpp
  src/bt/load_trajectory_from_file_node.cpp
  src/bt/progress_decorator_node.cpp
  src/bt/set_page_decorator_node.cpp
  src/bt/snp_bt_ros_nodes.cpp
  src/bt/snp_sequence_with_memory_node.cpp
  src/bt/plugins.cpp)
target_compile_definitions(${PROJECT_NAME}_bt_plugins PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(${PROJECT_NAME}_bt_plugins PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                             "$<INSTALL_INTERFACE:include>" ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_bt_plugins Qt5::Widgets)
ament_target_dependencies(
  ${PROJECT_NAME}_bt_plugins
  behaviortree_cpp
  behaviortree_ros2
  control_msgs
  geometry_msgs
  industrial_reconstruction_msgs
  noether_ros
  rclcpp_action
  sensor_msgs
  snp_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs)

# Widget
add_library(
  ${PROJECT_NAME}_widget SHARED
  src/snp_behavior_tree.cpp
  src/snp_widget.ui
  src/snp_widget.cpp
  # BT
  src/bt/bt_thread.cpp
  src/bt/text_edit_logger.cpp)
target_include_directories(
  ${PROJECT_NAME}_widget
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
         "$<INSTALL_INTERFACE:include>"
         ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_widget Qt5::Widgets noether::noether_gui)
ament_target_dependencies(
  ${PROJECT_NAME}_widget
  behaviortree_cpp
  behaviortree_ros2
  sensor_msgs
  trajectory_preview)

add_library(${PROJECT_NAME}_panel SHARED src/snp_panel.cpp)
target_link_libraries(${PROJECT_NAME}_panel ${PROJECT_NAME}_widget)
ament_target_dependencies(${PROJECT_NAME}_panel rviz_common pluginlib)

add_executable(roscon_app src/main.cpp)
target_link_libraries(roscon_app Qt5::Widgets ${PROJECT_NAME}_widget)
ament_target_dependencies(roscon_app rclcpp)

# Install the headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include/)

# Install the library(ies)
install(TARGETS ${PROJECT_NAME}_bt_plugins ${PROJECT_NAME}_widget ${PROJECT_NAME}_panel EXPORT ${PROJECT_NAME}-targets
        DESTINATION lib)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

install(TARGETS roscon_app DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_export_dependencies(${ROS2_DEPS})
ament_package()
