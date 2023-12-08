cmake_minimum_required(VERSION 3.5)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

# find dependencies
set(ROS2_DEPS industrial_reconstruction_msgs visualization_msgs rclpy)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
install(PROGRAMS scripts/reconstruction_sim_node DESTINATION lib/${PROJECT_NAME})

ament_package()
