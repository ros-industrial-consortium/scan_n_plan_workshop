cmake_minimum_required(VERSION 3.5)
project(snp_motion_execution)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS scripts/robot_enable_simulator scripts/execution_simulator DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
