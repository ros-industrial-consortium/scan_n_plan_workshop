cmake_minimum_required(VERSION 3.5)
project(snp_motion_execution)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(snp_msgs REQUIRED)

# uncomment the following section in order to fill in further dependencies manually. find_package(<dependency> REQUIRED)

add_executable(exec_sim_node src/exec_sim_node.cpp)
ament_target_dependencies(
  exec_sim_node
  rclcpp
  std_srvs
  rclcpp_action
  control_msgs
  snp_msgs)

add_executable(execute_motion_plan src/motion_ex_node.cpp)
ament_target_dependencies(
  execute_motion_plan
  rclcpp
  std_srvs
  rclcpp_action
  control_msgs
  snp_msgs)

install(TARGETS exec_sim_node execute_motion_plan DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
