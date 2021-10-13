cmake_minimum_required(VERSION 3.5)
project(snp_tpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snp_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/snp_tpp_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
    include
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp snp_msgs)
#rosidl_target_interfaces(${PROJECT_NAME}_node "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
