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
find_package(geometry_msgs REQUIRED)
find_package(noether_tpp REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(${PROJECT_NAME}_node src/snp_tpp_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  snp_msgs
  tf2_eigen
)
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  noether::noether_tpp
)

add_executable(${PROJECT_NAME}_test src/snp_tpp_test_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_test
  geometry_msgs
  rclcpp
  snp_msgs
)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
