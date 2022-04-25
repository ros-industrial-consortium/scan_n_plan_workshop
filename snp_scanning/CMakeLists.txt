cmake_minimum_required(VERSION 3.5)
# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(ament_cmake REQUIRED)
find_package(open3d_interface_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in further dependencies manually. find_package(<dependency> REQUIRED)

add_executable(open3d_sim_node src/open3d_sim_node.cpp)
target_link_libraries(open3d_sim_node Boost::filesystem)
ament_target_dependencies(open3d_sim_node rclcpp open3d_interface_msgs)

install(TARGETS open3d_sim_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()