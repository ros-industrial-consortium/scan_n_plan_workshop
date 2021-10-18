cmake_minimum_required(VERSION 3.5)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

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
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

add_executable(${PROJECT_NAME}_node src/snp_tpp_node.cpp)
target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION 14)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  noether::noether_tpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  snp_msgs
  tf2_eigen
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

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
