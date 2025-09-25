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
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(noether_tpp REQUIRED)
find_package(noether_gui REQUIRED)
find_package(PCL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_polygon_selection_tool REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Custom mesh modifier
add_library(
  ${PROJECT_NAME} SHARED
  src/roi_selection_mesh_modifier.cpp
  src/roi_selection_mesh_modifier_widget.cpp
  src/snp_raster_planner_widget.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} noether::noether_tpp noether::noether_gui)
ament_target_dependencies(
  ${PROJECT_NAME}
  geometry_msgs
  rviz_polygon_selection_tool
  rclcpp
  snp_msgs
  tf2_eigen)

# Noether plugin libraries
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})

add_library(${PROJECT_NAME}_widget_plugins SHARED src/widget_plugins.cpp)
target_link_libraries(${PROJECT_NAME}_widget_plugins ${PROJECT_NAME})

# Install the headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install the library(ies)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins ${PROJECT_NAME}_widget_plugins EXPORT ${PROJECT_NAME}-targets
        DESTINATION lib)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  noether_tpp
  noether_gui
  PCL
  geometry_msgs
  rviz_polygon_selection_tool
  rclcpp
  snp_msgs
  tf2_eigen)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/noether_plugin_libs.dsv.in")
ament_package()
