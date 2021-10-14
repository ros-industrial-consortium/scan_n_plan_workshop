cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  tesseract_monitoring
  tesseract_msgs
  tesseract_planning_server
)

find_package(Eigen3 REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    tesseract_msgs
    tesseract_monitoring
    tesseract_planning_server
  DEPENDS
    tesseract_command_language
    tesseract_process_managers
    tesseract_planning_server
)

add_executable(${PROJECT_NAME}_planning_server_node src/snp_planning_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_planning_server_node PRIVATE
  tesseract::tesseract_collision_core
  tesseract::tesseract_command_language
  tesseract::tesseract_process_managers
  ${catkin_LIBRARIES})
target_cxx_version(${PROJECT_NAME}_planning_server_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_planning_server_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_planning_server_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_planning_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

