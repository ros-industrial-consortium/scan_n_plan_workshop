cmake_minimum_required(VERSION 3.5.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX C)

find_package(ament_cmake REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating
                                          # targets
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(yaml-cpp REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(
  ${PROJECT_NAME} SHARED
  # src/plotting.cpp
  src/conversions.cpp src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC tesseract::tesseract_environment
         tesseract::tesseract_scene_graph
         tesseract::tesseract_srdf
         tesseract::tesseract_geometry
         tesseract::tesseract_collision_core
         tesseract::tesseract_visualization
         tesseract::tesseract_common
         tesseract::tesseract_motion_planners_core
         tesseract::tesseract_process_managers
         tesseract::tesseract_command_language
         yaml-cpp
         ${tf2_eigen_LIBRARIES}
         ${sensor_msgs_LIBRARIES}
         ${std_msgs_LIBRARIES}
         ${geometry_msgs_LIBRARIES}
         ${tesseract_msgs_LIBRARIES}
         ${rclcpp_LIBRARIES}
         ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(
  ${PROJECT_NAME} SYSTEM
  PUBLIC ${tf2_eigen_INCLUDE_DIRS}
         ${sensor_msgs_INCLUDE_DIRS}
         ${std_msgs_INCLUDE_DIRS}
         ${geometry_msgs_INCLUDE_DIRS}
         ${tesseract_msgs_INCLUDE_DIRS}
         ${rclcpp_INCLUDE_DIRS}
         ${EIGEN3_INCLUDE_DIRS}
         ${Boost_INCLUDE_DIRS})

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # find_package(GTest REQUIRED) find_package(tesseract_support REQUIRED) catkin_add_gtest(${PROJECT_NAME}_unit
  # test/tesseract_rosutils_unit.cpp) target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main
  # tesseract::tesseract_support tesseract::tesseract_scene_graph tesseract::tesseract_geometry
  # tesseract::tesseract_collision_core tesseract::tesseract_visualization tesseract::tesseract_common
  # tesseract::tesseract_motion_planners_core ${PROJECT_NAME} ${catkin_LIBRARIES})
  # target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  # target_clang_tidy(${PROJECT_NAME}_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  # target_cxx_version(${PROJECT_NAME}_unit PUBLIC VERSION 17) target_include_directories(${PROJECT_NAME}_unit PRIVATE
  # "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  # "$<INSTALL_INTERFACE:include>")
  # target_include_directories(${PROJECT_NAME}_unit SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
endif()
