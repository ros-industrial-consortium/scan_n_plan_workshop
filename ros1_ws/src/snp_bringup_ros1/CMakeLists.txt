cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED)

catkin_package()
#catkin_package(
#  CATKIN_DEPENDS
#    twc_msgs
#    tesseract_msgs
#    actionlib
#    tesseract_rosutils
#  DEPENDS
#    tesseract_motion_planners
#    tesseract_visualization
#    tesseract_command_language
#    tesseract_environment
#)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

