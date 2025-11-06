find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)
ament_add_gtest(test_bt test_bt.cpp)
target_link_libraries(test_bt Qt5::Widgets)
ament_target_dependencies(
  test_bt
  ament_index_cpp
  behaviortree_cpp
  behaviortree_ros2)
target_compile_definitions(test_bt PRIVATE BT_PLUGIN_LIB="${CMAKE_INSTALL_PREFIX}/lib/lib${PROJECT_NAME}_bt_plugins.so")
