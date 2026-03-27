cmake_minimum_required(VERSION 3.5)
project(snp_simulation)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS scripts/reconstruction_sim_node scripts/robot_enable_simulator scripts/execution_simulator
        DESTINATION lib/${PROJECT_NAME})

ament_package()
