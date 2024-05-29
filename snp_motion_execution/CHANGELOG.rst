^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package snp_motion_execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.1 (2024-05-29)
------------------

4.6.0 (2024-05-29)
------------------

4.5.0 (2024-05-29)
------------------

4.4.1 (2024-05-23)
------------------

4.4.0 (2024-05-21)
------------------

4.3.0 (2024-05-03)
------------------

4.2.0 (2024-04-15)
------------------

4.1.1 (2024-03-14)
------------------
* fix controller manager spawner launch in humble (`#100 <https://github.com/marip8/scan_n_plan_workshop/issues/100>`_)
* Contributors: Yolnan

4.1.0 (2024-03-13 18:12)
------------------------

4.0.0 (2024-03-13 16:00)
------------------------
* Passed robot_description directly to ros2_control.launch.py (`#82 <https://github.com/marip8/scan_n_plan_workshop/issues/82>`_)
  Co-authored-by: David Spielman <david,spielman7@gmail.com>
* Behavior tree application with reactive GUI (`#77 <https://github.com/marip8/scan_n_plan_workshop/issues/77>`_)
  * Added dependency on BehaviorTree.CPP
  * Added BT utilities
  * Added BT thread class
  * Added initial GUI related BT nodes
  * Copied SequenceWithMemoryNode
  * Renamed SequenceWithMemoryNode class and file; changed namespace
  * Modifications to operation of custom sequence with memory node
  * Added SNP BT service nodes
  * Refactored SNP widget
  * Let behavior tree framework spin node
  * Put SNP panel in namespace
  * Removed robot enable from execution server
  * Added publisher BT nodes
  * Updated UI to have halt button
  * Updated widget to register publisher nodes and add halt button
  * Changed planning service name
  * Added FJT node
  * Updated timeouts
  * Added replace joint start state node
  * Updated names of stacked widget pages
  * Added TPP node widget to GUI
  * Added trajectory preview widget
  * Do not disable stacked widget on task failure
  * Added BT files
  * Create new node for BT operations; pass in Rviz node for other operations
  * Updated SNP widget layout
  * Changed motion planning services to return approach, process, and departure trajectories separately. Updated BT nodes to correspond
  * Use position only joint trajectory controller for simulation
  * Minor updates to widget
  * Removed dependency on YAML
  * Updated application launch files
  * Renamed file to snp_bt_ros_nodes
  * Updated dependencies to include BT ROS2
  * Remove deletion of thread
  * Ran clang format
  * Updated documentation for custom BT nodes
  * Added text editor BT logger
  * Incorporated text edit logger
  * Updated node names and descriptions
  * Added start button and updated behavior of reset button
  * Updated version of BT ROS2
  * Added start state replacement tolerance
  * Updated text edit logger to show same failure message for condition nodes as action nodes
  * Use blackboard entry to transmit error messages from ROS behavior nodes to the logger
  * Updated colcon-action to v6
  * Create node for TPP widget separate from BT node and Rviz node
  * Added condition node for spinning the BT ROS node to keep it alive for parameter updates
  * Removed motion execution server
  * Moved creation of BT factory into start method to allow dynamic changes of ROS params used to register nodes
* Contributors: David Spielman, Michael Ripperger

3.19.0 (2024-01-26)
-------------------

3.18.0 (2024-01-04 17:42)
-------------------------

3.17.0 (2024-01-04 09:30)
-------------------------

3.16.0 (2024-01-04 09:21)
-------------------------

3.15.0 (2024-01-02)
-------------------

3.14.0 (2023-12-08)
-------------------
* Python snp motion execution (`#61 <https://github.com/marip8/scan_n_plan_workshop/issues/61>`_)
  * add python motion exec scripts
  * add sleeps to motion exec async calls
  * add thread blocking to async calls
  * update dependencies
  * remove cpp motion exec and python debug printouts
  * fix Cmake formatting
  * Renamed scripts
  * Minor formatting changes
  * Revised use of mutex in execution simulator; minor formatting changes
  * Fixed ros2_control launch file by providing ros_distro argument to xacro
  ---------
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
* Contributors: Yolnan

3.13.0 (2023-10-24)
-------------------

3.12.0 (2023-10-23)
-------------------

3.11.0 (2023-09-28)
-------------------
* Humble build (`#52 <https://github.com/marip8/scan_n_plan_workshop/issues/52>`_)
  * Fix templates on declaring parameters
  * Fix runtime errors
  * Update noether for humble build
  * add vtk to rosdep skip
  * Updated dependencies to be by ROS version
  * Change skip key to just 'libvtk'
  * Check ROS version for how params are declared
  * Added ROS version check for tf2_eigen include
  * Added tf2_eigen depends to various packages
  * Added tf2_eigen to planning package CMakeLists
  * Fixed a templated declare param previously missed
  * Clang formatting
  * Clang formatting (pt 2)
  * Cleaner ROS version checking
  * Set C++ version to 17 for `__has_include` macro
  * Removed #if's from different ROS versions
  * Fix where things built, but didn't work in humble
  * Fix load_yaml difference between foxy and humble+
  * Clang formatting
  * Update snp_blending support to work with humble
* Contributors: Tyler Marr

3.10.0 (2023-09-20)
-------------------

3.9.0 (2023-09-11 10:42)
------------------------

3.8.0 (2023-09-11 10:16)
------------------------

3.7.0 (2023-09-11 10:05)
------------------------

3.6.0 (2023-07-14)
------------------

3.5.0 (2023-06-05 17:23)
------------------------

3.4.0 (2023-06-05 13:16)
------------------------
* ROS Control (`#27 <https://github.com/marip8/scan_n_plan_workshop/issues/27>`_)
  * Added ros2 control code
  * Remap joint state gui to new topic
  * Parameterized ros2 control launch files
  * Added ros2_control deps to package.xml
  * Generalized ros2_control launch file and moved to snp_execution
* Contributors: Michael Ripperger

3.3.0 (2023-05-18)
------------------
* Integration - 5/17 (`#25 <https://github.com/marip8/scan_n_plan_workshop/issues/25>`_)
  * Separated simulated robot enable from simulated motion execution
  * Moved open3d mesh publisher to simulation node
  * Make motion execution node listen to full joint states topic
  * Updated Rviz config
  * Use Trajopt for raster process planner
* Contributors: Michael Ripperger

3.2.0 (2023-05-10)
------------------

3.1.0 (2023-05-09)
------------------

3.0.0 (2022-09-01)
------------------

2.0.0 (2022-08-10 09:16:43 -0500)
---------------------------------
* Merge branch 'integration_devel_5-19' into 'master'
  Integration devel 5 19
  See merge request swri/ros-i/rosworld2021/roscon2021!59
* Clang formatting
* Fixed bug with start state replacement and handling timeout
* Merge branch 'update/robot-motion' into 'master'
  Motion execution update
  See merge request swri/ros-i/rosworld2021/roscon2021!55
* Make motion execution node thread-safe; increase joint state time threshold
* Merge branch 'feature/robot-motion' into 'master'
  Motion execution update
  See merge request swri/ros-i/rosworld2021/roscon2021!54
* Remove print statements; add case to switch statement checking action result
* run clang formatting
* merge changes from working branch
* Added current state as start state of trajectory
* remove unused code
* motion works!
* almost able to move robot, wrong start state
* Merge branch 'add/motion_ex_node' into 'master'
  Motion Execution Node
  See merge request swri/ros-i/rosworld2021/roscon2021!41
* Renamed ROS comm objects
* Renamed motion execution nodes
* Revised motion execution node
* removed comments, fixed motion exec callback, addressed merge request threads
* ran clang & cmake
* exec node integrated
* fixed cmake to build foxy, still builds with warnings
* ran cmake & clang
* added motion execution handler, required edits to launch & application files
  precursor work for exec node dev, unbuilt, no clang/cmake
* Merge branch 'update/motion-execution' into 'master'
  Motion execution code clean up
  See merge request swri/ros-i/rosworld2021/roscon2021!36
* Comment unused arguments
* Merge branch 'remove/exec_node' into 'master'
  generated dummy nodes for robot_enable and follow_joint_trajectories
  See merge request swri/ros-i/rosworld2021/roscon2021!31
* ran clang & cmake
* fix package.xml
* generated dummy nodes for robot_enable and follow_joint_trajectories
* Merge branch 'design/define-message-types' into 'master'
  Define Service Types & Add Block Diagram
  See merge request swri/ros-i/rosworld2021/roscon2021!29
* PR Comments
* Merge branch 'update/repository-layout' into 'master'
  Repository layout update
  See merge request swri/ros-i/rosworld2021/roscon2021!22
* Moved ROS2 packages to top-level directory
* Contributors: David Merz, Jr, LCBW, Michael Ripperger, ben, mripperger

1.0.0 (2021-10-19 16:56:56 +0000)
---------------------------------
