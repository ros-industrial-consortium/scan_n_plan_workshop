^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package snp_motion_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.1 (2024-05-29)
------------------

4.6.0 (2024-05-29)
------------------
* Merge pull request `#111 <https://github.com/marip8/scan_n_plan_workshop/issues/111>`_ from DavidSpielman/feature/rerun_toolpaths
  Added Freespace Motion Plan
* Pass joint state as const ref
* Clang formatting
* No longer remove scan link at end of freespace plan, matches process plan
* Make adding mesh optional to motion planning
* Rename process plan callback to be more descriptive
* Ran clang formatting
* Defining manipulator info via server request parameters
* Removed commented lines from planning server
* Fixed typo in planCallback method
* Freespace plan callback in working state. Prev issue: Passing nested composite instruction to freespace task when it expect composite instruction one layer deep
* WIP: planning server fails when freespace motion plan problem is run
* WIP. Update bt node to pass in joint waypoint instead of poly waypoint
* Added freespace motion planning service
* Contributors: David Spielman, Michael Ripperger, Tyler Marr

4.5.0 (2024-05-29)
------------------

4.4.1 (2024-05-23)
------------------
* Merge pull request `#105 <https://github.com/marip8/scan_n_plan_workshop/issues/105>`_ from DavidSpielman/pr/dspielman/update/planner-org
  Updated Planner Organization
* Removed call to remove scan link in planCallback method
* Modified signature of plan method
* Reverted name of raster task name to task name
* Ran clang formatting
* Reorganized planner_server.cpp to simplify motion plan generations
* WIP update organization of planner code
* Contributors: David Spielman, Michael Ripperger

4.4.0 (2024-05-21)
------------------
* Merge pull request `#107 <https://github.com/marip8/scan_n_plan_workshop/issues/107>`_ from marip8/update/rolling-ci
  Update `tf2_eigen` includes
* Added workaround for deprecated tf2_eigen header
* Contributors: Michael Ripperger

4.3.0 (2024-05-03)
------------------

4.2.0 (2024-04-15)
------------------

4.1.1 (2024-03-14)
------------------
* Update default empty list args as a workaround for ros2 launch issue
* Contributors: Michael Ripperger

4.1.0 (2024-03-13 18:12)
------------------------
* Fully support min distance to contact during planning (`#88 <https://github.com/marip8/scan_n_plan_workshop/issues/88>`_)
  * Changed contact check distance to minimum distance to contact; passed min contact distance to profiles to plan accordingly
  * Added reduced contact distance pair between scan and specifiable links to motion planning profiles
  * Added contact check profile creation to header file
  * Added comments on scan links parameters
  * Rename to ExplicitCollisionPair; added documentation comments; used contact check profile helper function
* Contributors: Michael Ripperger

4.0.0 (2024-03-13 16:00)
------------------------
* Expose all planning server parameters in launch file (`#87 <https://github.com/marip8/scan_n_plan_workshop/issues/87>`_)
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
* Contributors: Michael Ripperger

3.19.0 (2024-01-26)
-------------------
* Minor Updates (`#75 <https://github.com/marip8/scan_n_plan_workshop/issues/75>`_)
  * Remove invalid line
  * Revised creation of program to be more readable
  * Updated noether and boost_plugin_loader dependencies
* Merge pull request `#71 <https://github.com/marip8/scan_n_plan_workshop/issues/71>`_ from DavidSpielman/pr/check_empty_manip_info_params
  Added checks to alert user if the base frame, motion group and tcp fra…
* Throwing exceptions instead of warnings to the user. Ran clang formatting
* Added checks to alert user if the base frame, motion group or tcp frame parameters are empty prior to creating a manipulator info and program
* Contributors: David Spielman, Michael Ripperger, Tyler Marr

3.18.0 (2024-01-04 17:42)
-------------------------
* Noether Update (`#70 <https://github.com/marip8/scan_n_plan_workshop/issues/70>`_)
  * Updated dependencies for new noether version
  * Changed dependency on noether_filtering to noether_tpp
  * Updated docker files
  * Updated CI
  * Remove ToolPaths message
  * Updated error statements
  * Updated TPP service name
  * Updated TPP widget to use new configurable TPP pipeline widget
* Contributors: Michael Ripperger

3.17.0 (2024-01-04 09:30)
-------------------------

3.16.0 (2024-01-04 09:21)
-------------------------
* Updated to Tesseract 0.21 (`#67 <https://github.com/marip8/scan_n_plan_workshop/issues/67>`_)
* Contributors: Michael Ripperger

3.15.0 (2024-01-02)
-------------------
* Convert tcpSpeedLimiter to Tesseract Plugin Task (`#63 <https://github.com/marip8/scan_n_plan_workshop/issues/63>`_)
  * create tcpSpeedLimiter tesseract plugin task
  bugfix to tcp_speed_limiter_task
  clean up tcp_speed_limiter task and convert tcp_speed_limiter class to function
  set tcp_max_speed param as launch arg
  * update code for clang and cmake formatting
  update code clang format w/ ubuntu20 clang 10
  * move tcpSpeedLimiter function to header file
* Contributors: Yolnan

3.14.0 (2023-12-08)
-------------------

3.13.0 (2023-10-24)
-------------------
* Update to Tesseract 0.20 (`#56 <https://github.com/marip8/scan_n_plan_workshop/issues/56>`_)
  * Updated motion planning node to be compatible with tesseract 0.20
  * Updated the task composer config file
  * Updated CI config
* Contributors: Michael Ripperger

3.12.0 (2023-10-23)
-------------------
* Various Updates (`#55 <https://github.com/marip8/scan_n_plan_workshop/issues/55>`_)
  * Updated OMPL profile
  * Increased TF timeout for ROI selection mesh modifier
  * Make trajopt costs a single value to support motion groups with more than 6 DoF
  * Make motion group, tcp frame, and camera frame dynamic parameters in application
  * Leverage YAML anchors in task composer config
  * Update OMPL tasks
  * Added parameter for OMPL max planning time
* Contributors: Michael Ripperger

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
* Allow selectable representation for collision object (`#51 <https://github.com/marip8/scan_n_plan_workshop/issues/51>`_)
  * Represent scan mesh as octomap instead of convex hull
  * Changed addition of scan mesh to collision environment to utilize octomap instead of convex hull
  * Updated contact test type in motion planner profiles
  * Make scan mesh collision object type selectable
  * Add and remove scan mesh collision links directly to the environment to leverage visualization capability of monitor
  * Added check on octomap resolution
  * Added a service for manually removing scan link
* Merge pull request `#49 <https://github.com/marip8/scan_n_plan_workshop/issues/49>`_ from marip8/update/ci
  Remove unused variable from CI
* Contributors: Michael Ripperger, Tyler Marr

3.9.0 (2023-09-11 10:42)
------------------------

3.8.0 (2023-09-11 10:16)
------------------------
* Parameterized task composer config file and task name
* Contributors: Michael Ripperger

3.7.0 (2023-09-11 10:05)
------------------------
* Add string header
* Removed unused header
* Update to latest tesseract (`#22 <https://github.com/marip8/scan_n_plan_workshop/issues/22>`_)
  * Updated to tesseract 0.14.0
  * Updated RVIZ to using new Tesseract widgets
  * Clang formatting
  * Removed extra rclcpp Node that was unneeded
  * Set tag to 0.14.0
  * Updated to latest tesseract version
  * Working planner
  * Clang formatting
  * Switched to RRTConnect
  * Updated to be able to use custom pipelines
  * Clang formatting
  * Clean up
  * More clean up
  * Cmake format
  * Removed no longer used task setup variables
  * Removed old commented code
  * Minor cleanup
  * Remove now unneeded custom raster definitions
  * Update tesseract dependencies to right version of planning
  * Clang formatting
  * Running and planning with tesseract 0.16, but trajectory produced is wrong
  * Updated dependencies
  * Updated yaml file for plugins
  * Added saving dotgraph results
  * Updated to successfully build and work, no custom tasks yet
  * Currently working on latest tesseract branches
  * Working with latest tesseract on foxy as of 6-15-2023
  * Clang formatting
  * cmake formating
  * Fixed issue with constant speed task not storing output
  * Fixed minimum length for jerk smoothing
  * Rearranged kinematic limit check to be last
  * Added kin limit check to freespace and transition motions
  * Reverted a debugging message in constant tcp speed task
  * Deleted unused taskflow generators file
  * Remove unused things
  * Updated dependencies
  * Get rid of commented out linking
  * Added contact check profile with parameters, defaults to original default
  * Switched IK to KDL for now as that works
  * Updated to 0.18.3 tesseract planning
  * Switched to abort tasks instead of errors
  * Fixed planning server to respect scanned collision mesh
  * Updated base docker image to tesseract_ros2
  * Reset BEFORE_INIT because it was invalid from tesseract_ros2 docker
  * Added taskflow to dependencies
  * Updated workspace underlay and added humble and rolling builds
  * Fix ros distro docker name
  * Reset an environment variable used by tesseract_ros2 docker
  * Updated so tesseract doesn't publish tf
  * Updated to version of tesseract_qt that doesn't need qt_advanced_docking
* Contributors: Michael Ripperger, Tyler Marr

3.6.0 (2023-07-14)
------------------

3.5.0 (2023-06-05 17:23)
------------------------
* Constant TCP velocity time parameterization (`#28 <https://github.com/marip8/scan_n_plan_workshop/issues/28>`_)
  * Initial draft of constant velocity time parameterization
  * Added cartesian time param task flow generator
  * Comment out explicit use of Cartesian time parameterization
  * Changed creation of path
  * Updated acceleration calculation
  * Added rotational velocity parameters
  * Updated cartesian time param task name
  * Updated cartesian time parameterization
  * Added profile for Cartesian time parameterization
  * Added cartesian time parameterization profile to planning server
  * Ran cmake format
  * Updated class and file naming
  * Optionally check joint accelerations against limits
  * Dynamically load planning-related ROS parameters
  * Clamp velocity/acceleration scales on (0.0, 1.0]
  * Added task generator for kinematic limits check
  * Removed kinematic limits check from constant TCP time parameterization; added kinematic limits check task to raster taskflow; added kinematic limits profile to planning server
* Ensure mesh is convexified before adding to environment (`#29 <https://github.com/marip8/scan_n_plan_workshop/issues/29>`_)
* Contributors: Michael Ripperger

3.4.0 (2023-06-05 13:16)
------------------------

3.3.0 (2023-05-18)
------------------
* Integration - 5/17 (`#25 <https://github.com/marip8/scan_n_plan_workshop/issues/25>`_)
  * Separated simulated robot enable from simulated motion execution
  * Moved open3d mesh publisher to simulation node
  * Make motion execution node listen to full joint states topic
  * Updated Rviz config
  * Use Trajopt for raster process planner
* Integration Changes - 5.10.2023 (`#24 <https://github.com/marip8/scan_n_plan_workshop/issues/24>`_)
  * Added updated scan trajectory around the work table
  * Added TPP yaml file
  * Parameterized TSDF values
  * Updated TSDF parameters in launch files for blending and automate demo
  * Added calibration files
  * Fixed table calibration
  * Updated camera calibration
  * Updated pointcloud parameter names to the latest realsense nomenclature
  * Updated scan trajectory
  * Updated tpp configuration
  * Updated Pushcorp URDF and TCP location
  * Show output from motion planner node on screen
  ---------
  Co-authored-by: Chris Lewis <drchrislewis@gmail.com>
* Contributors: Michael Ripperger

3.2.0 (2023-05-10)
------------------

3.1.0 (2023-05-09)
------------------

3.0.0 (2022-09-01)
------------------
* Reorganize application-specific files (`#13 <https://github.com/marip8/scan_n_plan_workshop/issues/13>`_)
  * Combined support and bringup package into single application implementation package
  * Removed application-specific instructions from README; replace with general description
  * Reverted to update from https://github.com/ros-industrial-consortium/scan_n_plan_workshop/pull/9
  * Set planner verbose by default
  * Changed name of dependencies file
  * Updated documentation
  * Updated documentation per review
* Contributors: Michael Ripperger

2.0.0 (2022-08-10 09:16:43 -0500)
---------------------------------
* Automate 2022 Integration (`#5 <https://github.com/marip8/scan_n_plan_workshop/issues/5>`_)
  * Remove temporary erase of first and last raster
  * Converted application window to widget
  * Update planning functions to not be blocking
  * Add Rviz panel for SNP application
  * Renamed rosconwindow to snp_widget
  * Changed launch files to use rviz panel version of application
  * Added ROI selection mesh modifier and widget
  * Added noether plugin for ROI mesh modifier
  * Add TPP widget
  * Added TPP app
  * Updated launch file to start TPP app instead of node
  * Updated Rviz config
  * Remove TPP parameter from service definition; added string for mesh frame to TPP service defintion; updated existing TPP nodes
  * Transform selection into mesh frame
  * Changed namespace from snp to snp_tpp
  * Created unique names for transition commands
  * Async callback for motion execution
  * fixup tpp widget header
  * Faster scan traj
  * Automate setup camera calibration
  * Updated LVS to ensure at least 5 wps
  * Updated the rviz config file
  * Added collision geometry for TCP
  * Updated dependencies and README
  * Updated TPP to use latest version of noether_gui
  * Ran CMake format
  * Ran clang format
  * Replaced references to open3d_interface
  * Updated .repos files
  * Added xmlrpcpp dependency for CI
* Merge branch 'integration_devel_5-19' into 'master'
  Integration 5/20
  See merge request swri/ros-i/rosworld2021/roscon2021!61
* CLANG
* Updated taskflow to enforce a minimum number of waypoints for transitions and freespace
* Updated to planning profiles
* Switched to global descartes first
* WIP: testing tool speed control
* Merge branch 'feature/collision-check-against-scan' into 'master'
  Add scan to motion planning environment
  See merge request swri/ros-i/rosworld2021/roscon2021!56
* Add touch link parameters to launch files
* Revised addition of scan to environment
* Motion Planning: Adding scan to collision environment
* Merge branch 'update/launch-files' into 'master'
  Simplify loading of URDF/SRDF in XML launch files
  See merge request swri/ros-i/rosworld2021/roscon2021!52
* Simplify loading of URDF/SRDF in XML launch files
* Merge branch 'update/launch-files' into 'master'
  Convert launch files from Python to XML
  See merge request swri/ros-i/rosworld2021/roscon2021!49
* Merge branch 'feature/ikfast' into 'master'
  Added tesseract IKFast plugin
  See merge request swri/ros-i/rosworld2021/roscon2021!51
* Clang-format
* Update to 0.7.4 version of IKFast constructor; add code for extracting redundancy capable joints
* Parameterize number of joints for IKFast
* Added tesseract IKFast plugin for HC10
* Merge branch 'update/environment-monitor' into 'master'
  Environment monitor Update
  See merge request swri/ros-i/rosworld2021/roscon2021!48
* Changed python launch files to xml
* Initialize plotter after environment is initialized
* Start state monitor to sync environment with current robot state
* Merge branch 'feature/tcp-velocity-limiter' into 'master'
  Added function for limiting tcp velocity by scaling timestamps and velcoties/accelerations
  See merge request swri/ros-i/rosworld2021/roscon2021!45
* Addressed PR review
* clang formatting
* Added tesseract monitor and trajectory preview
* Fix clang formatting
* added function for limiting tcp velocity by scaling timestamps and velcoties/accelerations
* Merge branch 'update/clang-format' into 'master'
  Clang-format Update
  See merge request swri/ros-i/rosworld2021/roscon2021!46
* Update for clang-formatting
* Merge branch 'update/motion-planning' into 'master'
  Planning Server
  See merge request swri/ros-i/rosworld2021/roscon2021!40
* Remove using namespace
* Use reference for string
* fixup tool z free sample
* Clang-format
* Updated trajectory start state
* Added flag for verbose output; added log messages in planning server
* Rotated tool poses 180 degrees about x-axis
* Added additional profiles
* Added input and seed checks to custom taskflows
* Updated Descartes and TrajOpt profiles to have tool z free
* Added launch file for planning server
* Initial implementation of motion planning server
* Merge branch 'fix/app-service-calls' into 'master'
  Fix service calls in application
  See merge request swri/ros-i/rosworld2021/roscon2021!42
* Updated motion planning service name
* Merge branch 'update/move-planning-code' into 'master'
  Move planning function out of GUI
  See merge request swri/ros-i/rosworld2021/roscon2021!27
* Moved planning code from GUI to planning server
* Merge branch 'feature/motion-planning-node' into 'master'
  Added planning server node shell
  See merge request swri/ros-i/rosworld2021/roscon2021!25
* Use node-specific logger
* Used variable for ROS2 dependencies
* Added planning server node shell
* Contributors: David Merz, Jr, Michael Ripperger, Tyler Marr, ben, dmerz, mripperger, tmarr

1.0.0 (2021-10-19 16:56:56 +0000)
---------------------------------
