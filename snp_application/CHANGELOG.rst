^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package snp_application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.1 (2024-05-29)
------------------
* Merge pull request `#121 <https://github.com/marip8/scan_n_plan_workshop/issues/121>`_ from marip8/fix/bt-node-types
  Changed custom nodes from control to action nodes
* Changed custom nodes from control to action nodes
* Contributors: Michael Ripperger

4.6.0 (2024-05-29)
------------------
* Merge pull request `#111 <https://github.com/marip8/scan_n_plan_workshop/issues/111>`_ from DavidSpielman/feature/rerun_toolpaths
  Added Freespace Motion Plan
* Catch exceptions in connect functions
* Rework updateStartState to use passed in joint state
* Change parameter name for consistency
* Give Qwarning instead of throwing error to prevent crashing
* set joint state output in 1 line
* Remove extra semicolons
* pass trajectory points by const ref
* Clang formatting
* Add Go Home button
* Add freespace go home capability to BT
* Switch BT generate freespace to use joint state inputs
* Remove FreespaceMotionPlanPub
* Ran clang formatting
* Removed outdated BT node to publish freespace motion trajectory
* Defining manipulator info via server request parameters
* Freespace plan callback in working state. Prev issue: Passing nested composite instruction to freespace task when it expect composite instruction one layer deep
* WIP. Update bt node to pass in joint waypoint instead of poly waypoint
* Contributors: David Spielman, Michael Ripperger, Tyler Marr

4.5.0 (2024-05-29)
------------------
* Merge pull request `#119 <https://github.com/marip8/scan_n_plan_workshop/issues/119>`_ from marip8/update/tpp-bt-behavior
* Updated behavior tree to revert to TPP node when TPP config button is pushed
* Contributors: Michael Ripperger

4.4.1 (2024-05-23)
------------------

4.4.0 (2024-05-21)
------------------
* Merge pull request `#107 <https://github.com/marip8/scan_n_plan_workshop/issues/107>`_ from marip8/update/rolling-ci
  Update `tf2_eigen` includes
* Added control_msgs dependency to snp_application
* Contributors: Michael Ripperger

4.3.0 (2024-05-03)
------------------
* Merge pull request `#109 <https://github.com/marip8/scan_n_plan_workshop/issues/109>`_ from DavidSpielman/pr/feature/reformat_motion_plan_pub_bt
  Reformatted motion plan pub BT node to receive one input trajectory
* Ran clang formatting
* Updated BT to incorporate combine trajectories node and reformatted motion pub BT node
* Reformatted motion plan pub node to publish one input trajectory
* Merge pull request `#108 <https://github.com/marip8/scan_n_plan_workshop/issues/108>`_ from DavidSpielman/feature/combine_traj_bt
  Added BT node that combines two trajectories
* Updated how overlapping trajectories are processed
* Removed first point of second trajectory from being combined to trajectories
* Renamed class name and members for clarity
* Added bt node that combines two input trajectories and outputs one trajectory
* Merge pull request `#106 <https://github.com/marip8/scan_n_plan_workshop/issues/106>`_ from DavidSpielman/feature/reverse_trajectory_bt
  Added BT node to reverse a given input trajectory
* Ran clang formatting
* Added BT node to reverse a given input trajectory
* Contributors: David Spielman, Michael Ripperger

4.2.0 (2024-04-15)
------------------
* Formatting updates
* Ran clang formatting
* Exposing stacked widget object instead of widget
* Created methods that expose textedit and stacked widget objects
* Contributors: David Spielman

4.1.1 (2024-03-14)
------------------

4.1.0 (2024-03-13 18:12)
------------------------

4.0.0 (2024-03-13 16:00)
------------------------
* add fjt action param to snp_application (`#99 <https://github.com/marip8/scan_n_plan_workshop/issues/99>`_)
  * add fjt action param to snp_application
  * update snp_widget for clang formatting
* Added parameters for StopReconstruction service arguments (`#92 <https://github.com/marip8/scan_n_plan_workshop/issues/92>`_)
  * Added parameters for StopReconstruction service arguments
  * Addressed review comments
  * Handle exception as error
* Allow customization in BT factory (`#86 <https://github.com/marip8/scan_n_plan_workshop/issues/86>`_)
  * Created virtual function in SNP widget for configuring BT factory such that inherited classes can add additional custom BT nodes
  * Changed section from private to protected
  * Export dependencies of snp_application
  * Nest headers in snp_application subdirectory
  * Remove unnecessary directory include
  * Removed unnecessary Q_OBJECT macros
  * Updated BT configuration method to include ROS timeouts
  * Moved template functions to header for use by other packages
* Add rel and tracking frame and live to recon request plus and declare them (`#84 <https://github.com/marip8/scan_n_plan_workshop/issues/84>`_)
* Fixed error that was always setting start state to zeros (`#81 <https://github.com/marip8/scan_n_plan_workshop/issues/81>`_)
* Minor updates (`#80 <https://github.com/marip8/scan_n_plan_workshop/issues/80>`_)
  * Make TPP widget non-modal so the load and save dialogs work correctly
  * Convert angles from degrees to radians for SNP raster planner
* Check for required params in snp_widget that default to empty (`#79 <https://github.com/marip8/scan_n_plan_workshop/issues/79>`_)
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
* Contributors: Michael Ripperger, Tyler Marr, Yolnan

3.19.0 (2024-01-26)
-------------------

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
* Created service for generating a motion plan for scan trajectories (`#66 <https://github.com/marip8/scan_n_plan_workshop/issues/66>`_)
  * Created a service for generating a motion plan for scan trajectories. Modified the snp_widget to utilize this service to read values from a YAML file and return a joint trajectory message as a response.
  * Minor formatting changes
  * Moved scan_trajectory_motion_plan_server_node to snp_scanning package
  * Renamed service
  * Added boolean success flag and string message to service definition
  * Revised applcation to provide and check success of scan motion planning service
  ---------
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
* Contributors: David Spielman

3.15.0 (2024-01-02)
-------------------

3.14.0 (2023-12-08)
-------------------
* add default rosparam vals for snp_widget (`#65 <https://github.com/marip8/scan_n_plan_workshop/issues/65>`_)
* Contributors: Yolnan

3.13.0 (2023-10-24)
-------------------

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
* Merge branch 'feature/gui-update' into 'master'
  GUI Flow Update
  See merge request swri/ros-i/rosworld2021/roscon2021!63
* Updated checks on data; re-routed logging to text edit instead of ROS log
* enable all process buttons and check existence of data
* remove commented code, change onUpdateStatus signature
* update flow of buttons
* Merge branch 'integration_devel_5-19' into 'master'
  Integration 5/20
  See merge request swri/ros-i/rosworld2021/roscon2021!61
* Updated open3d to do filtering
* Updated open3d params
* Merge branch 'feature/collision-check-against-scan' into 'master'
  Add scan to motion planning environment
  See merge request swri/ros-i/rosworld2021/roscon2021!56
* Revised addition of scan to environment
* Add Mesh to Motion Planning Service
* Merge branch 'integration_devel_5-19' into 'master'
  Integration devel 5 19
  See merge request swri/ros-i/rosworld2021/roscon2021!59
* Clang formatting
* Updated method for handling fixed trajectory, updated some open3d params
* Merge branch 'feature/robot-motion' into 'master'
  Motion execution update
  See merge request swri/ros-i/rosworld2021/roscon2021!54
* Updates to application
* Removed checks on number of parameters
* run clang formatting
* load scan traj from yaml
* merge changes from working branch
* Remove joint state sub from main application
* remove unused code
* motion works!
* fix joint_state callback
* almost able to move robot, wrong start state
* Merge branch 'update/motion-planning' into 'master'
  Planning Server
  See merge request swri/ros-i/rosworld2021/roscon2021!40
* Renamed declare and get function
* Move reset of motion plan
* Remove motion plan future typedef
* Fix placement of override cursor
* Add try-catch to application main function
* Minor CMake changes
* Added signal handler to quit Qt application
* Check motion planning server existence; override cursor to indicate planning
* Add base frame to tool path
* Update application to call motion planning service; purged tesseract dependencies; parameterized frame names
* Merge branch 'add/motion_ex_node' into 'master'
  Motion Execution Node
  See merge request swri/ros-i/rosworld2021/roscon2021!41
* Added error messages from motion execution server
* Minor updates to application
* removed comments, fixed motion exec callback, addressed merge request threads
* ran clang & cmake
* exec node integrated
* added motion execution handler, required edits to launch & application files
  precursor work for exec node dev, unbuilt, no clang/cmake
* Merge branch 'fix/app-service-calls' into 'master'
  Fix service calls in application
  See merge request swri/ros-i/rosworld2021/roscon2021!42
* Ran clang format
* Updated error messaging from services
* Updated calibration service callbacks
* Fix calls to services in application
* Updated motion planning service name
* Merge branch 'update/gui' into 'master'
  GUI update
  See merge request swri/ros-i/rosworld2021/roscon2021!37
* Update GUI state tracking to work with action/service callbacks
* Merge branch 'edit/application' into 'master'
  Added placeholder for automated scan execution
  See merge request swri/ros-i/rosworld2021/roscon2021!35
* Merge branch 'edit/application' into 'edit/application'
  Scan Motion Execution Updates
  See merge request lbayewallace/roscon2021!2
* Added asynchronous spinner to main application
* Added method to get node from widget
* Remove unneeded destructor
* Use modern signal/slot connection; code clean-up
* Fixed callback chain to include reconstruction triggers; minor clean up
* added motion execution for scanning process, ran clang & cmake
* Merge branch 'update-open3d-stop-reconstruction' into 'master'
  Updated open3d to specify mesh file name rather than predefined name in a specified directory
  See merge request swri/ros-i/rosworld2021/roscon2021!34
* Updated open3d to specify mesh file name rather than predefined name in a specified directory
* Merge branch 'refactor/change-execution-service-call' into 'master'
  Changing to new Motion Execution Call
  See merge request swri/ros-i/rosworld2021/roscon2021!32
* Apply 2 suggestion(s) to 1 file(s)
* Changing to new Motion Execution Call
* Merge branch 'design/define-message-types' into 'master'
  Define Service Types & Add Block Diagram
  See merge request swri/ros-i/rosworld2021/roscon2021!29
* PR Comments
* Merge branch 'update/move-planning-code' into 'master'
  Move planning function out of GUI
  See merge request swri/ros-i/rosworld2021/roscon2021!27
* Moved planning code from GUI to planning server
* Merge branch 'fix/package-name' into 'master'
  Fixed name of snp_application directory
  See merge request swri/ros-i/rosworld2021/roscon2021!28
* Fixed name of snp_application directory
* Contributors: David Merz, Jr, LCBW, Michael Ripperger, Tyler Marr, ben, dmerz, lbayewallace, mripperger, tmarr

1.0.0 (2021-10-19 16:56:56 +0000)
---------------------------------
