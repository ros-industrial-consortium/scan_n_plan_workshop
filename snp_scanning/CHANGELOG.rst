^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package snp_scanning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#117 <https://github.com/marip8/scan_n_plan_workshop/issues/117>`_ from marip8/update/snp-scanning
  SNP scanning update
* Use non-deprecated names in setup.cfg
* Specify no Python packages in setup.py for snp_scanning
* Merge pull request `#113 <https://github.com/marip8/scan_n_plan_workshop/issues/113>`_ from marip8/update/snp-scanning
  SNP Scanning Update
* Added dependency on Open3d
* Converted snp_scanning to python package
* Contributors: Michael Ripperger

4.3.0 (2024-05-03)
------------------

4.2.0 (2024-04-15)
------------------

4.1.1 (2024-03-14)
------------------

4.1.0 (2024-03-13 18:12)
------------------------

4.0.0 (2024-03-13 16:00)
------------------------
* Fixed issue with departure being length 1 with and with bad timestamps (`#85 <https://github.com/marip8/scan_n_plan_workshop/issues/85>`_)
  * Fixed issue with departure being length 1 with and with bad timestamps
  Author:    Tyler Marr <tylermarr17@gmail.com>
  Date:      Fri Mar 1 14:46:30 2024 -0500
  * Fixed missing pip upgrade of numpy bc of open3d issue
  Author:    Tyler Marr <tylermarr17@gmail.com>
  Date:      Fri Mar 1 14:49:28 2024 -0500
  * Use -2 index to get second to last point
  ---------
  Co-authored-by: David Spielman <david.spielman@swri.org>
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
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
* Contributors: Michael Ripperger, Tyler Marr

3.19.0 (2024-01-26)
-------------------
* Minor Updates (`#75 <https://github.com/marip8/scan_n_plan_workshop/issues/75>`_)
  * Remove invalid line
  * Revised creation of program to be more readable
  * Updated noether and boost_plugin_loader dependencies
* Contributors: Michael Ripperger

3.18.0 (2024-01-04 17:42)
-------------------------

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
* Convert `snp_scanning` node to Python (`#64 <https://github.com/marip8/scan_n_plan_workshop/issues/64>`_)
  * Updated industrial_reconstruction version
  * Replace snp_scanning c++ node with Python node
  * Updated Automate 2022 implementation
  * Update SNP blending implementation
* Contributors: Michael Ripperger

3.13.0 (2023-10-24)
-------------------

3.12.0 (2023-10-23)
-------------------

3.11.0 (2023-09-28)
-------------------

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
* Merge branch 'feature/robot-motion' into 'master'
  Motion execution update
  See merge request swri/ros-i/rosworld2021/roscon2021!54
* almost able to move robot, wrong start state
* Merge branch 'add/open3d_sim' into 'master'
  Add/open3d sim
  See merge request swri/ros-i/rosworld2021/roscon2021!38
* Open3D sim node clean up
* fix cmake, fix header, add filepath param & try-catch, add sim params & groups to launch
* ran clang
* added open3d sim node
* added 3d_sim shell & launch edits
* Contributors: LCBW, Michael Ripperger, ben, mripperger

1.0.0 (2021-10-19 16:56:56 +0000)
---------------------------------
