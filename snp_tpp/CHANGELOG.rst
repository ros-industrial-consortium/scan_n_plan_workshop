^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package snp_tpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.6.1 (2024-05-29)
------------------

4.6.0 (2024-05-29)
------------------
* Merge pull request `#114 <https://github.com/marip8/scan_n_plan_workshop/issues/114>`_ from BryanMqz/global-branch
  Scan N' Plan update that complies with multiple polygon selection tool
* Ran clang-format
* Revised selection tool integration changes
* Fixed double point generation in rviz_polygon_selection_tool so this can be set back
* Header include update
* Scan N' Plan update that complies with multiple polygon selection tool
* Contributors: BryanMqz, Michael Ripperger

4.5.0 (2024-05-29)
------------------

4.4.1 (2024-05-23)
------------------

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

4.1.0 (2024-03-13 18:12)
------------------------

4.0.0 (2024-03-13 16:00)
------------------------
* Minor updates (`#80 <https://github.com/marip8/scan_n_plan_workshop/issues/80>`_)
  * Make TPP widget non-modal so the load and save dialogs work correctly
  * Convert angles from degrees to radians for SNP raster planner
* Added Simplified Raster Planner  (`#76 <https://github.com/marip8/scan_n_plan_workshop/issues/76>`_)
  * WIP adding simple raster planner gui widget for snp. Need to update config and save methods to configure from a yaml file and save to a yaml file
  * Added functionality to configure/save the simplified raster planner gui from/to a yaml file
  * Added units to the labels of the tpp parameters in the gui. Removed commented code from header file.
* Contributors: David Spielman, Michael Ripperger

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

3.15.0 (2024-01-02)
-------------------

3.14.0 (2023-12-08)
-------------------

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
* Update YAML includes for TPP GUI widgets (`#37 <https://github.com/marip8/scan_n_plan_workshop/issues/37>`_)
  * Added include of YAML to TPP GUI widgets
  * Updated noether dependency
* Contributors: Michael Ripperger

3.5.0 (2023-06-05 17:23)
------------------------

3.4.0 (2023-06-05 13:16)
------------------------

3.3.0 (2023-05-18)
------------------
* TPP Update (`#26 <https://github.com/marip8/scan_n_plan_workshop/issues/26>`_)
  * Moved creation and configuration of plugin loader outside of TPP widget
  * Removed scroll area from TPP widget
  * Added library and search path environment variables to plugin loader
  * Updated noether dependency
* Include tf2_ros/buffer in roi_selection_mesh_modifier header (`#23 <https://github.com/marip8/scan_n_plan_workshop/issues/23>`_)
* Contributors: Michael Ripperger, srsidd

3.2.0 (2023-05-10)
------------------
* TPP Update (`#21 <https://github.com/marip8/scan_n_plan_workshop/issues/21>`_)
  * Updated to later version of noether
  * Separated ROI selection mesh modifier from widget
  * Added YAML load/save to ROI selection mesh modifier widget
  * Updated SNP TPP widget to load configuration from file specified as a parameter of the node
  * Added tool path config file to snp_automate_2022
  * Remove old TPP nodes
  * Exposed header files for use by dependent projects
  * Updated dependency on rviz_polygon_selection_tool
* Contributors: Michael Ripperger

3.1.0 (2023-05-09)
------------------
* Fix: Build Errors (`#15 <https://github.com/marip8/scan_n_plan_workshop/issues/15>`_)
  - use correct ros_industrial_cmake_boilerplate version in dependencies_tesseract.repo
  - add rviz_polygon_selection_tool to snp_tpp's package.xml
  Co-authored-by: David Merz, Jr <david.merz@swri.org>
* Contributors: DavidMerzJr

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
* Merge branch 'update/tpp' into 'master'
  TPP Update
  See merge request swri/ros-i/rosworld2021/roscon2021!57
* Update the TPP to have snake style organization
* Merge branch 'update/motion-planning' into 'master'
  Planning Server
  See merge request swri/ros-i/rosworld2021/roscon2021!40
* Normalized tool path orientations
* Merge branch 'fix/build' into 'master'
  Get packages building with newest tesseract_ros2
  See merge request swri/ros-i/rosworld2021/roscon2021!23
* Get packages building with newest tesseract_ros2
* Merge branch 'update/repository-layout' into 'master'
  Repository layout update
  See merge request swri/ros-i/rosworld2021/roscon2021!22
* Applied CMake formatting
* Applied clang formatting
* Moved ROS2 packages to top-level directory
* Contributors: Michael Ripperger, dmerz, jlangsfeld, mripperger

1.0.0 (2021-10-19 16:56:56 +0000)
---------------------------------
