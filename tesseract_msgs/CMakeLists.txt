cmake_minimum_required(VERSION 3.5.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX C)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

set(msg_files
    "msg/AllowedCollisionEntry.msg"
    "msg/ChainGroup.msg"
    "msg/CollisionGeometry.msg"
    "msg/CollisionMarginData.msg"
    "msg/CollisionMarginOverrideType.msg"
    "msg/ContactMarginPair.msg"
    "msg/ContactResult.msg"
    "msg/ContactResultVector.msg"
    "msg/Environment.msg"
    "msg/EnvironmentCommand.msg"
    "msg/EnvironmentState.msg"
    "msg/Geometry.msg"
    "msg/GroupsJointState.msg"
    "msg/GroupsJointStates.msg"
    "msg/GroupsKinematicPlugins.msg"
    "msg/GroupsTCP.msg"
    "msg/GroupsTCPs.msg"
    "msg/Inertial.msg"
    "msg/Joint.msg"
    "msg/JointCalibration.msg"
    "msg/JointDynamics.msg"
    "msg/JointGroup.msg"
    "msg/JointLimits.msg"
    "msg/JointMimic.msg"
    "msg/JointSafety.msg"
    "msg/JointState.msg"
    "msg/KinematicsInformation.msg"
    "msg/KinematicsPluginInfo.msg"
    "msg/Link.msg"
    "msg/LinkGroup.msg"
    "msg/Material.msg"
    "msg/Mesh.msg"
    "msg/ObjectColor.msg"
    "msg/OctreeSubType.msg"
    "msg/PlannerProfileRemapping.msg"
    "msg/PlanningRequest.msg"
    "msg/PlanningRequestArchive.msg"
    "msg/PlanningResponse.msg"
    "msg/PlanningResponseArchive.msg"
    "msg/PluginInfo.msg"
    "msg/ProfileMap.msg"
    "msg/SceneGraph.msg"
    "msg/StringDoublePair.msg"
    "msg/StringLimitsPair.msg"
    "msg/StringPair.msg"
    "msg/StringPluginInfoPair.msg"
    "msg/TaskInfo.msg"
    "msg/Trajectory.msg"
    "msg/TransformMap.msg"
    "msg/VisualGeometry.msg")

set(srv_files
    "srv/ComputeContactResultVector.srv"
    "srv/GetEnvironmentChanges.srv"
    "srv/GetEnvironmentInformation.srv"
    "srv/GetMotionPlan.srv"
    "srv/ModifyEnvironment.srv"
    "srv/SaveSceneGraph.srv")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  octomap_msgs
  trajectory_msgs
  actionlib_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament package settings
ament_export_dependencies(
  rosidl_default_runtime
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  octomap_msgs
  trajectory_msgs
  actionlib_msgs)

ament_package()
