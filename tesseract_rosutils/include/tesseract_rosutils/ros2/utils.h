/**
 * @file utils.h
 * @brief Tesseract ROS utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_ROSUTILS_ROS2_UTILS_H
#define TESSERACT_ROSUTILS_ROS2_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_msgs/msg/allowed_collision_entry.hpp>
#include <tesseract_msgs/msg/chain_group.hpp>
#include <tesseract_msgs/msg/collision_geometry.hpp>
#include <tesseract_msgs/msg/contact_margin_pair.hpp>
#include <tesseract_msgs/msg/contact_result_vector.hpp>
#include <tesseract_msgs/msg/environment_command.hpp>
#include <tesseract_msgs/msg/geometry.hpp>
#include <tesseract_msgs/msg/groups_joint_state.hpp>
#include <tesseract_msgs/msg/groups_joint_states.hpp>
#include <tesseract_msgs/msg/groups_kinematic_plugins.hpp>
#include <tesseract_msgs/msg/groups_tcp.hpp>
#include <tesseract_msgs/msg/groups_tc_ps.hpp>
#include <tesseract_msgs/msg/inertial.hpp>
#include <tesseract_msgs/msg/joint.hpp>
#include <tesseract_msgs/msg/joint_calibration.hpp>
#include <tesseract_msgs/msg/joint_dynamics.hpp>
#include <tesseract_msgs/msg/joint_group.hpp>
#include <tesseract_msgs/msg/joint_limits.hpp>
#include <tesseract_msgs/msg/joint_mimic.hpp>
#include <tesseract_msgs/msg/joint_safety.hpp>
#include <tesseract_msgs/msg/joint_state.hpp>
#include <tesseract_msgs/msg/kinematics_information.hpp>
#include <tesseract_msgs/msg/link.hpp>
#include <tesseract_msgs/msg/link_group.hpp>
#include <tesseract_msgs/msg/material.hpp>
#include <tesseract_msgs/msg/mesh.hpp>
#include <tesseract_msgs/msg/planning_request_archive.hpp>
#include <tesseract_msgs/msg/scene_graph.hpp>
#include <tesseract_msgs/msg/string_double_pair.hpp>
#include <tesseract_msgs/msg/string_pair.hpp>
#include <tesseract_msgs/msg/environment.hpp>
#include <tesseract_msgs/msg/environment_state.hpp>
#include <tesseract_msgs/msg/task_info.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>
#include <tesseract_msgs/msg/transform_map.hpp>
#include <tesseract_msgs/msg/visual_geometry.hpp>
#include <tesseract_msgs/msg/planner_profile_remapping.hpp>
#include <tesseract_msgs/msg/plugin_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
//#include <ros/serialization.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_process_managers/core/task_info.h>

namespace tesseract_rosutils::ros2
{
std::string locateResource(const std::string& url);

class ROSResourceLocator : public tesseract_common::SimpleResourceLocator
{
public:
  ROSResourceLocator();
};

bool isMsgEmpty(const sensor_msgs::msg::JointState& msg);

bool isIdentical(const tesseract_geometry::Geometry& shape1, const tesseract_geometry::Geometry& shape2);

bool isIdentical(const tesseract_scene_graph::Visual& visual1, const tesseract_scene_graph::Visual& visual2);

bool isIdentical(const tesseract_scene_graph::Collision& collision1,
                 const tesseract_scene_graph::Collision& collision2);

bool isIdentical(const tesseract_scene_graph::Link& link1, const tesseract_scene_graph::Link& link2);

/** \brief Construct the message that corresponds to the shape. Return false on failure. */
bool toMsg(tesseract_msgs::msg::Geometry& geometry_msgs, const tesseract_geometry::Geometry& geometry);

bool fromMsg(tesseract_geometry::Geometry::Ptr& geometry, const tesseract_msgs::msg::Geometry& geometry_msg);

bool toMsg(tesseract_msgs::msg::Material& material_msg, const tesseract_scene_graph::Material::Ptr& material);

bool fromMsg(tesseract_scene_graph::Material::Ptr& material, const tesseract_msgs::msg::Material& material_msg);

bool toMsg(tesseract_msgs::msg::Inertial& inertial_msg, const tesseract_scene_graph::Inertial::Ptr& inertial);

bool fromMsg(tesseract_scene_graph::Inertial::Ptr& inertial, const tesseract_msgs::msg::Inertial& inertial_msg);

bool toMsg(tesseract_msgs::msg::VisualGeometry& visual_msg, const tesseract_scene_graph::Visual& visual);

bool fromMsg(tesseract_scene_graph::Visual::Ptr& visual, const tesseract_msgs::msg::VisualGeometry& visual_msg);

bool toMsg(tesseract_msgs::msg::CollisionGeometry& collision_msg, const tesseract_scene_graph::Collision& collision);

bool fromMsg(tesseract_scene_graph::Collision::Ptr& collision,
             const tesseract_msgs::msg::CollisionGeometry& collision_msg);

bool toMsg(tesseract_msgs::msg::Link& link_msg, const tesseract_scene_graph::Link& link);

tesseract_scene_graph::Link fromMsg(const tesseract_msgs::msg::Link& link_msg);

bool toMsg(tesseract_msgs::msg::JointCalibration& joint_calibration_msg,
           const tesseract_scene_graph::JointCalibration::Ptr& joint_calibration);

bool fromMsg(tesseract_scene_graph::JointCalibration::Ptr& joint_calibration,
             const tesseract_msgs::msg::JointCalibration& joint_calibration_msg);

bool toMsg(tesseract_msgs::msg::JointDynamics& joint_dynamics_msg,
           const tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics);

bool fromMsg(tesseract_scene_graph::JointDynamics::Ptr& joint_dynamics,
             const tesseract_msgs::msg::JointDynamics& joint_dynamics_msg);

bool toMsg(tesseract_msgs::msg::JointLimits& joint_limits_msg,
           const tesseract_scene_graph::JointLimits::Ptr& joint_limits);

bool fromMsg(tesseract_scene_graph::JointLimits::Ptr& joint_limits,
             const tesseract_msgs::msg::JointLimits& joint_limits_msg);

bool toMsg(tesseract_msgs::msg::JointMimic& joint_mimic_msg, const tesseract_scene_graph::JointMimic::Ptr& joint_mimic);

bool fromMsg(tesseract_scene_graph::JointMimic::Ptr& joint_mimic,
             const tesseract_msgs::msg::JointMimic& joint_mimic_msg);

bool toMsg(tesseract_msgs::msg::JointSafety& joint_safety_msg,
           const tesseract_scene_graph::JointSafety::Ptr& joint_safety);

bool fromMsg(tesseract_scene_graph::JointSafety::Ptr& joint_safety,
             const tesseract_msgs::msg::JointSafety& joint_safety_msg);

bool toMsg(tesseract_msgs::msg::Joint& joint_msg, const tesseract_scene_graph::Joint& joint);

tesseract_planning::PlannerProfileRemapping
fromMsg(const tesseract_msgs::msg::PlannerProfileRemapping& profile_remapping_msg);
tesseract_msgs::msg::PlannerProfileRemapping
toMsg(const tesseract_planning::PlannerProfileRemapping& profile_remapping);

tesseract_common::PairsCollisionMarginData
fromMsg(const std::vector<tesseract_msgs::msg::ContactMarginPair>& contact_margin_pairs_msg);
std::vector<tesseract_msgs::msg::ContactMarginPair>
toMsg(const tesseract_common::PairsCollisionMarginData& contact_margin_pairs);

tesseract_common::CollisionMarginData fromMsg(const tesseract_msgs::msg::CollisionMarginData& contact_margin_data_msg);
tesseract_msgs::msg::CollisionMarginData toMsg(const tesseract_common::CollisionMarginData& contact_margin_data);

tesseract_common::CollisionMarginOverrideType
fromMsg(const tesseract_msgs::msg::CollisionMarginOverrideType& contact_margin_override_type_msg);
tesseract_msgs::msg::CollisionMarginOverrideType
toMsg(const tesseract_common::CollisionMarginOverrideType& contact_margin_override_type);

tesseract_scene_graph::Joint fromMsg(const tesseract_msgs::msg::Joint& joint_msg);

/**
 * @brief Convert allowed collision matrix to a vector of allowed collision entry messages
 * @param acm_msg Vector of allowed collision entries to populate
 * @param acm Allowed collision matrix to convert to message
 * @return True if successful, otherwise false
 */
bool toMsg(std::vector<tesseract_msgs::msg::AllowedCollisionEntry>& acm_msg,
           const tesseract_scene_graph::AllowedCollisionMatrix& acm);

void toMsg(tesseract_msgs::msg::SceneGraph& scene_graph_msg, const tesseract_scene_graph::SceneGraph& scene_graph);

tesseract_scene_graph::SceneGraph fromMsg(const tesseract_msgs::msg::SceneGraph& scene_graph_msg);

bool toMsg(tesseract_msgs::msg::EnvironmentCommand& command_msg, const tesseract_environment::Command& command);

tesseract_environment::Commands fromMsg(const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands_msg);

tesseract_environment::Command::Ptr fromMsg(const tesseract_msgs::msg::EnvironmentCommand& command_msg);

bool toMsg(std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands_msg,
           const tesseract_environment::Commands& commands, unsigned long past_revision);

void toMsg(tesseract_msgs::msg::EnvironmentState& state_msg, const tesseract_environment::Environment& env,
           bool include_joint_states = true);

/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
void toMsg(std::vector<tesseract_msgs::msg::JointState>& traj_msg, const tesseract_common::JointTrajectory& traj);

/**
 * @brief Generate a JointTrajectory from message
 * @param traj_msg The trajectory message to convert
 * @param traj The joint trajectory
 */
tesseract_common::JointTrajectory fromMsg(const std::vector<tesseract_msgs::msg::JointState>& traj_msg);

bool processMsg(tesseract_environment::Environment& env, const sensor_msgs::msg::JointState& joint_state_msg);
bool processMsg(const tesseract_environment::Environment::Ptr& env,
                const sensor_msgs::msg::JointState& joint_state_msg);

/**
 * @brief Apply the provided commands to the environment
 * @param env The environment to apply the commands
 * @param env_command_msg The commands to apply
 * @return True if successful, otherwise false
 */
bool processMsg(tesseract_environment::Environment& env,
                const std::vector<tesseract_msgs::msg::EnvironmentCommand>& env_command_msg);

/**
 * @brief Convert Geometry Pose Message to Eigen
 * @param pose Eigen type to filled out
 * @param pose_msg The message to be converted
 * @return True if successful, otherwise false
 */
bool fromMsg(Eigen::Isometry3d& pose, const geometry_msgs::msg::Pose& pose_msg);

/**
 * @brief Convert Eigen to Geometry Pose Message
 * @param pose_msg Geometry Pose Message to filled out
 * @param pose The Eigen type to be converted
 * @return True if successful, otherwise false
 */
bool toMsg(geometry_msgs::msg::Pose& pose_msg, const Eigen::Isometry3d& pose);

void toMsg(tesseract_msgs::msg::ContactResult& contact_result_msg,
           const tesseract_collision::ContactResult& contact_result, const rclcpp::Time& stamp = rclcpp::Time());

/**
 * @brief Convert kinematics plugin info to message
 * @param info Kinematics plugin info
 * @return Kinematics plugin info
 */
tesseract_msgs::msg::KinematicsPluginInfo toMsg(const tesseract_common::KinematicsPluginInfo& info);

/**
 * @brief Convert plugin info map to message
 * @param info_map plugin info map
 * @return plugin info map
 */
std::vector<tesseract_msgs::msg::StringPluginInfoPair> toMsg(const tesseract_common::PluginInfoMap& info_map);

/**
 * @brief Convert plugin info to message
 * @param info plugin info
 * @return plugin info
 */
tesseract_msgs::msg::PluginInfo toMsg(const tesseract_common::PluginInfo& info);

/**
 * @brief Convert a vector of Eigen::Isometry3d into a pose array
 * @param Pose Array
 * @param transforms A vector of transforms
 * @return True if successful, otherwise false
 */
bool toMsg(geometry_msgs::msg::PoseArray& pose_array, const tesseract_common::VectorIsometry3d& transforms);

/**
 * @brief Convert a chain group to message
 * @param group Chain group
 * @return Chain group message
 */
tesseract_msgs::msg::ChainGroup toMsg(tesseract_srdf::ChainGroups::const_reference group);

/**
 * @brief Convert a group's joint state to message
 * @param group Group's joint states
 * @return Group's joint states message
 */
tesseract_msgs::msg::GroupsJointStates toMsg(tesseract_srdf::GroupJointStates::const_reference group);

/**
 * @brief Convert a group's tool center points to message
 * @param group Group's tool center points
 * @return Group's tool center points message
 */
tesseract_msgs::msg::GroupsTCPs toMsg(tesseract_srdf::GroupTCPs::const_reference group);

/**
 * @brief Convert manipulator managers data to message
 * @param kin_info Kinematics information message to populate
 * @param manager The Manipulator manager to convert to message
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::msg::KinematicsInformation& kin_info_msg,
           const tesseract_srdf::KinematicsInformation& kin_info);

/**
 * @brief This will populate the kinematics information from the kinematics information message
 * @param kin_info The kinematics information data structure to populate
 * @param kin_info_msg The kinematics information message
 * @return True if successful, otherwise false
 */
bool fromMsg(tesseract_srdf::KinematicsInformation& kin_info,
             const tesseract_msgs::msg::KinematicsInformation& kin_info_msg);

/**
 * @brief Convert kinematics plugin info from message
 * @param info_msg Kinematics plugin info message
 * @return Kinematics plugin info
 */
tesseract_common::KinematicsPluginInfo fromMsg(const tesseract_msgs::msg::KinematicsPluginInfo& info_msg);

/**
 * @brief Convert plugin info map from message
 * @param info_map plugin info map message
 * @return plugin info map
 */
tesseract_common::PluginInfoMap fromMsg(const std::vector<tesseract_msgs::msg::StringPluginInfoPair>& info_map_msg);

/**
 * @brief Convert plugin info from message
 * @param info plugin info message
 * @return plugin info
 */
tesseract_common::PluginInfo fromMsg(const tesseract_msgs::msg::PluginInfo& info_msg);

/**
 * @brief This will populate a transform map message
 * @param transform_map_msg The transform map message
 * @param transform_map The transform map
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::msg::TransformMap& transform_map_msg, const tesseract_common::TransformMap& transform_map);

/**
 * @brief This will populate a transform map given a message
 * @param transform_map The transform map
 * @param transform_map_msg The transform map message
 * @return True if successful, otherwise false
 */
bool fromMsg(tesseract_common::TransformMap& transform_map, const tesseract_msgs::msg::TransformMap& transform_map_msg);

/**
 * @brief This will populate a joint states map message
 * @param joint_state_msg The joint states map message
 * @param joint_state The joint state map
 * @return True if successful, otherwise false
 */
bool toMsg(sensor_msgs::msg::JointState& joint_state_msg, const std::unordered_map<std::string, double>& joint_state);

/**
 * @brief This will populate a joint states from message
 * @param joint_state The joint state map
 * @param joint_state_msg The joint states map message
 * @return True if successful, otherwise false
 */
bool fromMsg(std::unordered_map<std::string, double>& joint_state, const sensor_msgs::msg::JointState& joint_state_msg);

/**
 * @brief Converts a Environment object to a Tesseract msg
 * @param tesseract_msg Resulting Message
 * @param env Input Environment object
 * @param include_joint_states If true, the joint_states element will be populated with the current state
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::msg::Environment& environment_msg, const tesseract_environment::Environment& env,
           bool include_joint_states = true);

/**
 * @brief Converts a Environment object to a Tesseract msg
 * @param tesseract_msg Resulting Message
 * @param env Input Environment object
 * @param include_joint_states If true, the joint_states element will be populated with the current state
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::msg::Environment& environment_msg, const tesseract_environment::Environment::ConstPtr& env,
           bool include_joint_states = true);

/**
 * @brief Converts a Tesseract msg to a Environment object
 * @param tesseract_msg Input Tesseract msg
 * @return Resulting Tesseract Object if successful, nullptr otherwise
 */
tesseract_environment::Environment::Ptr fromMsg(const tesseract_msgs::msg::Environment& environment_msg);

/**
 * @brief Converts a TaskInfo object to a TaskInfo msg
 * @param task_info_msg Resulting message
 * @param task_info TaskInfo object
 * @return True if successful, otherwise false
 */
bool toMsg(tesseract_msgs::msg::TaskInfo& task_info_msg, tesseract_planning::TaskInfo::ConstPtr task_info);

/**
 * @brief Converts a TaskInfo msg to a TaskInfo object
 * @param task_info_msg Input TaskInfo msg
 * @return Resulting Tesseract Object if successful, nullptr otherwise
 */
tesseract_planning::TaskInfo::Ptr fromMsg(const tesseract_msgs::msg::TaskInfo& task_info_msg);

/**
 * @brief Converts a tesseract_common::JointTrajectory msg to a trajectory_msgs::JointTrajectory object
 * @param joint_trajectory Input JointTrajectory msg
 * @return Resulting Tesseract
 */
trajectory_msgs::msg::JointTrajectory toMsg(const tesseract_common::JointTrajectory& joint_trajectory,
                                            const tesseract_scene_graph::SceneState& initial_state);

// template <typename MessageType>
// inline bool toFile(const std::string& filepath, const MessageType& msg)
//{
//  std::ofstream ofs(filepath, std::ios::out | std::ios::binary);

//  uint32_t serial_size = ros::serialization::serializationLength(msg);
//  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

//  ros::serialization::OStream ostream(obuffer.get(), serial_size);
//  ros::serialization::serialize(ostream, msg);
//  ofs.write((char*)obuffer.get(), serial_size);
//  ofs.close();

//  return true;
//}

// template <typename MessageType>
// inline MessageType fromFile(const std::string& filepath)
//{
//  std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
//  ifs.seekg(0, std::ios::end);
//  std::streampos end = ifs.tellg();
//  ifs.seekg(0, std::ios::beg);
//  std::streampos begin = ifs.tellg();

//  auto file_size = static_cast<long>(end - begin);
//  boost::shared_array<uint8_t> ibuffer(new uint8_t[static_cast<unsigned long>(file_size)]);
//  ifs.read((char*)ibuffer.get(), file_size);
//  ros::serialization::IStream istream(ibuffer.get(), static_cast<uint32_t>(file_size));

//  MessageType msg;
//  ros::serialization::deserialize(istream, msg);
//  ifs.close();

//  return msg;
//}

}  // namespace tesseract_rosutils::ros2

#endif
