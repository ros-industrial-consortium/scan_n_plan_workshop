#pragma once

#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/node.hpp>

namespace snp_application
{
/** @brief BT blackboard key for recording error messages from BT nodes */
inline static const std::string ERROR_MESSAGE_KEY = "error_message";

template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}

// Parameters
static const std::string MOTION_GROUP_PARAM = "motion_group";
static const std::string REF_FRAME_PARAM = "reference_frame";
static const std::string TCP_FRAME_PARAM = "tcp_frame";
static const std::string CAMERA_FRAME_PARAM = "camera_frame";
static const std::string MESH_FILE_PARAM = "mesh_file";
static const std::string START_STATE_REPLACEMENT_TOLERANCE_PARAM = "start_state_replacement_tolerance";
// Home state
static const std::string HOME_STATE_JOINT_VALUES_PARAM = "home_state_joint_values";
static const std::string HOME_STATE_JOINT_NAMES_PARAM = "home_state_joint_names";
//   Industrial Reconstruction
static const std::string IR_TSDF_VOXEL_PARAM = "ir.tsdf.voxel_length";
static const std::string IR_TSDF_SDF_PARAM = "ir.tsdf.sdf_trunc";
static const std::string IR_TSDF_MIN_X_PARAM = "ir.tsdf.min.x";
static const std::string IR_TSDF_MIN_Y_PARAM = "ir.tsdf.min.y";
static const std::string IR_TSDF_MIN_Z_PARAM = "ir.tsdf.min.z";
static const std::string IR_TSDF_MAX_X_PARAM = "ir.tsdf.max.x";
static const std::string IR_TSDF_MAX_Y_PARAM = "ir.tsdf.max.y";
static const std::string IR_TSDF_MAX_Z_PARAM = "ir.tsdf.max.z";
static const std::string IR_RGBD_DEPTH_SCALE_PARAM = "ir.rgbd.depth_scale";
static const std::string IR_RGBD_DEPTH_TRUNC_PARAM = "ir.rgbd.depth_trunc";
static const std::string IR_LIVE_PARAM = "ir.live";
static const std::string IR_MIN_FACES_PARAM = "ir.min_faces";
static const std::string IR_NORMAL_ANGLE_TOL_PARAM = "ir.normal_angle_tol";
static const std::string IR_NORMAL_X_PARAM = "ir.normal_x";
static const std::string IR_NORMAL_Y_PARAM = "ir.normal_y";
static const std::string IR_NORMAL_Z_PARAM = "ir.normal_z";
static const std::string IR_ARCHIVE_DIR_PARAM = "ir.archive_dir";

template <typename T>
T get_parameter(rclcpp::Node::SharedPtr node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

template <typename T>
T get_parameter_or(rclcpp::Node::SharedPtr node, const std::string& key, const T& default_val)
{
  T val;
  node->get_parameter_or(key, val, default_val);
  return val;
}

}  // namespace snp_application
