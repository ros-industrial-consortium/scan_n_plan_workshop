#pragma once

#include <rclcpp/node.hpp>

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>

// Parameters
// Behavior Tree
inline const char* BT_PLUGIN_LIBS_PARAM = "bt_plugin_libs";
inline const char* BT_ROS_PLUGIN_LIBS_PARAM = "bt_ros_plugin_libs";
inline const char* BT_FILES_PARAM = "bt_files";
inline const char* BT_TIMEOUT_PARAM = "bt_timeout";
inline const char* BT_PARAM = "tree";
inline const char* BT_FREESPACE_PARAM = "freespace_tree";
// General
inline const char* FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory_action";
inline const char* START_STATE_REPLACEMENT_TOLERANCE_PARAM = "start_state_replacement_tolerance";
inline const char* FREESPACE_MOTION_GROUP_PARAM = "freespace_motion_group";
// Home state
inline const char* HOME_STATE_JOINT_NAMES_PARAM = "home_state_joint_names";
inline const char* HOME_STATE_JOINT_POSITIONS_PARAM = "home_state_joint_positions";
// Frames
inline const char* REF_FRAME_PARAM = "reference_frame";
inline const char* TCP_FRAME_PARAM = "tcp_frame";
inline const char* CAMERA_FRAME_PARAM = "camera_frame";
inline const char* CAMERA_TCP_FRAME_PARAM = "camera_tcp_frame";
// Scan
inline const char* SCAN_MOTION_GROUP_PARAM = "scan_motion_group";
inline const char* SCAN_TRAJ_FILE_PARAM = "scan_trajectory_file";
inline const char* SCAN_MESH_FILE_PARAM = "scan_mesh_file";
inline const char* SCAN_TPP_CONFIG_FILE_PARAM = "scan_tpp_config_file";
// Process
inline const char* PROCESS_MOTION_GROUP_PARAM = "process_motion_group";
inline const char* PROCESS_MESH_FILE_PARAM = "process_mesh_file";
inline const char* PROCESS_TPP_CONFIG_FILE_PARAM = "process_tpp_config_file";
// Industrial Reconstruction
inline const char* IR_TSDF_VOXEL_PARAM = "ir_voxel_length";
inline const char* IR_TSDF_SDF_PARAM = "ir_sdf_trunc";
inline const char* IR_TSDF_MIN_X_PARAM = "ir_min_x";
inline const char* IR_TSDF_MIN_Y_PARAM = "ir_min_y";
inline const char* IR_TSDF_MIN_Z_PARAM = "ir_min_z";
inline const char* IR_TSDF_MAX_X_PARAM = "ir_max_x";
inline const char* IR_TSDF_MAX_Y_PARAM = "ir_max_y";
inline const char* IR_TSDF_MAX_Z_PARAM = "ir_max_z";
inline const char* IR_RGBD_DEPTH_SCALE_PARAM = "ir_depth_scale";
inline const char* IR_RGBD_DEPTH_TRUNC_PARAM = "ir_depth_trunc";
inline const char* IR_LIVE_PARAM = "ir_live";
inline const char* IR_MIN_FACES_PARAM = "ir_min_faces";
inline const char* IR_NORMAL_ANGLE_TOL_PARAM = "ir_normal_angle_tol";
inline const char* IR_NORMAL_X_PARAM = "ir_normal_x";
inline const char* IR_NORMAL_Y_PARAM = "ir_normal_y";
inline const char* IR_NORMAL_Z_PARAM = "ir_normal_z";
inline const char* IR_ARCHIVE_DIR_PARAM = "ir_archive_dir";

namespace snp_application
{
/**
 * @brief A behavior tree blackboard populated by ROS parameters
 * @details This blackboard declares parameters in a given ROS node that correspond directly to entries required in the
 * blackboard by the SNP behavior tree nodes. When the ROS parameters are updated, the entries in the blackboard also
 * get updated.
 */
class SnpBlackboard : public BT::Blackboard
{
public:
  SnpBlackboard(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr parent = {});

protected:
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
};

/**
 * @brief Function interface for generating a behavior tree factory
 */
using BehaviorTreeFactoryGenerator = std::function<std::unique_ptr<BT::BehaviorTreeFactory>()>;

/**
 * @brief Function for generating a behavior tree factory using a ROS node to load plugins and behavior tree files from
 * parameters.
 * @details This function uses the node to:
 * 1. Load standard BT plugins from the parameter BT_PLUGIN_LIBS_PARAM
 * 2. Load ROS2 BT plugins from the parameter BT_ROS_PLUGIN_LIBS_PARAM.
 * The node provided to this function is used to create/operate all ROS interfaces required by any generated behavior
 * tree nodes. The service/action timeout specified by the parameter BT_TIMEOUT_PARAM.
 * 3. Load behavior tree files specified in the parameter BT_FILES_PARAM
 */
std::unique_ptr<BT::BehaviorTreeFactory> generateBehaviorTreeFactory(rclcpp::Node::SharedPtr node);

}  // namespace snp_application
