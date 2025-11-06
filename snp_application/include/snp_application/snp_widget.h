#pragma once

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <QWidget>
#include <rclcpp/node.hpp>

namespace Ui
{
class SNPWidget;
}

class QStackedWidget;
class QTextEdit;

// Parameters
// Behavior Tree
inline static const char* BT_FILES_PARAM = "bt_files";
inline static const char* BT_PLUGIN_LIBS_PARAM = "bt_plugin_libs";
inline static const char* BT_ROS_PLUGIN_LIBS_PARAM = "bt_ros_plugin_libs";
inline static const char* BT_PARAM = "tree";
inline static const char* BT_FREESPACE_PARAM = "freespace_tree";
inline static const char* BT_TIMEOUT_PARAM = "bt_timeout";
// General
inline static const char* FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory_action";
inline static const std::string START_STATE_REPLACEMENT_TOLERANCE_PARAM = "start_state_replacement_tolerance";
// Motion groups
inline static const char* MOTION_GROUP_PROCESS_PARAM = "motion_group.process";
inline static const char* MOTION_GROUP_FREESPACE_PARAM = "motion_group.freespace";
inline static const char* MOTION_GROUP_SCAN_PARAM = "motion_group.scan";
// Frames
inline static const char* FRAME_REF_PARAM = "frame.reference";
inline static const char* FRAME_TCP_PARAM = "frame.tcp";
inline static const char* FRAME_CAMERA_PARAM = "frame.camera";
// Scan
inline static const char* SCAN_TRAJ_FILE_PARAM = "scan.trajectory_file";
inline static const char* SCAN_MESH_FILE_PARAM = "scan.mesh_file";
inline static const char* SCAN_TPP_CONFIG_FILE_PARAM = "scan.tpp_config_file";
// Process
inline static const char* PROCESS_MESH_FILE_PARAM = "process.mesh_file";
inline static const char* PROCESS_TPP_CONFIG_FILE_PARAM = "process.tpp_config_file";
// Home state
inline static const char* HOME_STATE_JOINT_VALUES_PARAM = "home_state_joint_values";
inline static const char* HOME_STATE_JOINT_NAMES_PARAM = "home_state_joint_names";
// Industrial Reconstruction
inline static const char* IR_TSDF_VOXEL_PARAM = "ir.tsdf.voxel_length";
inline static const char* IR_TSDF_SDF_PARAM = "ir.tsdf.sdf_trunc";
inline static const char* IR_TSDF_MIN_X_PARAM = "ir.tsdf.min.x";
inline static const char* IR_TSDF_MIN_Y_PARAM = "ir.tsdf.min.y";
inline static const char* IR_TSDF_MIN_Z_PARAM = "ir.tsdf.min.z";
inline static const char* IR_TSDF_MAX_X_PARAM = "ir.tsdf.max.x";
inline static const char* IR_TSDF_MAX_Y_PARAM = "ir.tsdf.max.y";
inline static const char* IR_TSDF_MAX_Z_PARAM = "ir.tsdf.max.z";
inline static const char* IR_RGBD_DEPTH_SCALE_PARAM = "ir.rgbd.depth_scale";
inline static const char* IR_RGBD_DEPTH_TRUNC_PARAM = "ir.rgbd.depth_trunc";
inline static const char* IR_LIVE_PARAM = "ir.live";
inline static const char* IR_MIN_FACES_PARAM = "ir.min_faces";
inline static const char* IR_NORMAL_ANGLE_TOL_PARAM = "ir.normal_angle_tol";
inline static const char* IR_NORMAL_X_PARAM = "ir.normal_x";
inline static const char* IR_NORMAL_Y_PARAM = "ir.normal_y";
inline static const char* IR_NORMAL_Z_PARAM = "ir.normal_z";
inline static const char* IR_ARCHIVE_DIR_PARAM = "ir.archive_dir";

namespace snp_application
{
class SNPWidget : public QWidget
{
public:
  explicit SNPWidget(rclcpp::Node::SharedPtr rviz_node, QWidget* parent = nullptr);

protected:
  void runTreeWithThread(const std::string& bt_tree_name);

  virtual std::unique_ptr<BT::BehaviorTreeFactory> createBTFactory(int ros_timeout);
  QStackedWidget* getStackedWidget();
  QTextEdit* getTextEdit();

  rclcpp::Node::SharedPtr bt_node_;
  Ui::SNPWidget* ui_;
  BT::Blackboard::Ptr board_;
  std::shared_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace snp_application
