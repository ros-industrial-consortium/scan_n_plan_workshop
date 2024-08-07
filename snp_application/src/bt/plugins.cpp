#include <snp_application/bt/button_approval_node.h>
#include <snp_application/bt/button_monitor_node.h>
#include <snp_application/bt/progress_decorator_node.h>
#include <snp_application/bt/set_page_decorator_node.h>
#include <snp_application/bt/snp_bt_ros_nodes.h>
#include <snp_application/bt/snp_sequence_with_memory_node.h>
#include <snp_application/bt/utils.h>

#include <behaviortree_cpp/bt_factory.h>

template <typename T>
void try_declare_parameter(rclcpp::Node::SharedPtr node, const std::string& key, const T& value)
{
  if (!node->has_parameter(key))
    node->declare_parameter(key, value);
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<snp_application::ButtonApprovalNode>("ButtonApproval");
  factory.registerNodeType<snp_application::ButtonMonitorNode>("ButtonMonitor");
  factory.registerNodeType<snp_application::ProgressDecoratorNode>("Progress");
  factory.registerNodeType<snp_application::SetPageDecoratorNode>("SetPage");
  factory.registerNodeType<snp_application::SNPSequenceWithMemory>("SNPSequenceWithMemory");
  factory.registerNodeType<snp_application::ReverseTrajectoryNode>("ReverseTrajectory");
  factory.registerNodeType<snp_application::CombineTrajectoriesNode>("CombineTrajectories");
}

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  using namespace snp_application;

  factory.registerNodeType<snp_application::RosSpinnerNode>("RosSpinner", params.nh);
  factory.registerNodeType<snp_application::TriggerServiceNode>("TriggerService", params);
  factory.registerNodeType<snp_application::ExecuteMotionPlanServiceNode>("ExecuteMotionPlanService", params);
  factory.registerNodeType<snp_application::ToolPathsPubNode>("ToolPathsPub", params);
  factory.registerNodeType<snp_application::MotionPlanPubNode>("MotionPlanPub", params);
  factory.registerNodeType<snp_application::FollowJointTrajectoryActionNode>("FollowJointTrajectoryAction", params);
  factory.registerNodeType<snp_application::GetCurrentJointStateNode>("GetCurrentJointState", params);
  factory.registerNodeType<snp_application::GenerateScanMotionPlanServiceNode>("GenerateScanMotionPlanService", params);
  factory.registerNodeType<snp_application::GenerateToolPathsServiceNode>("GenerateToolPathsService", params);

  // Nodes requiring parameters
  // Update trajectory start state
  try_declare_parameter<double>(params.nh, START_STATE_REPLACEMENT_TOLERANCE_PARAM, 1.0 * M_PI / 180.0);
  factory.registerNodeType<snp_application::UpdateTrajectoryStartStateNode>("UpdateTrajectoryStartState", params.nh);

  // Motion plan generation
  try_declare_parameter<std::string>(params.nh, MOTION_GROUP_PARAM, "");
  try_declare_parameter<std::string>(params.nh, REF_FRAME_PARAM, "");
  try_declare_parameter<std::string>(params.nh, TCP_FRAME_PARAM, "");
  factory.registerNodeType<snp_application::GenerateMotionPlanServiceNode>("GenerateMotionPlanService", params);

  try_declare_parameter<std::string>(params.nh, FREESPACE_MOTION_GROUP_PARAM, "");
  factory.registerNodeType<snp_application::GenerateFreespaceMotionPlanServiceNode>(
      "GenerateFreespaceMotionPlanService", params);

  // Industrial reconstruction start
  try_declare_parameter<std::string>(params.nh, CAMERA_FRAME_PARAM, "");
  try_declare_parameter<float>(params.nh, IR_TSDF_VOXEL_PARAM, 0.01f);
  try_declare_parameter<float>(params.nh, IR_TSDF_SDF_PARAM, 0.03f);
  try_declare_parameter<double>(params.nh, IR_TSDF_MIN_X_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MIN_Y_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MIN_Z_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MAX_X_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MAX_Y_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_TSDF_MAX_Z_PARAM, 0.0);
  try_declare_parameter<float>(params.nh, IR_RGBD_DEPTH_SCALE_PARAM, 1000.0);
  try_declare_parameter<float>(params.nh, IR_RGBD_DEPTH_TRUNC_PARAM, 1.1f);
  try_declare_parameter<bool>(params.nh, IR_LIVE_PARAM, true);
  try_declare_parameter<double>(params.nh, IR_NORMAL_ANGLE_TOL_PARAM, -1.0);
  try_declare_parameter<double>(params.nh, IR_NORMAL_X_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_NORMAL_Y_PARAM, 0.0);
  try_declare_parameter<double>(params.nh, IR_NORMAL_Z_PARAM, 1.0);
  factory.registerNodeType<snp_application::StartReconstructionServiceNode>("StartReconstructionService", params);

  // Industrial reconstruction stop
  try_declare_parameter<int>(params.nh, IR_MIN_FACES_PARAM, 0);
  try_declare_parameter<std::string>(params.nh, MESH_FILE_PARAM, "");
  try_declare_parameter<std::string>(params.nh, IR_ARCHIVE_DIR_PARAM, "");
  factory.registerNodeType<snp_application::StopReconstructionServiceNode>("StopReconstructionService", params);
}
