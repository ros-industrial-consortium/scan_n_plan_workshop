#include <snp_application/bt/button_approval_node.h>
#include <snp_application/bt/button_monitor_node.h>
#include <snp_application/bt/progress_decorator_node.h>
#include <snp_application/bt/set_page_decorator_node.h>
#include <snp_application/bt/snp_bt_ros_nodes.h>
#include <snp_application/bt/snp_sequence_with_memory_node.h>

#include <behaviortree_cpp/bt_factory.h>

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
  factory.registerNodeType<snp_application::UpdateTrajectoryStartStateNode>("UpdateTrajectoryStartState", params.nh);
  factory.registerNodeType<snp_application::RosSpinnerNode>("RosSpinner", params.nh);

  factory.registerNodeType<snp_application::TriggerServiceNode>("TriggerService", params);
  factory.registerNodeType<snp_application::ExecuteMotionPlanServiceNode>("ExecuteMotionPlanService", params);
  factory.registerNodeType<snp_application::GenerateMotionPlanServiceNode>("GenerateMotionPlanService", params);
  factory.registerNodeType<snp_application::GenerateFreespaceMotionPlanServiceNode>("GenerateFreespaceMotionPlanServic"
                                                                                    "e",
                                                                                    params);
  factory.registerNodeType<snp_application::GenerateScanMotionPlanServiceNode>("GenerateScanMotionPlanService", params);
  factory.registerNodeType<snp_application::GenerateToolPathsServiceNode>("GenerateToolPathsService", params);
  factory.registerNodeType<snp_application::StartReconstructionServiceNode>("StartReconstructionService", params);
  factory.registerNodeType<snp_application::StopReconstructionServiceNode>("StopReconstructionService", params);
  factory.registerNodeType<snp_application::ToolPathsPubNode>("ToolPathsPub", params);
  factory.registerNodeType<snp_application::MotionPlanPubNode>("MotionPlanPub", params);
  factory.registerNodeType<snp_application::FollowJointTrajectoryActionNode>("FollowJointTrajectoryAction", params);
  factory.registerNodeType<snp_application::GetCurrentJointStateNode>("GetCurrentJointState", params);
}
