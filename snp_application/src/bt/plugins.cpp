#include <snp_application/bt/button_approval_node.h>
#include <snp_application/bt/button_monitor_node.h>
#include <snp_application/bt/create_joint_state_message.h>
#include <snp_application/bt/extract_approach_process_departure_trajectories_node.h>
#include <snp_application/bt/load_trajectory_from_file_node.h>
#include <snp_application/bt/progress_decorator_node.h>
#include <snp_application/bt/set_page_decorator_node.h>
#include <snp_application/bt/snp_bt_ros_nodes.h>
#include <snp_application/bt/snp_sequence_with_memory_node.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/decorators/loop_node.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<snp_application::ButtonApprovalNode>("ButtonApproval");
  factory.registerNodeType<snp_application::ButtonMonitorNode>("ButtonMonitor");
  factory.registerNodeType<snp_application::CreateJointStateMessage>("CreateJointStateMessage");
  factory.registerNodeType<snp_application::ExtractApproachProcessDepartureTrajectoriesNode>(
      "ExtractApproachProcessDepartureTrajectories");
  factory.registerNodeType<snp_application::LoadTrajectoryFromFileNode>("LoadTrajectoryFromFile");
  factory.registerNodeType<snp_application::ProgressDecoratorNode>("Progress");
  factory.registerNodeType<snp_application::SetPageDecoratorNode>("SetPage");
  factory.registerNodeType<snp_application::SNPSequenceWithMemory>("SNPSequenceWithMemory");
  factory.registerNodeType<snp_application::ReverseTrajectoryNode>("ReverseTrajectory");
  factory.registerNodeType<snp_application::CombineTrajectoriesNode>("CombineTrajectories");
  factory.registerNodeType<BT::LoopNode<snp_msgs::msg::RasterMotionPlan>>("LoopThroughMotionPlans");
  factory.registerNodeType<snp_application::SplitMotionPlanNode>("SplitMotionPlan");
}

BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  using namespace snp_application;

  factory.registerNodeType<snp_application::RosSpinnerNode>("RosSpinner", params.nh.lock());
  factory.registerNodeType<snp_application::TriggerServiceNode>("TriggerService", params);
  factory.registerNodeType<snp_application::EmptyServiceNode>("EmptyService", params);
  factory.registerNodeType<snp_application::ToolPathsPubNode>("ToolPathsPub", params);
  factory.registerNodeType<snp_application::MotionPlanPubNode>("MotionPlanPub", params);
  factory.registerNodeType<snp_application::FollowJointTrajectoryActionNode>("FollowJointTrajectoryAction", params);
  factory.registerNodeType<snp_application::GetCurrentJointStateNode>("GetCurrentJointState", params);
  factory.registerNodeType<snp_application::UpdateTrajectoryStartStateNode>("UpdateTrajectoryStartState",
                                                                            params.nh.lock());
  factory.registerNodeType<snp_application::PlanToolPathServiceNode>("PlanToolPathService", params);
  factory.registerNodeType<snp_application::AddScanLinkServiceNode>("AddScanLinkService", params);
  factory.registerNodeType<snp_application::GenerateMotionPlanServiceNode>("GenerateMotionPlanService", params);
  factory.registerNodeType<snp_application::GenerateFreespaceMotionPlanServiceNode>(
      "GenerateFreespaceMotionPlanService", params);
  factory.registerNodeType<snp_application::StartReconstructionServiceNode>("StartReconstructionService", params);
  factory.registerNodeType<snp_application::StopReconstructionServiceNode>("StopReconstructionService", params);
}
