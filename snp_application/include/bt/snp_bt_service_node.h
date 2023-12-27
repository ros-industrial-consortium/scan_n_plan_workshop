#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <industrial_reconstruction_msgs/srv/start_reconstruction.hpp>
#include <industrial_reconstruction_msgs/srv/stop_reconstruction.hpp>
#include <snp_msgs/srv/execute_motion_plan.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <snp_msgs/srv/generate_scan_motion_plan.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace snp_application
{
static const std::string MOTION_GROUP_PARAM = "motion_group";
static const std::string REF_FRAME_PARAM = "reference_frame";
static const std::string TCP_FRAME_PARAM = "tcp_frame";
static const std::string CAMERA_FRAME_PARAM = "camera_frame";
static const std::string MESH_FILE_PARAM = "mesh_file";

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

class TriggerServiceNode : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  using BT::RosServiceNode<std_srvs::srv::Trigger>::providedPorts;
  using BT::RosServiceNode<std_srvs::srv::Trigger>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class ExecuteMotionPlanServiceNode : public BT::RosServiceNode<snp_msgs::srv::ExecuteMotionPlan>
{
public:
  inline static std::string MOTION_PLAN_INPUT_PORT_KEY = "motion_plan";
  inline static std::string USE_TOOL_INPUT_PORT_KEY = "use_tool";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(MOTION_PLAN_INPUT_PORT_KEY), BT::InputPort<bool>(USE_TOOL_INPUT_PORT_KEY) });
  }

  using BT::RosServiceNode<snp_msgs::srv::ExecuteMotionPlan>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class GenerateMotionPlanServiceNode : public BT::RosServiceNode<snp_msgs::srv::GenerateMotionPlan>
{
public:
  inline static std::string TOOL_PATHS_INPUT_PORT_KEY = "tool_paths";
  inline static std::string MOTION_PLAN_OUTPUT_PORT_KEY = "motion_plan";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::vector<snp_msgs::msg::ToolPath>>(TOOL_PATHS_INPUT_PORT_KEY),
      BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(MOTION_PLAN_OUTPUT_PORT_KEY)
    });
  }

  using BT::RosServiceNode<snp_msgs::srv::GenerateMotionPlan>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class GenerateScanMotionPlanServiceNode : public BT::RosServiceNode<snp_msgs::srv::GenerateScanMotionPlan>
{
public:
  inline static std::string MOTION_PLAN_OUTPUT_PORT_KEY = "motion_plan";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(MOTION_PLAN_OUTPUT_PORT_KEY) });
  }

  using BT::RosServiceNode<snp_msgs::srv::GenerateScanMotionPlan>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class GenerateToolPathsServiceNode : public BT::RosServiceNode<snp_msgs::srv::GenerateToolPaths>
{
public:
  inline static std::string TOOL_PATHS_OUTPUT_PORT_KEY = "tool_paths";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::OutputPort<std::vector<snp_msgs::msg::ToolPath>>(TOOL_PATHS_OUTPUT_PORT_KEY) });
  }

  using BT::RosServiceNode<snp_msgs::srv::GenerateToolPaths>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class StartReconstructionServiceNode : public BT::RosServiceNode<industrial_reconstruction_msgs::srv::StartReconstruction>
{
public:
  using BT::RosServiceNode<industrial_reconstruction_msgs::srv::StartReconstruction>::providedPorts;
  using BT::RosServiceNode<industrial_reconstruction_msgs::srv::StartReconstruction>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class StopReconstructionServiceNode : public BT::RosServiceNode<industrial_reconstruction_msgs::srv::StopReconstruction>
{
public:
  using BT::RosServiceNode<industrial_reconstruction_msgs::srv::StopReconstruction>::providedPorts;
  using BT::RosServiceNode<industrial_reconstruction_msgs::srv::StopReconstruction>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

class ToolPathsPubNode : public BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>
{
public:
  inline static std::string TOOL_PATHS_INPUT_PORT_KEY = "tool_paths";
  inline static BT::PortsList providedPorts() { return providedBasicPorts({ BT::InputPort<std::vector<snp_msgs::msg::ToolPath>>(TOOL_PATHS_INPUT_PORT_KEY) }); }
  using BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>::RosTopicPubNode;

  bool setMessage(geometry_msgs::msg::PoseArray& msg) override;
};

class MotionPlanPubNode : public BT::RosTopicPubNode<trajectory_msgs::msg::JointTrajectory>
{
public:
  inline static std::string MOTION_PLAN_INPUT_PORT_KEY = "motion_plan";
  inline static BT::PortsList providedPorts() { return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(MOTION_PLAN_INPUT_PORT_KEY) }); }
  using BT::RosTopicPubNode<trajectory_msgs::msg::JointTrajectory>::RosTopicPubNode;

  bool setMessage(trajectory_msgs::msg::JointTrajectory& msg) override;
};

class FollowJointTrajectoryActionNode : public BT::RosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
  inline static std::string TRAJECTORY_INPUT_PORT_KEY = "trajectory";
  inline static BT::PortsList providedPorts() { return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY) }); }
  using BT::RosActionNode<control_msgs::action::FollowJointTrajectory>::RosActionNode;

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

}  // namespace snp_application
