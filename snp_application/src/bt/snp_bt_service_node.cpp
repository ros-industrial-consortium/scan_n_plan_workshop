#include "bt/snp_bt_service_node.h"
#include "bt/utils.h"

#include <geometry_msgs/msg/pose_array.hpp>

namespace snp_application
{
bool TriggerServiceNode::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus TriggerServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool ExecuteMotionPlanServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->motion_plan = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, MOTION_PLAN_INPUT_PORT_KEY);
  request->use_tool = getBTInput<bool>(this, USE_TOOL_INPUT_PORT_KEY);
  return true;
}

BT::NodeStatus ExecuteMotionPlanServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool GenerateMotionPlanServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->tool_paths = getBTInput<std::vector<snp_msgs::msg::ToolPath>>(this, TOOL_PATHS_INPUT_PORT_KEY);

  request->motion_group = get_parameter<std::string>(node_, MOTION_GROUP_PARAM);
  request->tcp_frame = get_parameter<std::string>(node_, TCP_FRAME_PARAM);
  request->mesh_filename = get_parameter<std::string>(node_, MESH_FILE_PARAM);
  request->mesh_frame = get_parameter<std::string>(node_, REF_FRAME_PARAM);

  return true;
}

BT::NodeStatus GenerateMotionPlanServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Set output
  setOutput(MOTION_PLAN_OUTPUT_PORT_KEY, response->motion_plan);

  return BT::NodeStatus::SUCCESS;
}

bool GenerateScanMotionPlanServiceNode::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus GenerateScanMotionPlanServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Set output
  setOutput(MOTION_PLAN_OUTPUT_PORT_KEY, response->motion_plan);

  return BT::NodeStatus::SUCCESS;
}

bool GenerateToolPathsServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->mesh_filename = get_parameter<std::string>(node_, MESH_FILE_PARAM);
  request->mesh_frame = get_parameter<std::string>(node_, REF_FRAME_PARAM);

  return true;
}

BT::NodeStatus GenerateToolPathsServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Copy the tool paths
  std::vector<snp_msgs::msg::ToolPath> tool_paths = response->tool_paths;

  // Get the reference frame for the poses
  auto ref_frame = get_parameter<std::string>(node_, REF_FRAME_PARAM);

  // Set the reference frame in each pose array
  for(snp_msgs::msg::ToolPath& tp : tool_paths)
  {
    for(geometry_msgs::msg::PoseArray& arr : tp.segments)
    {
      arr.header.frame_id = ref_frame;
    }
  }

  // Set output
  setOutput(TOOL_PATHS_OUTPUT_PORT_KEY, tool_paths);

  return BT::NodeStatus::SUCCESS;
}

bool StartReconstructionServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->tsdf_params.voxel_length =     get_parameter_or<float>(node_, "tsdf.voxel_length", 0.01f);
  request->tsdf_params.sdf_trunc =        get_parameter_or<float>(node_, "tsdf.sdf_trunc", 0.03f);
  request->tsdf_params.min_box_values.x = get_parameter_or<double>(node_, "tsdf.min.x", 0.0);
  request->tsdf_params.min_box_values.y = get_parameter_or<double>(node_, "tsdf.min.y", 0.0);
  request->tsdf_params.min_box_values.z = get_parameter_or<double>(node_, "tsdf.min.z", 0.0);
  request->tsdf_params.max_box_values.x = get_parameter_or<double>(node_, "tsdf.max.x", 0.0);
  request->tsdf_params.max_box_values.y = get_parameter_or<double>(node_, "tsdf.max.y", 0.0);
  request->tsdf_params.max_box_values.z = get_parameter_or<double>(node_, "tsdf.max.z", 0.0);
  request->rgbd_params.depth_scale =      get_parameter_or<float>(node_, "rgbd.depth_scale", 1000.0);
  request->rgbd_params.depth_trunc =      get_parameter_or<float>(node_, "rgbd.depth_trunc", 1.1f);

  return true;
}

BT::NodeStatus StartReconstructionServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    // RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

bool StopReconstructionServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->archive_directory = "";
  request->mesh_filepath = get_parameter<std::string>(node_, MESH_FILE_PARAM);
  request->min_num_faces = 1000;

  industrial_reconstruction_msgs::msg::NormalFilterParams norm_filt;
  norm_filt.normal_direction.x = 0;
  norm_filt.normal_direction.y = 0;
  norm_filt.normal_direction.z = 1;
  norm_filt.angle = 85;
  request->normal_filters.push_back(norm_filt);

  return true;
}

BT::NodeStatus StopReconstructionServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if(!response->success)
  {
    RCLCPP_ERROR(node_->get_logger(), response->message);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

bool ToolPathsPubNode::setMessage(geometry_msgs::msg::PoseArray& msg)
{
  auto tool_paths = getBTInput<std::vector<snp_msgs::msg::ToolPath>>(this, TOOL_PATHS_INPUT_PORT_KEY);

  if(tool_paths.empty() || tool_paths.at(0).segments.empty())
    return true;

  // Set the frame ID from the first tool path segment
  msg.header.frame_id = tool_paths.at(0).segments.at(0).header.frame_id;

  for(const snp_msgs::msg::ToolPath& tp : tool_paths)
  {
    for(const geometry_msgs::msg::PoseArray& arr : tp.segments)
    {
      msg.poses.insert(msg.poses.end(), arr.poses.begin(), arr.poses.end());
    }
  }

  return true;
}

bool MotionPlanPubNode::setMessage(trajectory_msgs::msg::JointTrajectory& msg)
{
  msg = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, MOTION_PLAN_INPUT_PORT_KEY);
  return true;
}

} // namespace snp_application
