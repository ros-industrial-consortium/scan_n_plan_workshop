#include <snp_application/bt/snp_bt_ros_nodes.h>

#include <geometry_msgs/msg/pose_array.hpp>

namespace snp_application
{
bool TriggerServiceNode::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus TriggerServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
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
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
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
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Set output
  setOutput(APPROACH_OUTPUT_PORT_KEY, response->approach);
  setOutput(PROCESS_OUTPUT_PORT_KEY, response->process);
  setOutput(DEPARTURE_OUTPUT_PORT_KEY, response->departure);

  return BT::NodeStatus::SUCCESS;
}

sensor_msgs::msg::JointState jointTrajectoryPointToJointState(const trajectory_msgs::msg::JointTrajectory& jt,
                                                              const trajectory_msgs::msg::JointTrajectoryPoint& jtp)
{
  sensor_msgs::msg::JointState js;
  js.name = jt.joint_names;
  js.position = jtp.positions;
  return js;
}

bool GenerateFreespaceMotionPlanServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->js1 = snp_application::getBTInput<sensor_msgs::msg::JointState>(this, START_JOINT_STATE_INPUT_PORT_KEY);
  request->js2 = snp_application::getBTInput<sensor_msgs::msg::JointState>(this, GOAL_JOINT_STATE_INPUT_PORT_KEY);

  request->motion_group = get_parameter<std::string>(node_, MOTION_GROUP_PARAM);
  request->mesh_filename = get_parameter<std::string>(node_, MESH_FILE_PARAM);
  request->mesh_frame = get_parameter<std::string>(node_, REF_FRAME_PARAM);
  request->tcp_frame = get_parameter<std::string>(node_, TCP_FRAME_PARAM);

  return true;
}

BT::NodeStatus GenerateFreespaceMotionPlanServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Set output
  setOutput(TRAJECTORY_OUTPUT_PORT_KEY, response->trajectory);

  return BT::NodeStatus::SUCCESS;
}

bool GenerateScanMotionPlanServiceNode::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus GenerateScanMotionPlanServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Set output
  setOutput(APPROACH_OUTPUT_PORT_KEY, response->approach);
  setOutput(PROCESS_OUTPUT_PORT_KEY, response->process);
  setOutput(DEPARTURE_OUTPUT_PORT_KEY, response->departure);

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
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }

  // Copy the tool paths
  std::vector<snp_msgs::msg::ToolPath> tool_paths = response->tool_paths;

  // Get the reference frame for the poses
  auto ref_frame = get_parameter<std::string>(node_, REF_FRAME_PARAM);

  // Set the reference frame in each pose array
  for (snp_msgs::msg::ToolPath& tp : tool_paths)
  {
    for (geometry_msgs::msg::PoseArray& arr : tp.segments)
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
  request->tracking_frame = get_parameter<std::string>(node_, CAMERA_FRAME_PARAM);
  request->relative_frame = get_parameter<std::string>(node_, REF_FRAME_PARAM);
  request->tsdf_params.voxel_length = get_parameter_or<float>(node_, IR_TSDF_VOXEL_PARAM, 0.01f);
  request->tsdf_params.sdf_trunc = get_parameter_or<float>(node_, IR_TSDF_SDF_PARAM, 0.03f);
  request->tsdf_params.min_box_values.x = get_parameter_or<double>(node_, IR_TSDF_MIN_X_PARAM, 0.0);
  request->tsdf_params.min_box_values.y = get_parameter_or<double>(node_, IR_TSDF_MIN_Y_PARAM, 0.0);
  request->tsdf_params.min_box_values.z = get_parameter_or<double>(node_, IR_TSDF_MIN_Z_PARAM, 0.0);
  request->tsdf_params.max_box_values.x = get_parameter_or<double>(node_, IR_TSDF_MAX_X_PARAM, 0.0);
  request->tsdf_params.max_box_values.y = get_parameter_or<double>(node_, IR_TSDF_MAX_Y_PARAM, 0.0);
  request->tsdf_params.max_box_values.z = get_parameter_or<double>(node_, IR_TSDF_MAX_Z_PARAM, 0.0);
  request->rgbd_params.depth_scale = get_parameter_or<float>(node_, IR_RGBD_DEPTH_SCALE_PARAM, 1000.0);
  request->rgbd_params.depth_trunc = get_parameter_or<float>(node_, IR_RGBD_DEPTH_TRUNC_PARAM, 1.1f);
  request->live = get_parameter_or<bool>(node_, IR_LIVE_PARAM, true);

  return true;
}

BT::NodeStatus StartReconstructionServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    // config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

bool StopReconstructionServiceNode::setRequest(typename Request::SharedPtr& request)
{
  request->archive_directory = get_parameter_or<std::string>(node_, IR_ARCHIVE_DIR_PARAM, "");
  request->mesh_filepath = get_parameter<std::string>(node_, MESH_FILE_PARAM);
  request->min_num_faces = get_parameter<int>(node_, IR_MIN_FACES_PARAM);

  try
  {
    industrial_reconstruction_msgs::msg::NormalFilterParams norm_filt;
    norm_filt.angle = get_parameter<double>(node_, IR_NORMAL_ANGLE_TOL_PARAM);
    norm_filt.normal_direction.x = get_parameter<double>(node_, IR_NORMAL_X_PARAM);
    norm_filt.normal_direction.y = get_parameter<double>(node_, IR_NORMAL_Y_PARAM);
    norm_filt.normal_direction.z = get_parameter<double>(node_, IR_NORMAL_Z_PARAM);

    // Do not add a normal filter if the angle is less than 0.0
    if (norm_filt.angle > 0.0)
      request->normal_filters.push_back(norm_filt);
  }
  catch (const std::exception& ex)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, ex.what());
    return false;
  }

  return true;
}

BT::NodeStatus StopReconstructionServiceNode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, response->message);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

bool ToolPathsPubNode::setMessage(geometry_msgs::msg::PoseArray& msg)
{
  auto tool_paths = getBTInput<std::vector<snp_msgs::msg::ToolPath>>(this, TOOL_PATHS_INPUT_PORT_KEY);

  if (tool_paths.empty() || tool_paths.at(0).segments.empty())
    return true;

  // Set the frame ID from the first tool path segment
  msg.header.frame_id = tool_paths.at(0).segments.at(0).header.frame_id;

  for (const snp_msgs::msg::ToolPath& tp : tool_paths)
  {
    for (const geometry_msgs::msg::PoseArray& arr : tp.segments)
    {
      msg.poses.insert(msg.poses.end(), arr.poses.begin(), arr.poses.end());
    }
  }

  return true;
}

/**
 * @details Adapted from
 * https://github.com/a5-robotics/A5/blob/1c1b280970722c6b41d997f91ef50ff1571eeeac/a5_utils/src/trajectories/trajectories.cpp#L69-L75
 */
template <typename T>
static bool isSubset(std::vector<T> a, std::vector<T> b)
{
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  return std::includes(a.begin(), a.end(), b.begin(), b.end());
}

/**
 * @details Adapted from
 * https://github.com/a5-robotics/A5/blob/1c1b280970722c6b41d997f91ef50ff1571eeeac/a5_utils/src/trajectories/trajectories.cpp#L77C4-L96
 */
template <typename T>
static std::vector<std::size_t> getSubsetIndices(const std::vector<T>& superset, const std::vector<T>& subset)
{
  std::vector<std::size_t> indices;
  indices.reserve(subset.size());
  for (std::size_t i = 0; i < subset.size(); ++i)
  {
    const T& item = subset[i];
    auto it = std::find(superset.begin(), superset.end(), item);
    if (it == superset.end())
    {
      std::stringstream ss;
      ss << "Failed to find subset item (" << *it << ") in superset";
      throw std::runtime_error(ss.str());
    }
    indices.push_back(static_cast<std::size_t>(std::distance(superset.begin(), it)));
  }

  return indices;
}

/**
 * @details Adapted from
 * https://github.com/a5-robotics/A5/blob/1c1b280970722c6b41d997f91ef50ff1571eeeac/a5_utils/src/trajectories/trajectories.cpp#L104-L194
 */
trajectory_msgs::msg::JointTrajectory combine(const trajectory_msgs::msg::JointTrajectory& first,
                                              const trajectory_msgs::msg::JointTrajectory& second)
{
  if (first.joint_names == second.joint_names)
  {
    trajectory_msgs::msg::JointTrajectory result;
    result.header = first.header;
    result.joint_names = first.joint_names;

    // Insert the first trajectory points
    result.points.reserve(first.points.size() + second.points.size());
    result.points.insert(result.points.end(), first.points.begin(), first.points.end());

    // Insert the second trajectory points and modify their time from start to include the duration of the first
    // trajectory
    const auto lhs_duration =
        first.points.empty() ? builtin_interfaces::msg::Duration() : first.points.back().time_from_start;
    for (std::size_t i = 1; i < second.points.size(); ++i)
    {
      const trajectory_msgs::msg::JointTrajectoryPoint& pt = second.points[i];

      result.points.push_back(pt);
      result.points.back().time_from_start =
          rclcpp::Duration(result.points.back().time_from_start) + rclcpp::Duration(lhs_duration);
    }

    return result;
  }
  else if (isSubset(first.joint_names, second.joint_names))
  {
    // The first set of names includes the second set of names (i.e. first is the superset)
    trajectory_msgs::msg::JointTrajectory out(first);
    out.points.reserve(first.points.size() + second.points.size());

    // Get the indices of the subset in the superset
    std::vector<std::size_t> indices = getSubsetIndices(first.joint_names, second.joint_names);

    // Create new trajectory points from the subset with the additional superset joints
    const auto first_duration =
        first.points.empty() ? builtin_interfaces::msg::Duration() : first.points.back().time_from_start;
    for (std::size_t i = 1; i < second.points.size(); ++i)
    {
      const trajectory_msgs::msg::JointTrajectoryPoint& pt = second.points[i];

      // Copy the new trajectory point from the back of the first trajectory
      trajectory_msgs::msg::JointTrajectoryPoint new_pt(first.points.back());

      // Overwrite the joint values from the second trajectory
      for (std::size_t i = 0; i < pt.positions.size(); ++i)
      {
        const std::size_t idx = indices[i];
        new_pt.positions[idx] = pt.positions[i];
      }

      // Push this new trajectory point back onto the output trajectory
      new_pt.time_from_start = rclcpp::Duration(pt.time_from_start) + rclcpp::Duration(first_duration);
      out.points.push_back(new_pt);
    }

    return out;
  }
  else if (isSubset(second.joint_names, first.joint_names))
  {
    // The second set of names includes the first set of names (i.e. second is the superset)
    trajectory_msgs::msg::JointTrajectory out(second);
    out.points.reserve(first.points.size() + second.points.size());

    // Update the start times of the second trajectory
    const auto first_duration =
        first.points.empty() ? builtin_interfaces::msg::Duration() : first.points.back().time_from_start;
    for (trajectory_msgs::msg::JointTrajectoryPoint& pt : out.points)
    {
      pt.time_from_start = rclcpp::Duration(pt.time_from_start) + rclcpp::Duration(first_duration);
    }

    std::vector<std::size_t> indices = getSubsetIndices(second.joint_names, first.joint_names);

    // Iterate over the first points backwards and push them into the front of the new trajectory
    for (auto it = first.points.rbegin() + 1; it != first.points.rend(); ++it)
    {
      // Copy the trajectory from the first point of the second trajectory
      trajectory_msgs::msg::JointTrajectoryPoint new_pt(second.points.front());

      // Overwrite the joint values from the first trajectory
      for (std::size_t i = 0; i < indices.size(); ++i)
      {
        const std::size_t idx = indices[i];
        new_pt.positions[idx] = it->positions[i];
      }
      // Insert the new trajectory point at the beginning of the trajectory
      out.points.insert(out.points.begin(), new_pt);
    }

    return out;
  }

  throw std::runtime_error("The joint names/orderings are neither the same nor subsets of the other");
}

bool MotionPlanPubNode::setMessage(trajectory_msgs::msg::JointTrajectory& msg)
{
  try
  {
    msg = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, TRAJECTORY_INPUT_PORT_KEY);
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    ss << "Error extracting trajectory message: '" << ex.what() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return false;
  }

  return true;
}

bool FollowJointTrajectoryActionNode::setGoal(Goal& goal)
{
  goal.trajectory = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, TRAJECTORY_INPUT_PORT_KEY);
  for (auto& point : goal.trajectory.points)
  {
    point.effort.clear();
  }
  return true;
}

BT::NodeStatus FollowJointTrajectoryActionNode::onResultReceived(const WrappedResult& result)
{
  BT::NodeStatus status = BT::NodeStatus::SUCCESS;

  // Check the action result code
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    default:
      status = BT::NodeStatus::FAILURE;
      break;
  }

  // Check the action specific code
  switch (result.result->error_code)
  {
    case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
      break;
    default:
      status = BT::NodeStatus::FAILURE;
      break;
  }

  return status;
}

UpdateTrajectoryStartStateNode::UpdateTrajectoryStartStateNode(const std::string& instance_name,
                                                               const BT::NodeConfig& config,
                                                               rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(instance_name, config), node_(node)
{
}

BT::NodeStatus UpdateTrajectoryStartStateNode::tick()
{
  BT::Expected<trajectory_msgs::msg::JointTrajectory> trajectory_input =
      getInput<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY);
  if (!trajectory_input)
  {
    std::stringstream ss;
    ss << "Failed to get required trajectory_input value: '" << trajectory_input.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }
  trajectory_msgs::msg::JointTrajectory trajectory = trajectory_input.value();

  BT::Expected<sensor_msgs::msg::JointState> joint_state_input =
      getInput<sensor_msgs::msg::JointState>(START_JOINT_STATE_INPUT_PORT_KEY);
  if (!joint_state_input)
  {
    std::stringstream ss;
    ss << "Failed to get required joint_state_input value: '" << joint_state_input.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }
  sensor_msgs::msg::JointState joint_state = joint_state_input.value();

  // Get the tolerance from parameter
  auto tolerance = get_parameter<double>(node_, START_STATE_REPLACEMENT_TOLERANCE_PARAM);

  // Replace the start state of the trajectory with the current joint state
  {
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = trajectory.points.front().positions;
    start_point.velocities = std::vector<double>(start_point.positions.size(), 0);
    start_point.accelerations = std::vector<double>(start_point.positions.size(), 0);
    start_point.effort = std::vector<double>(start_point.positions.size(), 0);
    start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);

    // Find the index of the trajectory joint in the latest joint state message
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
    {
      const std::string& name = trajectory.joint_names[i];
      auto it = std::find(joint_state.name.begin(), joint_state.name.end(), name);
      if (it == joint_state.name.end())
      {
        std::stringstream ss;
        ss << "Failed to find joint '" << name << "' in latest joint state message";
        config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
        return BT::NodeStatus::FAILURE;
      }

      auto idx = std::distance(joint_state.name.begin(), it);

      // Check tolerance
      const double diff = std::abs(start_point.positions[i] - joint_state.position[idx]);
      if (diff > tolerance)
      {
        std::stringstream ss;
        ss << "Joint '" << trajectory.joint_names[i] << "' difference from start state (" << diff
           << " radians) exceeds start state replacement tolerance (" << tolerance << " radians)";
        config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
        return BT::NodeStatus::FAILURE;
      }

      start_point.positions[i] = joint_state.position[idx];
    }

    trajectory.points[0] = start_point;
  }

  BT::Result output = setOutput(TRAJECTORY_OUTPUT_PORT_KEY, trajectory);
  if (!output)
  {
    std::stringstream ss;
    ss << "Failed to set required output value: '" << output.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

/**
 * @details Adapted from
 * https://github.com/a5-robotics/A5/blob/1c1b280970722c6b41d997f91ef50ff1571eeeac/a5_utils/src/trajectories/trajectories.cpp#L77C4-L96
 */
trajectory_msgs::msg::JointTrajectory reverseTrajectory(const trajectory_msgs::msg::JointTrajectory& in)
{
  trajectory_msgs::msg::JointTrajectory out;
  out.header = in.header;
  out.joint_names = in.joint_names;

  const rclcpp::Duration& path_duration = in.points.back().time_from_start;

  for (auto it = in.points.rbegin(); it != in.points.rend(); ++it)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt(*it);

    // Change the signs of the velocity and acceleration
    for (std::size_t i = 0; i < pt.velocities.size(); ++i)
    {
      pt.velocities[i] = -pt.velocities[i];
    }
    for (std::size_t i = 0; i < pt.accelerations.size(); ++i)
    {
      pt.accelerations[i] = -pt.accelerations[i];
    }

    // Change the time from start
    pt.time_from_start = path_duration - it->time_from_start;

    out.points.push_back(std::move(pt));
  }

  return out;
}

ReverseTrajectoryNode::ReverseTrajectoryNode(const std::string& instance_name, const BT::NodeConfig& config)
  : BT::SyncActionNode(instance_name, config)
{
}

BT::NodeStatus ReverseTrajectoryNode::tick()
{
  BT::Expected<trajectory_msgs::msg::JointTrajectory> input =
      getInput<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY);
  if (!input)
  {
    std::stringstream ss;
    ss << "Failed to get required input value: '" << input.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }
  trajectory_msgs::msg::JointTrajectory in = input.value();

  trajectory_msgs::msg::JointTrajectory out = reverseTrajectory(in);

  BT::Result output = setOutput(TRAJECTORY_OUTPUT_PORT_KEY, out);
  if (!output)
  {
    std::stringstream ss;
    ss << "Failed to set required output value: '" << output.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

CombineTrajectoriesNode::CombineTrajectoriesNode(const std::string& instance_name, const BT::NodeConfig& config)
  : BT::SyncActionNode(instance_name, config)
{
}

BT::NodeStatus CombineTrajectoriesNode::tick()
{
  BT::Expected<trajectory_msgs::msg::JointTrajectory> first_input =
      getInput<trajectory_msgs::msg::JointTrajectory>(FIRST_TRAJECTORY_INPUT_PORT_KEY);
  BT::Expected<trajectory_msgs::msg::JointTrajectory> second_input =
      getInput<trajectory_msgs::msg::JointTrajectory>(SECOND_TRAJECTORY_INPUT_PORT_KEY);

  if (!first_input)
  {
    std::stringstream ss;
    ss << "Failed to get required input value: '" << first_input.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }

  if (!second_input)
  {
    std::stringstream ss;
    ss << "Failed to get required input value: '" << second_input.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }

  trajectory_msgs::msg::JointTrajectory first_trajectory = first_input.value();
  trajectory_msgs::msg::JointTrajectory second_trajectory = second_input.value();

  trajectory_msgs::msg::JointTrajectory out = combine(first_trajectory, second_trajectory);

  BT::Result output = setOutput(TRAJECTORY_OUTPUT_PORT_KEY, out);
  if (!output)
  {
    std::stringstream ss;
    ss << "Failed to set required output value: '" << output.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetCurrentJointStateNode::onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg)
{
  if (!last_msg)
  {
    std::stringstream ss;
    ss << "Failed to find a joint state";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }
  BT::Result output = setOutput(JOINT_STATE_OUTPUT_PORT_KEY, *last_msg);

  return BT::NodeStatus::SUCCESS;
}

RosSpinnerNode::RosSpinnerNode(const std::string& instance_name, const BT::NodeConfig& config,
                               rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(instance_name, config), node_(node)
{
}

BT::NodeStatus RosSpinnerNode::tick()
{
  rclcpp::spin_some(node_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace snp_application
