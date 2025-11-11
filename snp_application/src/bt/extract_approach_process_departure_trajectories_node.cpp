#include <snp_application/bt/extract_approach_process_departure_trajectories_node.h>
#include <snp_application/bt/utils.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

/**
 * @brief Tuple of an approach, process, and departure trajectory
 */
using ApproachProcessDeparture =
    std::tuple<trajectory_msgs::msg::JointTrajectory, trajectory_msgs::msg::JointTrajectory,
               trajectory_msgs::msg::JointTrajectory>;

/**
 * @brief Extracts an approach, process, and departure trajectory from a nominal trajectory
 * @details The approach trajectory consists of the first two points of the trajectory.
 * The process trajectory consists of points 1 through n-1.
 * The departure trajectory consists of points n-1 and n
 */
static ApproachProcessDeparture extract(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.size() <= 4)
    throw std::runtime_error(
        "Trajectory must have at least 4 points to be decomposed into an approach, process, and departure trajectory");

  // Lambda function for creating a sub-trajectory
  auto create_sub_trajectory =
      [&trajectory](const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator& start,
                    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator& end) {
        trajectory_msgs::msg::JointTrajectory output;
        output.header = trajectory.header;
        output.joint_names = trajectory.joint_names;

        for (auto it = start; it < end; ++it)
        {
          auto pt = *it;

          // Offset the time from start
          pt.time_from_start.sec -= start->time_from_start.sec;
          pt.time_from_start.nanosec -= start->time_from_start.nanosec;

          output.points.push_back(pt);
        }

        return output;
      };

  // Create an approach trajectory from the first two points
  trajectory_msgs::msg::JointTrajectory approach_trajectory =
      create_sub_trajectory(trajectory.points.begin(), trajectory.points.begin() + 2);

  // Create a process trajectory from points 1 through n-1
  trajectory_msgs::msg::JointTrajectory process_trajectory =
      create_sub_trajectory(trajectory.points.begin() + 1, trajectory.points.end() - 1);

  // Create a departure trajectory from points n-1 and n
  trajectory_msgs::msg::JointTrajectory departure_trajectory =
      create_sub_trajectory(trajectory.points.end() - 2, trajectory.points.end());

  return std::make_tuple(approach_trajectory, process_trajectory, departure_trajectory);
}

namespace snp_application
{
BT::PortsList ExtractApproachProcessDepartureTrajectoriesNode::providedPorts()
{
  return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(APPROACH_OUTPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(PROCESS_OUTPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(DEPARTURE_OUTPUT_PORT_KEY) };
}

ExtractApproachProcessDepartureTrajectoriesNode::ExtractApproachProcessDepartureTrajectoriesNode(
    const std::string& instance_name, const BT::NodeConfig& config)
  : BT::SyncActionNode(instance_name, config)
{
}

BT::NodeStatus ExtractApproachProcessDepartureTrajectoriesNode::tick()
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

  trajectory_msgs::msg::JointTrajectory approach, process, departure;
  std::tie(approach, process, departure) = extract(input.value());

  const BT::Result output_1 = setOutput(APPROACH_OUTPUT_PORT_KEY, approach);
  const BT::Result output_2 = setOutput(PROCESS_OUTPUT_PORT_KEY, process);
  const BT::Result output_3 = setOutput(DEPARTURE_OUTPUT_PORT_KEY, departure);

  const std::vector<BT::Result> outputs{ output_1, output_2, output_3 };

  for (const auto& output : outputs)
  {
    if (!output)
    {
      config().blackboard->set(ERROR_MESSAGE_KEY, output.get_unexpected().error());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace snp_application
