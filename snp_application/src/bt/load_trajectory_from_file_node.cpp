#include <snp_application/bt/load_trajectory_from_file_node.h>
#include <snp_application/bt/utils.h>

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<builtin_interfaces::msg::Time>
{
  static Node encode(const builtin_interfaces::msg::Time& rhs)
  {
    Node node;
    node["sec"] = rhs.sec;
    node["nanosec"] = rhs.nanosec;
    return node;
  }

  static bool decode(const Node& node, builtin_interfaces::msg::Time& rhs)
  {
    rhs.sec = node["sec"].as<uint32_t>();
    rhs.nanosec = node["nanosec"].as<uint32_t>();
    return true;
  }
};

template <>
struct convert<std_msgs::msg::Header>
{
  static Node encode(const std_msgs::msg::Header& rhs)
  {
    Node node;
    node["stamp"] = rhs.stamp;
    node["frame_id"] = rhs.frame_id;
    return node;
  }

  static bool decode(const Node& node, std_msgs::msg::Header& rhs)
  {
    rhs.stamp = node["stamp"].as<builtin_interfaces::msg::Time>();
    rhs.frame_id = node["frame_id"].as<std::string>();
    return true;
  }
};

template <>
struct convert<trajectory_msgs::msg::JointTrajectoryPoint>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectoryPoint& rhs)
  {
    Node node;
    node["positions"] = rhs.positions;
    node["velocities"] = rhs.velocities;
    node["accelerations"] = rhs.accelerations;
    node["effort"] = rhs.effort;
    node["time_from_start"]["sec"] = rhs.time_from_start.sec;
    node["time_from_start"]["nanosec"] = rhs.time_from_start.nanosec;
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectoryPoint& rhs)
  {
    rhs.positions = node["positions"].as<std::vector<double>>();
    rhs.velocities = node["velocities"].as<std::vector<double>>();
    rhs.accelerations = node["accelerations"].as<std::vector<double>>();
    rhs.effort = node["effort"].as<std::vector<double>>();
    rhs.time_from_start.sec = node["time_from_start"]["sec"].as<uint32_t>();
    rhs.time_from_start.nanosec = node["time_from_start"]["nanosec"].as<uint32_t>();
    return true;
  }
};

template <>
struct convert<trajectory_msgs::msg::JointTrajectory>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectory& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["joint_names"] = rhs.joint_names;
    node["points"] = rhs.points;
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectory& rhs)
  {
    rhs.header = node["header"].as<std_msgs::msg::Header>();
    rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();
    rhs.points = node["points"].as<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>>();
    return true;
  }
};

}  // namespace YAML

trajectory_msgs::msg::JointTrajectory loadTrajectory(const std::string& file)
{
  YAML::Node scan_traj_node = YAML::LoadFile(file);
  return scan_traj_node.as<trajectory_msgs::msg::JointTrajectory>();
}

/**
 * @brief Tuple of an approach, process, and departure trajectory
 */
using ApproachProcessDeparture =
    std::tuple<trajectory_msgs::msg::JointTrajectory, trajectory_msgs::msg::JointTrajectory,
               trajectory_msgs::msg::JointTrajectory>;

/**
 * @brief Decomposes a trajectory into an approach, process, and departure trajectory
 * @details The approach trajectory consists of the first two points of the trajectory.
 * The process trajectory consists of points 1 through n-1.
 * The departure trajectory consists of points n-1 and n
 */
static ApproachProcessDeparture decomposeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.size() <= 4)
    throw std::runtime_error(
        "Trajectory must have at least 4 points to be decomposed into an approach, process, and departure trajectory");

  // Create an approach trajectory from the first two points
  trajectory_msgs::msg::JointTrajectory approach_trajectory;
  approach_trajectory.header = trajectory.header;
  approach_trajectory.joint_names = trajectory.joint_names;
  approach_trajectory.points.push_back(trajectory.points[0]);
  approach_trajectory.points.push_back(trajectory.points[1]);

  // Create a process trajectory from points 1 through n-1
  trajectory_msgs::msg::JointTrajectory process_trajectory;
  process_trajectory.header = trajectory.header;
  process_trajectory.joint_names = trajectory.joint_names;
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i)
  {
    process_trajectory.points.push_back(trajectory.points[i]);
  }

  // Create a departure trajectory from points n-1 and n
  trajectory_msgs::msg::JointTrajectory departure_trajectory;
  departure_trajectory.header = trajectory.header;
  departure_trajectory.joint_names = trajectory.joint_names;
  departure_trajectory.points.push_back(*(trajectory.points.rbegin() + 1));
  departure_trajectory.points.push_back(*trajectory.points.rbegin());

  return std::make_tuple(approach_trajectory, process_trajectory, departure_trajectory);
}

namespace snp_application
{
BT::PortsList LoadTrajectoryFromFileNode::providedPorts()
{
  return { BT::InputPort(FILE_NAME_INPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(APPROACH_OUTPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(PROCESS_OUTPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(DEPARTURE_OUTPUT_PORT_KEY) };
}

LoadTrajectoryFromFileNode::LoadTrajectoryFromFileNode(const std::string& instance_name, const BT::NodeConfig& config)
  : BT::SyncActionNode(instance_name, config)
{
}

BT::NodeStatus LoadTrajectoryFromFileNode::tick()
{
  BT::Expected<std::string> input = getInput<std::string>(FILE_NAME_INPUT_PORT_KEY);
  if (!input)
  {
    std::stringstream ss;
    ss << "Failed to get required input value: '" << input.error() << "'";
    config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());
    return BT::NodeStatus::FAILURE;
  }

  trajectory_msgs::msg::JointTrajectory approach, process, departure;
  std::tie(approach, process, departure) = decomposeTrajectory(loadTrajectory(input.value()));

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
