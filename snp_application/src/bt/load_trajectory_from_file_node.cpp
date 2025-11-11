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

namespace snp_application
{
BT::PortsList LoadTrajectoryFromFileNode::providedPorts()
{
  return { BT::InputPort(FILE_NAME_INPUT_PORT_KEY),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) };
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

  // Load the trajectory from file
  YAML::Node scan_traj_node = YAML::LoadFile(input.value());

  // Set the output
  const BT::Result output =
      setOutput(TRAJECTORY_OUTPUT_PORT_KEY, scan_traj_node.as<trajectory_msgs::msg::JointTrajectory>());
  if (!output)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, output.get_unexpected().error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace snp_application
