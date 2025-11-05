#pragma once

#include <behaviortree_cpp/action_node.h>

namespace snp_application
{
/**
 * @brief Loads a trajectory from a YAML file in to a `joint_trajectory_msgs/JointTrajectory` message
 */
class LoadTrajectoryFromFileNode : public BT::SyncActionNode
{
public:
  inline static std::string FILE_NAME_INPUT_PORT_KEY = "file";
  inline static std::string TRAJECTORY_OUTPUT_PORT_KEY = "approach";
  static BT::PortsList providedPorts();

  explicit LoadTrajectoryFromFileNode(const std::string& instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

}  // namespace snp_application
