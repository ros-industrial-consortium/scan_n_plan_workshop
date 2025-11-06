#pragma once

#include <behaviortree_cpp/action_node.h>

namespace snp_application
{
/**
 * @brief Loads a trajectory from a YAML file into a `joint_trajectory_msgs/JointTrajectory` message
 * @ingroup bt_plugins
 */
class LoadTrajectoryFromFileNode : public BT::SyncActionNode
{
public:
  inline static const std::string FILE_NAME_INPUT_PORT_KEY = "file";
  inline static const std::string TRAJECTORY_OUTPUT_PORT_KEY = "trajectory";
  static BT::PortsList providedPorts();

  explicit LoadTrajectoryFromFileNode(const std::string& instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

}  // namespace snp_application
