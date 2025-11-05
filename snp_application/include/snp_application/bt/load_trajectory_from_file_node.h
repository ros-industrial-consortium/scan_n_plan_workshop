#pragma once

#include <behaviortree_cpp/action_node.h>

namespace snp_application
{
class LoadTrajectoryFromFileNode : public BT::SyncActionNode
{
public:
  inline static std::string FILE_NAME_INPUT_PORT_KEY = "file";
  inline static std::string APPROACH_OUTPUT_PORT_KEY = "approach";
  inline static std::string PROCESS_OUTPUT_PORT_KEY = "process";
  inline static std::string DEPARTURE_OUTPUT_PORT_KEY = "departure";
  static BT::PortsList providedPorts();

  explicit LoadTrajectoryFromFileNode(const std::string& instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

}  // namespace snp_application
