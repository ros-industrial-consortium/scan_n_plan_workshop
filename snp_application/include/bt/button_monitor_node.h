#pragma once

#include <atomic>
#include <behaviortree_cpp/condition_node.h>

namespace snp_application
{
class ButtonMonitorNode : public BT::ConditionNode
{
public:
  inline static std::string BUTTON_PORT_KEY = "button";
  static BT::PortsList providedPorts() { return { BT::InputPort<std::string>(BUTTON_PORT_KEY) }; }

  ButtonMonitorNode(const std::string& name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;

  std::atomic_bool ok_;
};

} // namespace snp_application
