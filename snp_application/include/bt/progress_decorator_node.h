#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace snp_application
{
class ProgressDecoratorNode : public BT::DecoratorNode
{
public:
  inline static std::string PROGRESS_BAR_KEY = "progress_bar";
  inline static std::string START_PORT_KEY = "start";
  inline static std::string END_PORT_KEY = "end";

  static BT::PortsList providedPorts() { return {BT::InputPort<int>(START_PORT_KEY), BT::InputPort<int>(END_PORT_KEY) }; }

  using BT::DecoratorNode::DecoratorNode;

protected:
  BT::NodeStatus tick() override;
};

} // namespace snp_application
