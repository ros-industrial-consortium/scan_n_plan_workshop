#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace snp_application
{
class SetPageDecoratorNode : public BT::DecoratorNode
{
public:
  inline static std::string STACKED_WIDGET_KEY = "stacked_widget";
  inline static std::string INDEX_PORT_KEY = "index";
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>(INDEX_PORT_KEY) }; }

  using BT::DecoratorNode::DecoratorNode;

protected:
  BT::NodeStatus tick() override;
};

} // namespace snp_application
