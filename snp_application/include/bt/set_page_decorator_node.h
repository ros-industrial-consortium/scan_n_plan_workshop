#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace snp_application
{
/**
 * @brief Decorator node that sets the page of a Qt stacked widget
 * @details A pointer to a stacked widget (QStackedWidget*) is provided via the blackboard.
 * When the node is entered, the page of the stacked widget is set to the index provided in the "index" input port
 */
class SetPageDecoratorNode : public BT::DecoratorNode
{
public:
  inline static std::string STACKED_WIDGET_KEY = "stacked_widget";
  inline static std::string INDEX_PORT_KEY = "index";
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(INDEX_PORT_KEY) };
  }

  using BT::DecoratorNode::DecoratorNode;

protected:
  BT::NodeStatus tick() override;
};

}  // namespace snp_application
