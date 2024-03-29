#include <snp_application/bt/button_monitor_node.h>
#include <snp_application/bt/utils.h>

#include <QAbstractButton>

namespace snp_application
{
ButtonMonitorNode::ButtonMonitorNode(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config), ok_(true)
{
  auto button_key = getBTInput<std::string>(this, BUTTON_PORT_KEY);
  auto* button = this->config().blackboard->get<QAbstractButton*>(button_key);

  QObject::connect(button, &QAbstractButton::clicked, [this, button_key](const bool) { ok_ = false; });
}

BT::NodeStatus ButtonMonitorNode::tick()
{
  if (!ok_)
  {
    ok_ = true;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace snp_application
