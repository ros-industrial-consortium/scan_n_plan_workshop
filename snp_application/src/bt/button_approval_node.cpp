#include "bt/button_approval_node.h"
#include "bt/utils.h"

#include <QAbstractButton>

namespace snp_application
{
ButtonApprovalNode::ButtonApprovalNode(const std::string& instance_name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(instance_name, config), approved_(false), disapproved_(false)
{
  auto approve_button_key = getBTInput<std::string>(this, APPROVE_BUTTON_PORT_KEY);
  if (!approve_button_key.empty())
  {
    approve_button_ = this->config().blackboard->get<QAbstractButton*>(approve_button_key);
    QObject::connect(approve_button_, &QAbstractButton::clicked, [this](const bool) { approved_ = true; });
  }

  auto disapprove_button_key = getBTInput<std::string>(this, DISAPPROVE_BUTTON_PORT_KEY);
  if (!disapprove_button_key.empty())
  {
    disapprove_button_ = this->config().blackboard->get<QAbstractButton*>(disapprove_button_key);
    QObject::connect(disapprove_button_, &QAbstractButton::clicked, [this](const bool) { disapproved_ = true; });
  }

  setButtonsEnabled(false);
}

void ButtonApprovalNode::setButtonsEnabled(bool enable)
{
  if (approve_button_ != nullptr)
    approve_button_->setEnabled(enable);

  if (disapprove_button_ != nullptr)
    disapprove_button_->setEnabled(enable);
}

BT::NodeStatus ButtonApprovalNode::onStart()
{
  approved_ = false;
  disapproved_ = false;

  setButtonsEnabled(true);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ButtonApprovalNode::onRunning()
{
  if (disapproved_)
  {
    setButtonsEnabled(false);
    return BT::NodeStatus::FAILURE;
  }

  if (approved_)
  {
    setButtonsEnabled(false);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ButtonApprovalNode::onHalted()
{
  setButtonsEnabled(false);
}

}  // namespace snp_application
