#pragma once

#include <atomic>
#include <behaviortree_cpp/action_node.h>

class QAbstractButton;

namespace snp_application
{
/**
 * @brief Action node that listens to the state of two Qt buttons to succeed or fail the
 * @details Pointers (QAbstractButton*) to the buttons are provided via the blackboard using the keys defined in the
 * input ports When the node is active and the "approve" button is pressed, the node returns success. When the node is
 * active and the "disapprove" button is pressed, the node returns failure.
 */
class ButtonApprovalNode : public BT::StatefulActionNode
{
public:
  inline static std::string APPROVE_BUTTON_PORT_KEY = "approve_button";
  inline static std::string DISAPPROVE_BUTTON_PORT_KEY = "disapprove_button";
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(APPROVE_BUTTON_PORT_KEY),
             BT::InputPort<std::string>(DISAPPROVE_BUTTON_PORT_KEY) };
  }

  explicit ButtonApprovalNode(const std::string& instance_name, const BT::NodeConfig& config);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setButtonsEnabled(bool enable);

  std::atomic_bool approved_;
  std::atomic_bool disapproved_;

  QAbstractButton* approve_button_{ nullptr };
  QAbstractButton* disapprove_button_{ nullptr };
};

}  // namespace snp_application
