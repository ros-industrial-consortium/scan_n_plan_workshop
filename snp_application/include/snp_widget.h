#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>

namespace Ui
{
class SNPWidget;
}

namespace snp_application
{
class SNPWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SNPWidget(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

private:
  void runTreeWithThread();

  rclcpp::Node::SharedPtr node_;
  Ui::SNPWidget* ui_;
  BT::Blackboard::Ptr board_;
  BT::BehaviorTreeFactory factory_;
};

} // namespace snp_application
