#pragma once

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <QWidget>
#include <rclcpp/node.hpp>

namespace Ui
{
class SNPWidget;
}

class QStackedWidget;
class QTextEdit;

namespace snp_application
{
class SNPWidget : public QWidget
{
public:
  explicit SNPWidget(rclcpp::Node::SharedPtr rviz_node, QWidget* parent = nullptr);

protected:
  void runTreeWithThread(const std::string& bt_tree_name);

  virtual std::unique_ptr<BT::BehaviorTreeFactory> createBTFactory(int ros_timeout);
  QStackedWidget* getStackedWidget();
  QTextEdit* getTextEdit();

  rclcpp::Node::SharedPtr bt_node_;
  Ui::SNPWidget* ui_;
  BT::Blackboard::Ptr board_;
  std::shared_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace snp_application
