#pragma once

#include <snp_application/snp_behavior_tree.h>
#include <QWidget>

namespace Ui
{
class SNPWidget;
}

namespace BT
{
class StatusChangeLogger;
}

class QStackedWidget;
class QTextEdit;

namespace snp_application
{
class SNPWidget : public QWidget
{
public:
  explicit SNPWidget(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr blackboard,
                     BehaviorTreeFactoryGenerator bt_factory_generator, QWidget* parent = nullptr);

protected:
  void runTreeWithThread(const std::string& bt_tree_name);

  QStackedWidget* getStackedWidget();
  QTextEdit* getTextEdit();

  BT::Blackboard::Ptr blackboard_;
  BehaviorTreeFactoryGenerator bt_factory_generator_;
  Ui::SNPWidget* ui_;

  std::shared_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace snp_application
