#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

class QTextEdit;

namespace snp_application
{
/**
 * @brief Logs a subset of node status changes to a Qt text edit widget
 */
class TextEditLogger : public BT::StatusChangeLogger
{
public:
  TextEditLogger(BT::TreeNode* root_node, QTextEdit* log);

  void callback(BT::Duration /*timestamp*/, const BT::TreeNode& node, BT::NodeStatus /*prev_status*/,
                BT::NodeStatus status) override;

  void flush() override;

protected:
  QTextEdit* log_;
};

}  // namespace snp_application
