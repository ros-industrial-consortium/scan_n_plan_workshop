#include "bt/text_edit_logger.h"

#include <QTextCursor>
#include <QTextEdit>
#include <QTextStream>

namespace snp_application
{
TextEditLogger::TextEditLogger(BT::TreeNode* root_node, QTextEdit* log) : BT::StatusChangeLogger(root_node), log_(log)
{
}

void TextEditLogger::callback(BT::Duration /*timestamp*/, const BT::TreeNode& node, BT::NodeStatus /*prev_status*/,
                              BT::NodeStatus status)
{
  QString message;
  QTextStream ss(&message);

  static const QString info_html = "<font color=\"Grey\">";
  static const QString success_html = "<font color=\"Green\">";
  static const QString end_html = "</font>";

  static const QString fail_html = "<b><i><font color=\"Red\">";
  static const QString fail_end_html = "</font></i></b>";

  switch (node.type())
  {
    case BT::NodeType::ACTION:
      switch (status)
      {
        case BT::NodeStatus::RUNNING:
          ss << info_html << "[ " << QString::fromStdString(node.name()) << " ]  ->  RUNNING" << end_html;
          break;
        case BT::NodeStatus::SUCCESS:
          ss << success_html << "[ " << QString::fromStdString(node.name()) << " ]  ->  SUCCESS" << end_html;
          break;
        case BT::NodeStatus::FAILURE:
          ss << fail_html << "[ " << QString::fromStdString(node.name()) << " ]  ->  FAILED" << fail_end_html;
          break;
        default:
          break;
      }
      break;
    case BT::NodeType::CONDITION:
      switch (status)
      {
        case BT::NodeStatus::FAILURE:
          ss << fail_html << "[ " << QString::fromStdString(node.name()) << " ]  ->  FAILED" << fail_end_html;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

  if (!message.isEmpty())
  {
    log_->textCursor().movePosition(QTextCursor::End);
    QMetaObject::invokeMethod(log_, "append", Qt::QueuedConnection, Q_ARG(QString, message));
  }
}

void TextEditLogger::flush()
{
}

}  // namespace snp_application
