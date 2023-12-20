#include "bt/progress_decorator_node.h"
#include "bt/utils.h"

#include <QProgressBar>

namespace snp_application
{
BT::NodeStatus ProgressDecoratorNode::tick()
{
  auto progress_bar = config().blackboard->get<QProgressBar*>(PROGRESS_BAR_KEY);
  auto start = getBTInput<int>(this, START_PORT_KEY);
  auto end = getBTInput<int>(this, END_PORT_KEY);

  // Set the initial progress
  QMetaObject::invokeMethod(progress_bar, "setValue", Qt::QueuedConnection, Q_ARG(int, start));

  BT::NodeStatus status = child()->executeTick();
  switch(status)
  {
  case BT::NodeStatus::SUCCESS:
    // Set the final progress
    QMetaObject::invokeMethod(progress_bar, "setValue", Qt::QueuedConnection, Q_ARG(int, end));
    break;
  default:
    break;
  }

  return status;
}

} // namespace snp_application
