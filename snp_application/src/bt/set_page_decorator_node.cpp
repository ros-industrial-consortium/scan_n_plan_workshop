#include "bt/set_page_decorator_node.h"
#include "bt/utils.h"

#include <QStackedWidget>

namespace snp_application
{
BT::NodeStatus SetPageDecoratorNode::tick()
{
  auto * stacked_widget = config().blackboard->get<QStackedWidget*>(STACKED_WIDGET_KEY);
  if(!QMetaObject::invokeMethod(stacked_widget, "setEnabled", Qt::QueuedConnection, Q_ARG(bool, true)))
    throw std::runtime_error("Failed to call setEnabled");

  auto index = getBTInput<int>(this, INDEX_PORT_KEY);

  if(!QMetaObject::invokeMethod(stacked_widget, "setCurrentIndex", Qt::QueuedConnection, Q_ARG(int, index)))
    throw std::runtime_error("Failed to call setCurrentIndex");

  BT::NodeStatus status = child()->executeTick();

  switch(status)
  {
  case BT::NodeStatus::RUNNING:
    break;
  default:
    if(!QMetaObject::invokeMethod(stacked_widget, "setEnabled", Qt::QueuedConnection, Q_ARG(bool, false)))
      throw std::runtime_error("Failed to call setEnabled");
  }

  return status;
}

} // namespace snp_application
