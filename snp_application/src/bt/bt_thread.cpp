#include "bt/bt_thread.h"

namespace snp_application
{
BTThread::BTThread(QObject *parent)
  : QThread(parent), tree({}), result(BT::NodeStatus::IDLE)
{

}

BTThread::BTThread(BT::Tree tree, QObject* parent)
  : QThread(parent), tree(std::move(tree)), result(BT::NodeStatus::IDLE)
{
}

void BTThread::run()
{
  try
  {
    result = tree.tickWhileRunning();
  }
  catch (const std::exception& ex)
  {
    message = ex.what();
    result = BT::NodeStatus::FAILURE;
  }
}

} // namespace snp_application
