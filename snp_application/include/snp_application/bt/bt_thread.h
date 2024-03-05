#pragma once

#include <QThread>
#include <behaviortree_cpp/bt_factory.h>

namespace snp_application
{
/**
 * @brief Thread used to tick a behavior tree
 */
class BTThread : public QThread
{
public:
  BTThread(QObject* parent = nullptr);
  BTThread(BT::Tree tree, QObject* parent = nullptr);

  BT::Tree tree;
  BT::NodeStatus result;
  QString message;

protected:
  void run() override;
};

}  // namespace snp_application
