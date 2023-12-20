#pragma once

#include <memory>
#include <QThread>
#include <behaviortree_cpp/bt_factory.h>

namespace snp_application
{
class BTThread : public QThread
{
  Q_OBJECT
public:
  BTThread(QObject* parent = nullptr);
  BTThread(BT::Tree tree, QObject* parent = nullptr);

  BT::Tree tree;
  BT::NodeStatus result;
  QString message;

protected:
  void run() override;
};

} // namespace snp_application
