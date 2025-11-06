#include <snp_application/snp_widget.h>

#include <QMessageBox>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

namespace snp_application
{
class SNPPanel : public rviz_common::Panel
{
public:
  void onInitialize() override
  {
    try
    {
      auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

      // Create a BT blackboard
      // This blackboard declares parameters in the node that correspond directly to entries required in the blackboard
      // When the parameters are updated, the entries in the blackboard also get updated
      auto blackboard = std::make_shared<SnpBlackboard>(node);

      // Use a BT factory generation function that loads BT plugin libraries specified by ROS parameter and registers BT
      // files specified by ROS parameter
      BehaviorTreeFactoryGenerator bt_factory_gen = std::bind(generateBehaviorTreeFactory, node);

      auto* widget = new snp_application::SNPWidget(node, blackboard, bt_factory_gen, this);
      auto layout = new QVBoxLayout(this);
      layout->addWidget(widget);
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  }
};

}  // namespace snp_application

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(snp_application::SNPPanel, rviz_common::Panel)
