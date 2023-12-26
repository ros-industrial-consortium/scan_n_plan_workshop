#include "snp_widget.h"

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
      // Create a new node that will be spun by the behavior tree framework
      auto node = std::make_shared<rclcpp::Node>("snp_application");

      auto layout = new QVBoxLayout();
      layout->addWidget(new snp_application::SNPWidget(node, this));
      setLayout(layout);
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  }
};

} // namespace snp_application

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(snp_application::SNPPanel, rviz_common::Panel)
