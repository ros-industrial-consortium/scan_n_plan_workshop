#pragma once

#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>

namespace noether
{
class ConfigurableTPPPipelineWidget;
}

namespace boost_plugin_loader
{
class PluginLoader;
}

namespace snp_tpp
{
class TPPWidget : public QWidget
{
  Q_OBJECT

public:
  TPPWidget(rclcpp::Node::SharedPtr node, boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

private:
  void callback(const snp_msgs::srv::GenerateToolPaths::Request::SharedPtr req,
                const snp_msgs::srv::GenerateToolPaths::Response::SharedPtr res);

  noether::ConfigurableTPPPipelineWidget* pipeline_widget_{ nullptr };
  rclcpp::Service<snp_msgs::srv::GenerateToolPaths>::SharedPtr server_{ nullptr };
};

}  // namespace snp_tpp
