#include <snp_tpp/tpp_widget.h>

#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <pcl/io/vtk_lib_io.h>
#include <QVBoxLayout>
#include <QAction>
#include <QToolBar>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <yaml-cpp/yaml.h>

namespace
{
geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses.reserve(segment.size());
  for (auto waypoint : segment)
  {
    // Renormalize orientation
    Eigen::Quaterniond q(waypoint.linear());
    q.normalize();
    waypoint.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();

    pose_array.poses.push_back(tf2::toMsg(waypoint));
  }
  return pose_array;
}

snp_msgs::msg::ToolPath toMsg(const noether::ToolPath& path)
{
  snp_msgs::msg::ToolPath path_msg;
  path_msg.segments.reserve(path.size());
  for (const auto& segment : path)
    path_msg.segments.push_back(toMsg(segment));
  return path_msg;
}

std::vector<snp_msgs::msg::ToolPath> toMsg(const noether::ToolPaths& paths)
{
  std::vector<snp_msgs::msg::ToolPath> paths_msg;
  paths_msg.reserve(paths.size());
  for (const auto& path : paths)
    paths_msg.push_back(toMsg(path));
  return paths_msg;
}

}  // namespace

namespace snp_tpp
{
TPPWidget::TPPWidget(rclcpp::Node::SharedPtr node, boost_plugin_loader::PluginLoader&& loader, QWidget* parent)
  : QWidget(parent)
{
  auto layout = new QVBoxLayout(this);

  // Create toolbar
  auto* tool_bar = new QToolBar(this);

  // Create "Load TPP configuration" action
  auto* load_action = new QAction(QIcon::fromTheme("document-properties"), "Load TPP configuration...", this);
  load_action->setToolTip("Load TPP configuration from file...");
  load_action->setShortcut(QKeySequence("Ctrl+C"));  // Match shortcut in noether
  tool_bar->addAction(load_action);

  // Create "Save TPP configuration" action
  auto* save_action = new QAction(QIcon::fromTheme("document-save"), "Save TPP configuration...", this);
  save_action->setToolTip("Save TPP configuration...");
  save_action->setShortcut(QKeySequence("Ctrl+S"));  // Match shortcut in noether
  tool_bar->addAction(save_action);

  layout->addWidget(tool_bar);

  // Set the default configuration file directory for the tool path planner
  std::string default_configuration_file_directory;
  node->declare_parameter("default_configuration_file_directory", default_configuration_file_directory);
  node->get_parameter("default_configuration_file_directory", default_configuration_file_directory);

  pipeline_widget_ =
      new noether::ConfigurableTPPPipelineWidget(std::move(loader), default_configuration_file_directory, this);
  layout->addWidget(pipeline_widget_);

  // Set up the ROS interfaces
  server_ = node->create_service<snp_msgs::srv::GenerateToolPaths>(
      "generate_tool_paths", std::bind(&TPPWidget::callback, this, std::placeholders::_1, std::placeholders::_2));

  // Load a parameter-specified configuration file for the tool path planner
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->get_parameter("config_file", config_file);
  if (!config_file.empty())
    pipeline_widget_->configure(QString::fromStdString(config_file));

  param_callback_ =
      node->add_on_set_parameters_callback(std::bind(&TPPWidget::parametersCallback, this, std::placeholders::_1));

  connect(load_action, &QAction::triggered, pipeline_widget_,
          &noether::ConfigurableTPPPipelineWidget::onLoadConfiguration);
  connect(save_action, &QAction::triggered, pipeline_widget_,
          &noether::ConfigurableTPPPipelineWidget::onSaveConfiguration);
}

rcl_interfaces::msg::SetParametersResult TPPWidget::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters)
  {
    if (param.get_name() == "config_file")
    {
      const std::string config_file = param.as_string();
      if (!config_file.empty())
      {
        pipeline_widget_->configure(QString::fromStdString(config_file));
      }
    }
  }

  return result;
}

/**
 * @details Adapted from https://en.cppreference.com/w/cpp/error/throw_with_nested
 */
static void printException(const std::exception& e, std::ostream& ss, int level = 0)
{
  ss << std::string(level * 4, ' ') << e.what() << '\n';
  try
  {
    std::rethrow_if_nested(e);
  }
  catch (const std::exception& nested_exception)
  {
    printException(nested_exception, ss, level + 1);
  }
}

void TPPWidget::callback(const snp_msgs::srv::GenerateToolPaths::Request::SharedPtr req,
                         const snp_msgs::srv::GenerateToolPaths::Response::SharedPtr res)
{
  try
  {
    noether::ToolPathPlannerPipeline pipeline = pipeline_widget_->createPipeline();

    // Load the mesh
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(req->mesh_filename, mesh) < 1)
      throw std::runtime_error("Failed to load mesh");
    mesh.header.frame_id = req->mesh_frame;

    // Plan the tool paths
    std::vector<noether::ToolPaths> tool_paths_vector = pipeline.plan(mesh);

    // Concatenate the tool paths from each sub-mesh into a single tool paths object
    noether::ToolPaths tool_paths_concat;
    for (const noether::ToolPaths& tool_paths : tool_paths_vector)
      tool_paths_concat.insert(tool_paths_concat.end(), tool_paths.begin(), tool_paths.end());

    res->tool_paths = toMsg(tool_paths_concat);
    res->success = true;
    res->message = "Success";
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    printException(ex, ss);
    res->message = ss.str();
    res->success = false;
  }
}

}  // namespace snp_tpp
