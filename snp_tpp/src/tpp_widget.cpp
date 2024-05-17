#include <snp_tpp/tpp_widget.h>

#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <pcl/io/vtk_lib_io.h>
#include <QVBoxLayout>
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

  pipeline_widget_ = new noether::ConfigurableTPPPipelineWidget(std::move(loader), this);
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
    std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

    noether::ToolPaths test;
    for(const noether::ToolPaths& foo : tool_paths)
    {
      test.insert(test.end(), foo.begin(), foo.end());
    }

    res->tool_paths = toMsg(test);
    res->success = true;
    res->message = "Success";
  }
  catch (const std::exception& ex)
  {
    res->message = ex.what();
    res->success = false;
  }
}

}  // namespace snp_tpp
