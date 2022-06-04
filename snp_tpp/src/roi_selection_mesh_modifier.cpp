#include "roi_selection_mesh_modifier.h"
#include "ui_roi_selection_mesh_modifier_widget.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/submesh_extraction/extruded_polygon_mesh_extractor.h>
#include <noether_gui/plugin_interface.h>
#include <tf2_eigen/tf2_eigen.h>

namespace snp_tpp
{
ROISelectionMeshModifier::ROISelectionMeshModifier(noether::ExtrudedPolygonSubMeshExtractor extractor, const Eigen::MatrixX3d& boundary)
  : boundary_(boundary)
  , extractor_(std::move(extractor))
{
}

std::vector<pcl::PolygonMesh> ROISelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  if (boundary_.rows() < 3)
    return { mesh };
  else
    return { extractor_.extract(mesh, boundary_) };
}

ROISelectionMeshModifierWidget::ROISelectionMeshModifierWidget(QWidget* parent)
  : noether::MeshModifierWidget(parent)
  , ui_(new Ui::ROISelectionMeshModifier())
  , node_(std::make_shared<rclcpp::Node>("roi_selection_mesh_modifier"))
  , client_(node_->create_client<rviz_polygon_selection_tool::srv::GetSelection>("get_selection"))
{
  ui_->setupUi(this);
}

noether::MeshModifier::ConstPtr ROISelectionMeshModifierWidget::create() const
{
  noether::ExtrudedPolygonSubMeshExtractor extractor;
  extractor.params.max_cluster_size = ui_->max_cluster_size->value();
  extractor.params.min_cluster_size = ui_->min_cluster_size->value();
  extractor.params.cluster_tolerance = ui_->cluster_tolerance->value();
  extractor.params.plane_distance_threshold = ui_->plane_distance_threshold->value();

  if (!client_->service_is_ready())
    throw std::runtime_error("Service is not available");

  auto request = std::make_shared<rviz_polygon_selection_tool::srv::GetSelection::Request>();
  auto future = client_->async_send_request(request);
  rclcpp::FutureReturnCode ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(3));
  switch (ret)
  {
    case rclcpp::FutureReturnCode::SUCCESS:
      break;
    case rclcpp::FutureReturnCode::TIMEOUT:
      throw std::runtime_error("Service call to '" + std::string(client_->get_service_name()) + "' timed out");
    default:
      throw std::runtime_error("Service call to '" + std::string(client_->get_service_name()) + "' failed");
  }

  rviz_polygon_selection_tool::srv::GetSelection::Response::SharedPtr response = future.get();

  Eigen::MatrixX3d boundary(response->selection.size(), 3);
  for (Eigen::Index i = 0; i < boundary.rows(); ++i)
  {
    Eigen::Vector3d v;
    tf2::fromMsg(response->selection[i].point, v);
    boundary.row(i) = v;
  }

  return std::make_unique<ROISelectionMeshModifier>(extractor, boundary);
}

} // namespace snp_tpp
