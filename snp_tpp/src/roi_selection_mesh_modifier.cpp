#include <snp_tpp/roi_selection_mesh_modifier.h>

#include <noether_tpp/serialization.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace snp_tpp
{
ROISelectionMeshModifier::ROISelectionMeshModifier()
{
  // Initialize RCLPP if not already
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = std::make_shared<rclcpp::Node>("roi_selection_mesh_modifier");
  client_ = node_->create_client<rviz_polygon_selection_tool::srv::GetSelection>("get_selection");
  buffer_ =
      std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}

std::vector<pcl::PolygonMesh> ROISelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Check if the ROI selection service is ready
  if (!client_->service_is_ready())
    throw std::runtime_error("ROI selection service is not available");

  // Call the ROI selection service
  auto request = std::make_shared<rviz_polygon_selection_tool::srv::GetSelection::Request>();
  auto future = client_->async_send_request(request);

  switch (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(3)))
  {
    case rclcpp::FutureReturnCode::SUCCESS:
      break;
    case rclcpp::FutureReturnCode::TIMEOUT:
      throw std::runtime_error("ROI selection service call to '" + std::string(client_->get_service_name()) +
                               "' timed out");
    default:
      throw std::runtime_error("ROI selection service call to '" + std::string(client_->get_service_name()) +
                               "' failed");
  }
  rviz_polygon_selection_tool::srv::GetSelection::Response::SharedPtr response = future.get();

  // Extract the modified meshes
  std::vector<pcl::PolygonMesh> modified_meshes;

  for (const geometry_msgs::msg::PolygonStamped& polygon : response->selection)
  {
    if (polygon.polygon.points.size() < 3)
      continue;

    Eigen::MatrixX3d boundary(polygon.polygon.points.size(), 3);
    for (Eigen::Index i = 0; i < boundary.rows(); ++i)
    {
      // Lookup transform between mesh frame and selection frame
      Eigen::Isometry3d transform = tf2::transformToEigen(buffer_->lookupTransform(
          mesh.header.frame_id, polygon.header.frame_id, tf2::TimePointZero, std::chrono::seconds(1)));

      Eigen::Vector3f v(polygon.polygon.points[i].x, polygon.polygon.points[i].y, polygon.polygon.points[i].z);

      boundary.row(i) = transform * v.cast<double>();
    }

    modified_meshes.push_back(extractor.extract(mesh, boundary));
  }

  return modified_meshes;
}

}  // namespace snp_tpp

namespace YAML
{
/** @cond */
Node convert<snp_tpp::ROISelectionMeshModifier>::encode(const snp_tpp::ROISelectionMeshModifier& val)
{
  Node node;
  node["max_cluster_size"] = val.extractor.extractor.params.max_cluster_size;
  node["min_cluster_size"] = val.extractor.extractor.params.min_cluster_size;
  node["cluster_tolerance"] = val.extractor.extractor.params.cluster_tolerance;
  node["plane_distance_threshold"] = val.extractor.extractor.params.plane_distance_threshold;
  return node;
}

bool convert<snp_tpp::ROISelectionMeshModifier>::decode(const Node& node, snp_tpp::ROISelectionMeshModifier& val)
{
  val.extractor.extractor.params.max_cluster_size = YAML::getMember<double>(node, "max_cluster_size");
  val.extractor.extractor.params.min_cluster_size = YAML::getMember<double>(node, "min_cluster_size");
  val.extractor.extractor.params.cluster_tolerance = YAML::getMember<double>(node, "cluster_tolerance");
  val.extractor.extractor.params.plane_distance_threshold = YAML::getMember<double>(node, "plane_distance_threshold");
  return true;
}
/** @endcond */

}  // namespace YAML
