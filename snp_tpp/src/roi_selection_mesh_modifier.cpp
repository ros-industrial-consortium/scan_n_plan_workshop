#include <snp_tpp/roi_selection_mesh_modifier.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace snp_tpp
{
ROISelectionMeshModifier::ROISelectionMeshModifier(rclcpp::Node::SharedPtr node,
                                                   noether::ExtrudedPolygonSubMeshExtractor extractor,
                                                   std::vector<geometry_msgs::msg::PolygonStamped> boundaries)
  : boundaries_(std::move(boundaries)), extractor_(std::move(extractor)), buffer_(node->get_clock()), listener_(buffer_)
{
}

std::vector<pcl::PolygonMesh> ROISelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  std::vector<pcl::PolygonMesh> modified_meshes;

  for (const geometry_msgs::msg::PolygonStamped& polygon : boundaries_)
  {
    if (polygon.polygon.points.size() < 3)
      continue;

    Eigen::MatrixX3d boundary(polygon.polygon.points.size(), 3);
    for (Eigen::Index i = 0; i < boundary.rows(); ++i)
    {
      // Lookup transform between mesh header and
      Eigen::Isometry3d transform = tf2::transformToEigen(buffer_.lookupTransform(
          mesh.header.frame_id, polygon.header.frame_id, tf2::TimePointZero, std::chrono::seconds(3)));

      Eigen::Vector3f v(polygon.polygon.points[i].x, polygon.polygon.points[i].y, polygon.polygon.points[i].z);

      boundary.row(i) = transform * v.cast<double>();
    }

    modified_meshes.push_back(extractor_.extract(mesh, boundary));
  }

  return modified_meshes;
}

}  // namespace snp_tpp
