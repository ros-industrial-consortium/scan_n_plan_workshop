#include <snp_tpp/roi_selection_mesh_modifier.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2/time.h>
#include <geometry_msgs/msg/point32.hpp>

namespace snp_tpp
{
ROISelectionMeshModifier::ROISelectionMeshModifier(rclcpp::Node::SharedPtr node,
                                                   noether::ExtrudedPolygonSubMeshExtractor extractor,
                                                   std::vector<geometry_msgs::msg::PolygonStamped> boundary)
  : boundary_(std::move(boundary)), extractor_(std::move(extractor)), buffer_(node->get_clock()), listener_(buffer_)
{
}

std::vector<pcl::PolygonMesh> ROISelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  std::vector<pcl::PolygonMesh> modifiedMeshes;

  for(const geometry_msgs::msg::PolygonStamped& inner_vector_ : boundary_)
  {
    if (inner_vector_.polygon.points.size() < 5) // 5 because one single point/dot counts as 2, so a triangle (3 points in space) will count as size 6
    {
      std::cout << "***** Polygon points size is less than 3, size is: " << inner_vector_.polygon.points.size()/2 << " *****" << std::endl;
      continue;
    }
    Eigen::MatrixX3d inner_matrix(inner_vector_.polygon.points.size(), 3);
    for (Eigen::Index i = 0; i < inner_matrix.rows(); ++i)
    {
      geometry_msgs::msg::Point point_;
      // Lookup transform between mesh header and
      Eigen::Isometry3d transform = tf2::transformToEigen(buffer_.lookupTransform(
          mesh.header.frame_id, inner_vector_.header.frame_id, tf2::TimePointZero, std::chrono::seconds(3)));
      // this below to pass from Point32 to Point, to pass Point to tf2::fromMsg, or else it will throw an error
      point_.x = inner_vector_.polygon.points[i].x;
      point_.y = inner_vector_.polygon.points[i].y;
      point_.z = inner_vector_.polygon.points[i].z;
      Eigen::Vector3d v;
      tf2::fromMsg(point_, v);
      inner_matrix.row(i) = transform * v;
    }
    modifiedMeshes.push_back(extractor_.extract(mesh, inner_matrix));
  }
  return modifiedMeshes;
}
}  // namespace snp_tpp
