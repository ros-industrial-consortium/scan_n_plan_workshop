#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/extruded_polygon_subset_extractor.h>
#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rviz_polygon_selection_tool/srv/get_selection.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

FWD_DECLARE_YAML_STRUCTS()

namespace snp_tpp
{
class ROISelectionMeshModifier : public noether::MeshModifier
{
public:
  ROISelectionMeshModifier();

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

  noether::ExtrudedPolygonSubMeshExtractor extractor;

protected:
  DECLARE_YAML_FRIEND_CLASSES(ROISelectionMeshModifier)

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedPtr client_;
  tf2_ros::Buffer::SharedPtr buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace snp_tpp

FWD_DECLARE_YAML_CONVERT(snp_tpp::ROISelectionMeshModifier)
