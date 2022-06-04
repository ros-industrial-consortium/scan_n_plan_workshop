#pragma once

#include <Eigen/Dense>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/submesh_extraction/extruded_polygon_mesh_extractor.h>
#include <noether_gui/widgets.h>
#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

namespace Ui
{
class ROISelectionMeshModifier;
}

namespace snp_tpp
{
class ROISelectionMeshModifier : public noether::MeshModifier
{
public:
  ROISelectionMeshModifier(noether::ExtrudedPolygonSubMeshExtractor extractor, const Eigen::MatrixX3d& boundary);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

private:
  const Eigen::MatrixX3d boundary_;
  const noether::ExtrudedPolygonSubMeshExtractor extractor_;
};

class ROISelectionMeshModifierWidget : public noether::MeshModifierWidget
{
  Q_OBJECT
public:
  ROISelectionMeshModifierWidget(QWidget* parent = nullptr);

  noether::MeshModifier::ConstPtr create() const override;

private:
  Ui::ROISelectionMeshModifier* ui_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedPtr client_;
};

} // namespace snp_tpp
