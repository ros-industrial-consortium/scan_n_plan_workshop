#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/extruded_polygon_subset_extractor.h>

FWD_DECLARE_YAML_STRUCTS()

namespace snp_tpp
{
class ROISelectionMeshModifier : public noether::MeshModifier
{
public:
  ROISelectionMeshModifier() = default;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

  noether::ExtrudedPolygonSubMeshExtractor extractor;

protected:
  DECLARE_YAML_FRIEND_CLASSES(ROISelectionMeshModifier)
};

}  // namespace snp_tpp

FWD_DECLARE_YAML_CONVERT(snp_tpp::ROISelectionMeshModifier)
