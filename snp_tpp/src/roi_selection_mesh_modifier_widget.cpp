#include <snp_tpp/roi_selection_mesh_modifier_widget.h>
#include "ui_roi_selection_mesh_modifier_widget.h"

#include <noether_tpp/serialization.h>

static const std::string MAX_CLUSTER_SIZE_KEY = "max_cluster_size";
static const std::string MIN_CLUSTER_SIZE_KEY = "min_cluster_size";
static const std::string CLUSTER_TOLERANCE_KEY = "cluster_tolerance";
static const std::string PLANE_DISTANCE_THRESHOLD_KEY = "plane_distance_threshold";

namespace snp_tpp
{
ROISelectionMeshModifierWidget::ROISelectionMeshModifierWidget(QWidget* parent)
  : noether::BaseWidget(parent), ui_(new Ui::ROISelectionMeshModifier())
{
  ui_->setupUi(this);
}

void ROISelectionMeshModifierWidget::configure(const YAML::Node& config)
{
  ui_->max_cluster_size->setValue(YAML::getMember<int>(config, MAX_CLUSTER_SIZE_KEY));
  ui_->min_cluster_size->setValue(YAML::getMember<int>(config, MIN_CLUSTER_SIZE_KEY));
  ui_->cluster_tolerance->setValue(YAML::getMember<double>(config, CLUSTER_TOLERANCE_KEY));
  ui_->plane_distance_threshold->setValue(YAML::getMember<double>(config, PLANE_DISTANCE_THRESHOLD_KEY));
}

void ROISelectionMeshModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "ROISelection";
  config[MAX_CLUSTER_SIZE_KEY] = ui_->max_cluster_size->value();
  config[MIN_CLUSTER_SIZE_KEY] = ui_->min_cluster_size->value();
  config[CLUSTER_TOLERANCE_KEY] = ui_->cluster_tolerance->value();
  config[PLANE_DISTANCE_THRESHOLD_KEY] = ui_->plane_distance_threshold->value();
}

}  // namespace snp_tpp
