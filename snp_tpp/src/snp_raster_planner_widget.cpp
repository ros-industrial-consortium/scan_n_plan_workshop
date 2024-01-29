#include <snp_tpp/snp_raster_planner_widget.h>
#include "ui_snp_raster_planner_widget.h"

#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

#include <yaml-cpp/yaml.h>

namespace snp_tpp
{
SNPRasterPlannerWidget::SNPRasterPlannerWidget(QWidget* parent)
  : noether::ToolPathPlannerWidget(parent), ui_(new Ui::SNPRasterPlanner())
{
    ui_->setupUi(this);
}

void SNPRasterPlannerWidget::configure(const YAML::Node& config)
{
  RasterPlannerWidget::configure(config);
  search_radius_->setValue(getEntry<double>(config, "search_radius"));
  min_segment_size_->setValue(getEntry<double>(config, "min_segment_size"));
}

void SNPRasterPlannerWidget::save(YAML::Node& config) const
{
  RasterPlannerWidget::save(config);
  config["search_radius"] = search_radius_->value();
  config["min_segment_size"] = min_segment_size_->value();
}

noether::ToolPathPlanner::ConstPtr SNPRasterPlannerWidget::create() const
{
  auto dir_gen = std::make_unique<noether::PrincipalAxisDirectionGenerator>(ui_->double_spin_box_rotation_offset->value());
  auto orig_gen = std::make_unique<noether::CentroidOriginGenerator>();
  auto planner = std::make_unique<noether::PlaneSlicerRasterPlanner>(std::move(dir_gen), std::move(orig_gen));
  planner->setLineSpacing(ui_->double_spin_box_line_spacing->value());
  planner->setPointSpacing(ui_->double_spin_box_point_spacing->value());
  planner->setMinHoleSize(0.1);
  planner->setSearchRadius(0.1);
  planner->setMinSegmentSize(ui_->double_spin_box_min_segment_length->value());

  return planner;
}

}  // namespace noether
