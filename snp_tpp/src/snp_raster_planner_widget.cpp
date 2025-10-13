#include <snp_tpp/snp_raster_planner_widget.h>
#include "ui_snp_raster_planner_widget.h"

#include <noether_gui/utils.h>
#include <noether_tpp/serialization.h>

namespace snp_tpp
{
SNPRasterPlannerWidget::SNPRasterPlannerWidget(QWidget* parent)
  : noether::BaseWidget(parent), ui_(new Ui::SNPRasterPlanner())
{
  ui_->setupUi(this);
}

void SNPRasterPlannerWidget::configure(const YAML::Node& config)
{
  {
    auto dir_gen_config = YAML::getMember<YAML::Node>(config, "direction_generator");
    ui_->double_spin_box_rotation_offset->setValue(YAML::getMember<double>(dir_gen_config, "rotation_offset"));
  }
  ui_->double_spin_box_point_spacing->setValue(YAML::getMember<double>(config, "point_spacing"));
  ui_->double_spin_box_line_spacing->setValue(YAML::getMember<double>(config, "line_spacing"));
  ui_->double_spin_box_min_segment_length->setValue(YAML::getMember<double>(config, "min_segment_size"));
}

void SNPRasterPlannerWidget::save(YAML::Node& config) const
{
  config["name"] = "PlaneSlicer";
  config["gui_plugin_name"] = "SNPRaster";
  {
    YAML::Node dir_gen_config;
    dir_gen_config["name"] = "PrincipalAxis";
    dir_gen_config["rotation_offset"] = ui_->double_spin_box_rotation_offset->value();
    config["direction_generator"] = dir_gen_config;
  }
  {
    YAML::Node origin_gen_config;
    origin_gen_config["name"] = "Centroid";
    config["origin_generator"] = origin_gen_config;
  }
  config["line_spacing"] = ui_->double_spin_box_line_spacing->value();
  config["point_spacing"] = ui_->double_spin_box_point_spacing->value();
  config["min_hole_size"] = 0.1;
  config["search_radius"] = 0.1;
  config["min_segment_size"] = ui_->double_spin_box_min_segment_length->value();
  config["bidirectional"] = true;
}

}  // namespace snp_tpp
