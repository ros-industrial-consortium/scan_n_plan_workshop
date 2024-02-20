#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace Ui
{
class SNPRasterPlanner;
}

namespace snp_tpp
{
class SNPRasterPlannerWidget : public noether::ToolPathPlannerWidget
{
  Q_OBJECT
public:
  SNPRasterPlannerWidget(QWidget* parent = nullptr);

  noether::ToolPathPlanner::ConstPtr create() const override final;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::SNPRasterPlanner* ui_;
};

}  // namespace snp_tpp
