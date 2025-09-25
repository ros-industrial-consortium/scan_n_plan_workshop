#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class SNPRasterPlanner;
}

namespace snp_tpp
{
class SNPRasterPlannerWidget : public noether::BaseWidget
{
public:
  SNPRasterPlannerWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::SNPRasterPlanner* ui_;
};

}  // namespace snp_tpp
