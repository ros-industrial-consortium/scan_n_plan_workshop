#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class ROISelectionMeshModifier;
}

namespace snp_tpp
{
class ROISelectionMeshModifierWidget : public noether::BaseWidget
{
public:
  ROISelectionMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

private:
  Ui::ROISelectionMeshModifier* ui_;
};

}  // namespace snp_tpp
