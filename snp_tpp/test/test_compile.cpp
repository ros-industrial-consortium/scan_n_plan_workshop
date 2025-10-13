#include <snp_tpp/roi_selection_mesh_modifier.h>
#include <snp_tpp/roi_selection_mesh_modifier_widget.h>
#include <snp_tpp/snp_raster_planner_widget.h>

int main(int argc, char** /*argv*/)
{
  {
    snp_tpp::ROISelectionMeshModifier mod;
  }
  {
    snp_tpp::ROISelectionMeshModifierWidget w;
  }
  {
    snp_tpp::SNPRasterPlannerWidget w;
  }
  return 0;
}
