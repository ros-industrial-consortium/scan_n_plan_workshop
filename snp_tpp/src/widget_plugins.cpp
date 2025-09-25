#include <snp_tpp/roi_selection_mesh_modifier_widget.h>
#include <snp_tpp/snp_raster_planner_widget.h>

#include <noether_gui/plugin_interface.h>

using namespace noether;

namespace snp_tpp
{
EXPORT_SIMPLE_MESH_MODIFIER_WIDGET_PLUGIN(ROISelectionMeshModifierWidget, ROISelection)
EXPORT_SIMPLE_TOOL_PATH_PLANNER_WIDGET_PLUGIN(SNPRasterPlannerWidget, SNPRaster)
}  // namespace snp_tpp
