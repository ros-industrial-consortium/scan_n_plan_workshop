tesseract_msgs/EnvironmentState environment_state
tesseract_msgs/EnvironmentCommand[] commands # Additional Commands to be applied to environment prior to planning

string name # The name of the taskflow to use
string instructions # This should an xml string of the command language instructions
string seed # This should an xml string of the command language instructions (Optional)
bool debug # Enable debug content
bool profile # Enable profiling of the planning request

tesseract_msgs/PlannerProfileRemapping plan_profile_remapping
tesseract_msgs/PlannerProfileRemapping composite_profile_remapping

string TRAJOPT_PLANNER_NAME = TrajOptPlanner
string OMPL_PLANNER_NAME = OMPLPlanner
string DESCARTES_PLANNER_NAME = DescartesPlanner
string FREESPACE_PLANNER_NAME = FreespacePlanner
string CARTESIAN_PLANNER_NAME = CartesianPlanner

# Raster planner using freespace planner for transitions
string RASTER_FT_PLANNER_NAME = RasterFTPlanner

# Raster planner using freespace planner for transitions providing dual transitions
string RASTER_FT_DT_PLANNER_NAME = RasterFTDTPlanner

# Raster planner with approach and departure using freespace planner for transitions
string RASTER_FT_WAAD_PLANNER_NAME = RasterFTWAADPlanner

# Raster planner with approach and departure using freespace planner for transitions providing dual transitions
string RASTER_FT_WAAD_DT_PLANNER_NAME = RasterFTWAADDTPlanner

# Raster planner using cartesian planner for transitions
string RASTER_CT_PLANNER_NAME = RasterCTPlanner

# Raster planner using cartesian planner for transitions providing dual transitions
string RASTER_CT_DT_PLANNER_NAME = RasterCTDTPlanner

# Raster planner with approach and departure using cartesian planner for transitions
string RASTER_CT_WAAD_PLANNER_NAME = RasterCTWAADPlanner

# Raster planner with approach and departure using cartesian planner for transitions providing dual transitions
string RASTER_CT_WAAD_DT_PLANNER_NAME = RasterCTWAADDTPlanner

# Raster planner performs global plan first then macro planning using freespace planner for transitions
string RASTER_G_FT_PLANNER_NAME = RasterGFTPlanner

# Raster planner performs global plan first then macro planning using cartesian planner for transitions
string RASTER_G_CT_PLANNER_NAME = RasterGCTPlanner

# Raster only planner using freespace planner for transitions
string RASTER_O_FT_PLANNER_NAME = RasterOFTPlanner

# Raster only planner using cartesian planner for transitions
string RASTER_O_CT_PLANNER_NAME = RasterOCTPlanner

# Raster only planner performs global plan first then macro planning using freespace planner for transitions
string RASTER_O_G_FT_PLANNER_NAME = RasterOGFTPlanner

# Raster only planner performs global plan first then macro planning using cartesian planner for transitions
string RASTER_O_G_CT_PLANNER_NAME = RasterOGCTPlanner
