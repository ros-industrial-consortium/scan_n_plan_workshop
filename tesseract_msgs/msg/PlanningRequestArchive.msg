tesseract_msgs/Environment environment
tesseract_msgs/EnvironmentCommand[] commands # Additional Commands to be applied to environment prior to planning

string name # The name of the taskflow to use
string instructions # This should an xml string of the command language instructions
string seed # This should an xml string of the command language instructions (Optional)
bool debug # Enable debug content
bool profile # Enable profiling of the planning request

tesseract_msgs/PlannerProfileRemapping plan_profile_remapping
tesseract_msgs/PlannerProfileRemapping composite_profile_remapping
