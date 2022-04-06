# Header
std_msgs/Header header

Environment environment # (Optional) Override the existing Tesseract Environment
tesseract_msgs/EnvironmentCommand[] commands # (Optional) Additional Commands to be applied to environment prior to trajectory visualization

# Tesseract Command Language Instructions
string instructions

# If instructions are provided this will be ignored
tesseract_msgs/JointState[] joint_trajectory
