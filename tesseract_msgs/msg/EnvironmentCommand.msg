# Possible Commands
uint8 ADD_LINK=0
uint8 MOVE_LINK=1
uint8 MOVE_JOINT=2
uint8 REMOVE_LINK=3
uint8 REMOVE_JOINT=4
uint8 CHANGE_LINK_ORIGIN=5
uint8 CHANGE_JOINT_ORIGIN=6
uint8 CHANGE_LINK_COLLISION_ENABLED = 7
uint8 CHANGE_LINK_VISIBILITY=8
uint8 ADD_ALLOWED_COLLISION=9
uint8 REMOVE_ALLOWED_COLLISION=10
uint8 REMOVE_ALLOWED_COLLISION_LINK=11
uint8 UPDATE_JOINT_STATE=12
uint8 ADD_SCENE_GRAPH=13
uint8 CHANGE_JOINT_POSITION_LIMITS=14
uint8 CHANGE_JOINT_VELOCITY_LIMITS=15
uint8 CHANGE_JOINT_ACCELERATION_LIMITS=16
uint8 ADD_KINEMATICS_INFORMATION=17
uint8 REPLACE_JOINT=18
uint8 CHANGE_COLLISION_MARGINS=19

# Specifies the type of command. Dictates which of the fields below are used
uint8 command

# ADD - Adds a link and the associated joint to the environment
tesseract_msgs/Link add_link
tesseract_msgs/Joint add_joint
bool add_replace_allowed

# MOVE_LINK - Changes the joint associated with the link
tesseract_msgs/Joint move_link_joint

# MOVE_JOINT - Changes the parent link associated with the joint
string move_joint_name
string move_joint_parent_link

# REMOVE_LINK - Removes the link from the environment
string remove_link

# REMOVE_JOINT - Removes the joint from the environment
string remove_joint

# REPLACE_JOINT - Replace the joint in the environment
tesseract_msgs/Joint replace_joint

# CHANGE_LINK_ORIGIN - Changes the origin of the link
string change_link_origin_name
geometry_msgs/Pose change_link_origin_pose

# CHANGE_JOINT_ORIGIN - Changes the origin of the joint
string change_joint_origin_name
geometry_msgs/Pose change_joint_origin_pose

# CHANGE_LINK_COLLISION_ENABLED - Enables/Disables collisions for the link
string change_link_collision_enabled_name
bool change_link_collision_enabled_value

# CHANGE_LINK_VISIBILITY - Enables/Disables visibility for the link
string change_link_visibility_name
bool change_link_visibility_value

# ADD_ALLOWED_COLLISION - Adds an allowable collision to the allowable collision matrix
tesseract_msgs/AllowedCollisionEntry add_allowed_collision

# REMOVE_ALLOWED_COLLISION - Removes an allowable collision from the allowable collision matrix
tesseract_msgs/AllowedCollisionEntry remove_allowed_collision

# REMOVE_ALLOWED_COLLISION_LINK
string remove_allowed_collision_link

# UPDATE_JOINT_STATE
sensor_msgs/JointState joint_state

# ADD_SCENE_GRAPH Command
tesseract_msgs/SceneGraph scene_graph
tesseract_msgs/Joint scene_graph_joint
string scene_graph_prefix

# CHANGE_JOINT_POSITION_LIMITS Command
tesseract_msgs/StringLimitsPair[] change_joint_position_limits

# CHANGE_JOINT_VELOCITY_LIMITS Command
tesseract_msgs/StringDoublePair[] change_joint_velocity_limits

# CHANGE_JOINT_ACCELERATION_LIMITS Command
tesseract_msgs/StringDoublePair[] change_joint_acceleration_limits

# ADD_KINEMATICS_INFORMATION Command
tesseract_msgs/KinematicsInformation add_kinematics_information

# CHANGE_COLLISION_MARGINS
tesseract_msgs/CollisionMarginData collision_margin_data
tesseract_msgs/CollisionMarginOverrideType collision_margin_override_type
