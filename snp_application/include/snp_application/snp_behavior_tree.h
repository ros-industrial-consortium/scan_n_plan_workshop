#pragma once

#include <rclcpp/node.hpp>

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>

// Parameters
// Behavior Tree
/**
 * @brief Parameter defining the location of standard behavior tree plugin libraries
 * @ingroup bt_params
 */
inline const char* BT_PLUGIN_LIBS_PARAM = "bt_plugin_libs";
/**
 * @brief Parameter defining the location of ROS2-enabled behavior tree plugin libraries
 * @ingroup bt_params
 */
inline const char* BT_ROS_PLUGIN_LIBS_PARAM = "bt_ros_plugin_libs";
/**
 * @brief Parameter defining the location of behavior tree XML files to add to the behavior tree factory
 * @ingroup bt_params
 */
inline const char* BT_FILES_PARAM = "bt_files";
/**
 * @brief Service/action timeout (s) for ROS2 behavior tree nodes
 * @ingroup bt_params
 */
inline const char* BT_TIMEOUT_PARAM = "bt_timeout";
/**
 * @brief Parameter defining the name of the primary behavior tree to be run by the SNP application
 * @ingroup bt_params
 */
inline const char* BT_PARAM = "tree";
/**
 * @brief Parameter defining the name of the behavior tree used to perform freespace motions (e.g., for moving the robot
 * to a home position)
 * @ingroup bt_params
 */
inline const char* BT_FREESPACE_PARAM = "freespace_tree";
// General
/**
 * @brief Parameter defining the name of the follow joint trajectory action
 * @ingroup bt_params
 */
inline const char* FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory_action";
/**
 * @brief Parameter defining the per-joint angle tolerance (in radians) with which the start state of a trajectory can
 * be replaced with the current state of the robot
 * @ingroup bt_params
 */
inline const char* START_STATE_REPLACEMENT_TOLERANCE_PARAM = "start_state_replacement_tolerance";
/**
 * @brief Parameter defining the motion group to be used for freespace moves
 * @ingroup bt_params
 */
inline const char* FREESPACE_MOTION_GROUP_PARAM = "freespace_motion_group";
// Home state
/**
 * @brief Parameter defining the joint names of the robot when moving to the "home" position
 * @ingroup bt_params
 */
inline const char* HOME_STATE_JOINT_NAMES_PARAM = "home_state_joint_names";
/**
 * @brief Parameter defining the positions of the joints of the robot at the "home" position
 * @ingroup bt_params
 */
inline const char* HOME_STATE_JOINT_POSITIONS_PARAM = "home_state_joint_positions";
// Frames
/**
 * @brief Parameter defining the Cartesian reference frame of the application, to which all Cartesian waypoints are
 * relative
 * @ingroup bt_params
 */
inline const char* REF_FRAME_PARAM = "reference_frame";
/**
 * @brief Parameter defining the process motion tool center point (TCP) frame
 * @ingroup bt_params
 */
inline const char* TCP_FRAME_PARAM = "tcp_frame";
/**
 * @brief Parameter defining the calibrated camera optical frame
 * @details This frame represents the calibrated camera optical frame.
 * This frame is not expected to necessarily be part of the URDF; it may be published by an external node or be attached
 * to a frame that is published by an external node (e.g., robot driver)
 * @ingroup bt_params
 */
inline const char* CAMERA_FRAME_PARAM = "camera_frame";
/**
 * @brief Parameter defining the camera tool center point (TCP) frame
 * @details This frame represents a frame on the camera (typically colocated with the calibrated camera optical frame)
 * used for motion planning purposes. This frame is expected to be defined in the URDF such that it can be used by name
 * by the motion planning environment.
 * @ingroup bt_params
 */
inline const char* CAMERA_TCP_FRAME_PARAM = "camera_tcp_frame";
// Scan
/**
 * @brief Parameter defining the motion group used for scan motions
 * @ingroup bt_params
 */
inline const char* SCAN_MOTION_GROUP_PARAM = "scan_motion_group";
/**
 * @brief Parameter defining a pre-defined trajectory used for scanning
 * @ingroup bt_params
 */
inline const char* SCAN_TRAJ_FILE_PARAM = "scan_trajectory_file";
/**
 * @brief Parameter defining a mesh file used for tool path generation for scan tool paths
 * @ingroup bt_params
 */
inline const char* SCAN_MESH_FILE_PARAM = "scan_mesh_file";
/**
 * @brief Parameter defining the tool path planning configuration file used for generating scan tool paths
 * @ingroup bt_params
 */
inline const char* SCAN_TPP_CONFIG_FILE_PARAM = "scan_tpp_config_file";
// Process
/**
 * @brief Parameter defining the motion group used for process motions
 * @ingroup bt_params
 */
inline const char* PROCESS_MOTION_GROUP_PARAM = "process_motion_group";
/**
 * @brief Parameter defining the mesh file that represents the mesh reconstruction used to drive process motions
 * @ingroup bt_params
 */
inline const char* PROCESS_MESH_FILE_PARAM = "process_mesh_file";
/**
 * @brief Parameter defining the tool path planning configuration file used to generate process tool paths
 * @ingroup bt_params
 */
inline const char* PROCESS_TPP_CONFIG_FILE_PARAM = "process_tpp_config_file";
// Industrial Reconstruction
/**
 * @brief Parameter defining the voxel length (m) of the TSDF reconstruction volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_VOXEL_PARAM = "ir_voxel_length";
/**
 * @brief Parameter defining the SDF truncation length (m) of the TSDF reconstruction volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_SDF_PARAM = "ir_sdf_trunc";
/**
 * @brief Parameter defining the x-value of the minimum point of the TSDF volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_MIN_X_PARAM = "ir_min_x";
/**
 * @brief Parameter defining the y-value of the minimum point of the TSDF volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_MIN_Y_PARAM = "ir_min_y";
/**
 * @brief Parameter defining the z-value of the minimum point of the TSDF volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_MIN_Z_PARAM = "ir_min_z";
/**
 * @brief Parameter defining the x-value of the maximum point of the TSDF volume @ingroup bt_params */
inline const char* IR_TSDF_MAX_X_PARAM = "ir_max_x";
/**
 * @brief Parameter defining the y-value of the maximum point of the TSDF volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_MAX_Y_PARAM = "ir_max_y";
/**
 * @brief Parameter defining the z-value of the maximum point of the TSDF volume
 * @ingroup bt_params
 */
inline const char* IR_TSDF_MAX_Z_PARAM = "ir_max_z";
/**
 * @brief Parameter defining the depth scale factor used to convert depth image pixel values to meters
 * @ingroup bt_params
 */
inline const char* IR_RGBD_DEPTH_SCALE_PARAM = "ir_depth_scale";
/**
 * @brief Parameter defining the depth truncation distance (m) of points in depth images
 *  @ingroup bt_params
 */
inline const char* IR_RGBD_DEPTH_TRUNC_PARAM = "ir_depth_trunc";
/**
 * @brief Parameter defining a flag for running the reconstruction process with "live" feedback
 * @ingroup bt_params
 */
inline const char* IR_LIVE_PARAM = "ir_live";
/**
 * @brief Parameter defining the minimum number of faces required by the reconstruction mesh
 * @ingroup bt_params
 */
inline const char* IR_MIN_FACES_PARAM = "ir_min_faces";
/** @brief Parameter defining the angle tolerance (radians) of each face of the reconstruction mesh with the reference
 * normal vector.
 *  @details Faces with normals that deviate from the reference normal by an angle greater than the tolerance are
 * removed from the reconstruction mesh. A value less than 0 disables this functionality.
 *  @ingroup bt_params
 */
inline const char* IR_NORMAL_ANGLE_TOL_PARAM = "ir_normal_angle_tol";
/**
 * @brief Parameter defining the x-direction of the reference normal used for face normal filtering
 * @ingroup bt_params
 */
inline const char* IR_NORMAL_X_PARAM = "ir_normal_x";
/**
 * @brief Parameter defining the y-direction of the reference normal used for face normal filtering
 * @ingroup bt_params
 */
inline const char* IR_NORMAL_Y_PARAM = "ir_normal_y";
/**
 * @brief Parameter defining the z-direction of the reference normal used for face normal filtering
 * @ingroup bt_params
 */
inline const char* IR_NORMAL_Z_PARAM = "ir_normal_z";
/**
 * @brief Parameter defining the archive directory used for saving reconstruction data (e.g., RGB-D images, etc.)
 * @ingroup bt_params
 */
inline const char* IR_ARCHIVE_DIR_PARAM = "ir_archive_dir";

namespace snp_application
{
/**
 * @brief A behavior tree blackboard populated by ROS parameters
 * @details This blackboard declares parameters in a given ROS node that correspond directly to entries required in the
 * blackboard by the SNP behavior tree nodes. When the ROS parameters are updated, the entries in the blackboard also
 * get updated.
 * @ingroup bt_params
 */
class SnpBlackboard : public BT::Blackboard
{
public:
  SnpBlackboard(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr parent = {});

protected:
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
};

/**
 * @brief Function interface for generating a behavior tree factory
 */
using BehaviorTreeFactoryGenerator = std::function<std::unique_ptr<BT::BehaviorTreeFactory>()>;

/**
 * @brief Function for generating a behavior tree factory using a ROS node to load plugins and behavior tree files from
 * parameters.
 * @details This function uses the node to:
 * 1. Load standard BT plugins from the parameter BT_PLUGIN_LIBS_PARAM
 * 2. Load ROS2 BT plugins from the parameter BT_ROS_PLUGIN_LIBS_PARAM.
 * The node provided to this function is used to create/operate all ROS interfaces required by any generated behavior
 * tree nodes. The service/action timeout specified by the parameter BT_TIMEOUT_PARAM.
 * 3. Load behavior tree files specified in the parameter BT_FILES_PARAM
 */
std::unique_ptr<BT::BehaviorTreeFactory> generateBehaviorTreeFactory(rclcpp::Node::SharedPtr node);

}  // namespace snp_application
