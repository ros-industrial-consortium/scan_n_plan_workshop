#include <snp_application/snp_behavior_tree.h>
#include <snp_application/bt/utils.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <behaviortree_ros2/plugins.hpp>

namespace snp_application
{
SnpBlackboard::SnpBlackboard(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr parent) : BT::Blackboard(parent)
{
  // Set the error message key in the blackboard
  set(ERROR_MESSAGE_KEY, "");

  // Lambda for setting blackboard values from ROS2 parameters
  auto update_from_parameters =
      [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
    for (const auto& param : parameters)
    {
      switch (param.get_type())
      {
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
          this->set(param.get_name(), param.as_bool());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
          this->set(param.get_name(), param.as_int());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
          this->set(param.get_name(), param.as_double());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
          this->set(param.get_name(), param.as_string());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
          this->set(param.get_name(), param.as_byte_array());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
          this->set(param.get_name(), param.as_bool_array());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
          this->set(param.get_name(), param.as_integer_array());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
          this->set(param.get_name(), param.as_double_array());
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
          this->set(param.get_name(), param.as_string_array());
          break;
        default:
          continue;
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  };

  parameter_cb_handle_ = node->add_on_set_parameters_callback(update_from_parameters);

  // Declare parameters
  // General BT parameters
  node->declare_parameter<std::vector<std::string>>(BT_FILES_PARAM);
  node->declare_parameter<std::vector<std::string>>(BT_PLUGIN_LIBS_PARAM);
  node->declare_parameter<std::vector<std::string>>(BT_ROS_PLUGIN_LIBS_PARAM);
  node->declare_parameter<std::string>(BT_PARAM);
  node->declare_parameter<std::string>(BT_FREESPACE_PARAM);
  node->declare_parameter<int>(BT_TIMEOUT_PARAM);  // seconds
  // General
  node->declare_parameter<std::string>(FOLLOW_JOINT_TRAJECTORY_ACTION);
  node->declare_parameter<double>(START_STATE_REPLACEMENT_TOLERANCE_PARAM);
  // Home state
  node->declare_parameter<std::vector<std::string>>(HOME_STATE_JOINT_NAMES_PARAM);
  node->declare_parameter<std::vector<double>>(HOME_STATE_JOINT_POSITIONS_PARAM);
  node->declare_parameter<std::string>(FREESPACE_MOTION_GROUP_PARAM);
  // Frames
  node->declare_parameter<std::string>(REF_FRAME_PARAM);
  node->declare_parameter<std::string>(TCP_FRAME_PARAM);
  node->declare_parameter<std::string>(CAMERA_FRAME_PARAM);
  node->declare_parameter<std::string>(CAMERA_TCP_FRAME_PARAM);
  // Scan
  node->declare_parameter<std::string>(SCAN_MOTION_GROUP_PARAM);
  node->declare_parameter<std::string>(SCAN_TRAJ_FILE_PARAM);
  node->declare_parameter<std::string>(SCAN_MESH_FILE_PARAM);
  node->declare_parameter<std::string>(SCAN_TPP_CONFIG_FILE_PARAM);
  // Process
  node->declare_parameter<std::string>(PROCESS_MOTION_GROUP_PARAM);
  node->declare_parameter<std::string>(PROCESS_MESH_FILE_PARAM);
  node->declare_parameter<std::string>(PROCESS_TPP_CONFIG_FILE_PARAM);
  // Industrial Reconstruction
  node->declare_parameter<double>(IR_TSDF_VOXEL_PARAM);
  node->declare_parameter<double>(IR_TSDF_SDF_PARAM);
  node->declare_parameter<double>(IR_RGBD_DEPTH_SCALE_PARAM);
  node->declare_parameter<double>(IR_RGBD_DEPTH_TRUNC_PARAM);
  node->declare_parameter<double>(IR_TRANSLATION_FILTER_DISTANCE);
  node->declare_parameter<double>(IR_ROTATION_FILTER_DISTANCE);
  node->declare_parameter<double>(IR_TSDF_MIN_X_PARAM);
  node->declare_parameter<double>(IR_TSDF_MIN_Y_PARAM);
  node->declare_parameter<double>(IR_TSDF_MIN_Z_PARAM);
  node->declare_parameter<double>(IR_TSDF_MAX_X_PARAM);
  node->declare_parameter<double>(IR_TSDF_MAX_Y_PARAM);
  node->declare_parameter<double>(IR_TSDF_MAX_Z_PARAM);
  node->declare_parameter<bool>(IR_LIVE_PARAM);
  node->declare_parameter<double>(IR_NORMAL_ANGLE_TOL_PARAM);
  node->declare_parameter<double>(IR_NORMAL_X_PARAM);
  node->declare_parameter<double>(IR_NORMAL_Y_PARAM);
  node->declare_parameter<double>(IR_NORMAL_Z_PARAM);
  node->declare_parameter<long>(IR_MIN_FACES_PARAM);
  node->declare_parameter<std::string>(IR_ARCHIVE_DIR_PARAM);
}

std::unique_ptr<BT::BehaviorTreeFactory> generateBehaviorTreeFactory(rclcpp::Node::SharedPtr node)
{
  auto bt_factory = std::make_unique<BT::BehaviorTreeFactory>();

  // Register non-ROS plugins
  {
    auto bt_plugins = snp_application::get_parameter<std::vector<std::string>>(node, BT_PLUGIN_LIBS_PARAM);
    for (const std::string& plugin : bt_plugins)
      bt_factory->registerFromPlugin(std::filesystem::path(plugin));
  }

  // Register ROS plugins
  {
    BT::RosNodeParams ros_params;
    ros_params.nh = node;
    ros_params.wait_for_server_timeout = std::chrono::seconds(0);
    ros_params.server_timeout = std::chrono::seconds(snp_application::get_parameter<int>(node, BT_TIMEOUT_PARAM));

    auto bt_ros_plugins = snp_application::get_parameter<std::vector<std::string>>(node, BT_ROS_PLUGIN_LIBS_PARAM);
    for (const std::string& plugin : bt_ros_plugins)
      RegisterRosNode(*bt_factory, std::filesystem::path(plugin), ros_params);
  }

  auto bt_files = snp_application::get_parameter<std::vector<std::string>>(node, BT_FILES_PARAM);
  if (bt_files.empty())
    throw std::runtime_error("Parameter '" + std::string(BT_FILES_PARAM) + "' is empty");

  for (const std::string& file : bt_files)
    bt_factory->registerBehaviorTreeFromFile(file);

  return bt_factory;
}

}  // namespace snp_application
