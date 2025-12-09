#pragma once

#include <snp_application/bt/utils.h>

#include <deque>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <industrial_reconstruction_msgs/srv/start_reconstruction.hpp>
#include <industrial_reconstruction_msgs/srv/stop_reconstruction.hpp>
#include <noether_ros/srv/plan_tool_path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <snp_msgs/srv/generate_freespace_motion_plan.hpp>
#include <snp_msgs/srv/add_scan_link.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

namespace snp_application
{
/**
 * @brief Wrapper around the ROS service node that writes error messages to the ERROR_MESSAGE_KEY in the behavior tree
 * blackboard.
 * @ingroup bt_plugins
 */
template <typename T>
class SnpRosServiceNode : public BT::RosServiceNode<T>
{
public:
  using BT::RosServiceNode<T>::RosServiceNode;

  inline BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    std::stringstream ss;
    ss << "Service '" << BT::RosServiceNode<T>::service_name_ << "'";

    switch (error)
    {
      case BT::SERVICE_UNREACHABLE:
        ss << " is unreachable";
        break;
      case BT::SERVICE_TIMEOUT:
        ss << " timed out";
        break;
      case BT::INVALID_REQUEST:
        ss << " was sent an invalid request";
        break;
      case BT::SERVICE_ABORTED:
        ss << " was aborted";
        break;
      default:
        break;
    }

    this->config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());

    return BT::NodeStatus::FAILURE;
  }

  template <typename OutputT>
  inline BT::NodeStatus setOutputAndCheck(const std::string& key, const OutputT& value)
  {
    BT::Result output = BT::RosServiceNode<T>::setOutput(key, value);
    if (!output)
    {
      this->config().blackboard->set(ERROR_MESSAGE_KEY, output.get_unexpected().error());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief Wrapper around the ROS action node that writes error messages to the ERROR_MESSAGE_KEY in the behavior tree
 * blackboard.
 * @ingroup bt_plugins
 */
template <typename T>
class SnpRosActionNode : public BT::RosActionNode<T>
{
public:
  using BT::RosActionNode<T>::RosActionNode;

  inline BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    std::stringstream ss;
    ss << "Action '" << BT::RosActionNode<T>::action_name_ << "' failed: '";

    switch (error)
    {
      case BT::SERVER_UNREACHABLE:
        ss << "server unreachable'";
        break;
      case BT::SEND_GOAL_TIMEOUT:
        ss << "goal timed out'";
        break;
      case BT::GOAL_REJECTED_BY_SERVER:
        ss << "goal rejected by server'";
        break;
      case BT::ACTION_ABORTED:
        ss << "action aborted'";
        break;
      case BT::ACTION_CANCELLED:
        ss << "action cancelled'";
        break;
      case BT::INVALID_GOAL:
        ss << "invalid goal'";
        break;
      default:
        break;
    }

    this->config().blackboard->set(ERROR_MESSAGE_KEY, ss.str());

    return BT::NodeStatus::FAILURE;
  }

  template <typename OutputT>
  inline BT::NodeStatus setOutputAndCheck(const std::string& key, const OutputT& value)
  {
    BT::Result output = BT::RosServiceNode<T>::setOutput(key, value);
    if (!output)
    {
      this->config().blackboard->set(ERROR_MESSAGE_KEY, output.get_unexpected().error());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief Calls a `std_srvs/Trigger` service
 * @ingroup bt_plugins
 */
class TriggerServiceNode : public SnpRosServiceNode<std_srvs::srv::Trigger>
{
public:
  using SnpRosServiceNode<std_srvs::srv::Trigger>::providedPorts;
  using SnpRosServiceNode<std_srvs::srv::Trigger>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `std_srvs/Empty` service
 * @ingroup bt_plugins
 */
class EmptyServiceNode : public SnpRosServiceNode<std_srvs::srv::Empty>
{
public:
  using SnpRosServiceNode<std_srvs::srv::Empty>::providedPorts;
  using SnpRosServiceNode<std_srvs::srv::Empty>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `snp_msgs/GenerateMotionPlan` service
 * @ingroup bt_plugins
 */
class GenerateMotionPlanServiceNode : public SnpRosServiceNode<snp_msgs::srv::GenerateMotionPlan>
{
public:
  inline static const std::string TOOL_PATHS_INPUT_PORT_KEY = "tool_paths";
  inline static const std::string MOTION_GROUP_INPUT_PORT_KEY = "motion_group";
  inline static const std::string TCP_FRAME_INPUT_PORT_KEY = "tcp_frame";
  /*
  inline static const std::string APPROACH_OUTPUT_PORT_KEY = "approach";
  inline static const std::string PROCESS_OUTPUT_PORT_KEY = "process";
  inline static const std::string DEPARTURE_OUTPUT_PORT_KEY = "departure";
  */
  inline static const std::string MOTION_PLANS_OUTPUT_PORT_KEY = "motion_plans";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::vector<snp_msgs::msg::ToolPath>>(TOOL_PATHS_INPUT_PORT_KEY),
                                BT::InputPort<std::string>(MOTION_GROUP_INPUT_PORT_KEY),
                                BT::InputPort<std::string>(TCP_FRAME_INPUT_PORT_KEY),
                                /*
                                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(APPROACH_OUTPUT_PORT_KEY),
                                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(PROCESS_OUTPUT_PORT_KEY),
                                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(DEPARTURE_OUTPUT_PORT_KEY)
                                */
                                BT::OutputPort<std::vector<snp_msgs::msg::RasterMotionPlan>>(MOTION_PLANS_OUTPUT_PORT_KEY),});
  }

  using SnpRosServiceNode<snp_msgs::srv::GenerateMotionPlan>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `snp_msgs/AddScanLink` service
 * @details The function of this service is to add a collision object (e.g., a surface reconstruction mesh) to the
 * motion planning environment
 * @ingroup bt_plugins
 */
class AddScanLinkServiceNode : public SnpRosServiceNode<snp_msgs::srv::AddScanLink>
{
public:
  inline static const std::string MESH_FILE_INPUT_PORT_KEY = "mesh_file";
  inline static const std::string MESH_FRAME_INPUT_PORT_KEY = "mesh_frame";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>(MESH_FILE_INPUT_PORT_KEY),
                                BT::InputPort<std::string>(MESH_FRAME_INPUT_PORT_KEY) });
  }

  using SnpRosServiceNode<snp_msgs::srv::AddScanLink>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `snp_msgs/GenerateFreespaceMotionPlan` service
 * @ingroup bt_plugins
 */
class GenerateFreespaceMotionPlanServiceNode : public SnpRosServiceNode<snp_msgs::srv::GenerateFreespaceMotionPlan>
{
public:
  inline static const std::string MOTION_GROUP_INPUT_PORT_KEY = "motion_group";
  inline static const std::string TCP_FRAME_INPUT_PORT_KEY = "tcp_frame";
  inline static const std::string START_JOINT_STATE_INPUT_PORT_KEY = "start_joint_state";
  inline static const std::string GOAL_JOINT_STATE_INPUT_PORT_KEY = "goal_joint_state";
  inline static const std::string TRAJECTORY_OUTPUT_PORT_KEY = "trajectory";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>(MOTION_GROUP_INPUT_PORT_KEY),
                                BT::InputPort<std::string>(TCP_FRAME_INPUT_PORT_KEY),
                                BT::InputPort<sensor_msgs::msg::JointState>(START_JOINT_STATE_INPUT_PORT_KEY),
                                BT::InputPort<sensor_msgs::msg::JointState>(GOAL_JOINT_STATE_INPUT_PORT_KEY),
                                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) });
  }

  using SnpRosServiceNode<snp_msgs::srv::GenerateFreespaceMotionPlan>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `noether_ros/PlanToolPath` service
 * @ingroup bt_plugins
 */
class PlanToolPathServiceNode : public SnpRosServiceNode<noether_ros::srv::PlanToolPath>
{
public:
  inline static const std::string CONFIG_FILE_INPUT_PORT_KEY = "config_file";
  inline static const std::string MESH_FILE_INPUT_PORT_KEY = "mesh_file";
  inline static const std::string MESH_FRAME_INPUT_PORT_KEY = "mesh_frame";
  inline static const std::string TOOL_PATHS_OUTPUT_PORT_KEY = "tool_paths";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>(CONFIG_FILE_INPUT_PORT_KEY),
                                BT::InputPort<std::string>(MESH_FILE_INPUT_PORT_KEY),
                                BT::InputPort<std::string>(MESH_FRAME_INPUT_PORT_KEY),
                                BT::OutputPort<std::vector<snp_msgs::msg::ToolPath>>(TOOL_PATHS_OUTPUT_PORT_KEY) });
  }

  using SnpRosServiceNode<noether_ros::srv::PlanToolPath>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `industrial_reconstruction_msgs/StartReconstruction` service
 * @ingroup bt_plugins
 */
class StartReconstructionServiceNode
  : public SnpRosServiceNode<industrial_reconstruction_msgs::srv::StartReconstruction>
{
public:
  inline static const std::string CAMERA_FRAME_INPUT_PORT_KEY = "camera_frame";
  inline static const std::string REF_FRAME_INPUT_PORT_KEY = "ref_frame";
  inline static const std::string VOXEL_LENGTH_INPUT_PORT_KEY = "voxel_length";
  inline static const std::string SDF_TRUNC_INPUT_PORT_KEY = "sdf_trunc";
  inline static const std::string DEPTH_SCALE_INPUT_PORT_KEY = "depth_scale";
  inline static const std::string DEPTH_TRUNC_INPUT_PORT_KEY = "depth_trunc";
  inline static const std::string TRANSLATION_FILTER_DISTANCE_KEY = "translation_filter_distance";
  inline static const std::string ROTATION_FILTER_DISTANCE_KEY = "rotation_filter_distance";
  inline static const std::string MIN_X_INPUT_PORT_KEY = "min_x";
  inline static const std::string MIN_Y_INPUT_PORT_KEY = "min_y";
  inline static const std::string MIN_Z_INPUT_PORT_KEY = "min_z";
  inline static const std::string MAX_X_INPUT_PORT_KEY = "max_x";
  inline static const std::string MAX_Y_INPUT_PORT_KEY = "max_y";
  inline static const std::string MAX_Z_INPUT_PORT_KEY = "max_z";
  inline static const std::string LIVE_RECONSTRUCTION_INPUT_PORT_KEY = "live_reconstruction";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>(CAMERA_FRAME_INPUT_PORT_KEY), BT::InputPort<std::string>(REF_FRAME_INPUT_PORT_KEY),
          BT::InputPort<double>(VOXEL_LENGTH_INPUT_PORT_KEY), BT::InputPort<double>(SDF_TRUNC_INPUT_PORT_KEY),
          BT::InputPort<double>(DEPTH_SCALE_INPUT_PORT_KEY), BT::InputPort<double>(DEPTH_TRUNC_INPUT_PORT_KEY),
          BT::InputPort<double>(TRANSLATION_FILTER_DISTANCE_KEY), BT::InputPort<double>(ROTATION_FILTER_DISTANCE_KEY),
          BT::InputPort<double>(MIN_X_INPUT_PORT_KEY), BT::InputPort<double>(MIN_Y_INPUT_PORT_KEY),
          BT::InputPort<double>(MIN_Z_INPUT_PORT_KEY), BT::InputPort<double>(MAX_X_INPUT_PORT_KEY),
          BT::InputPort<double>(MAX_Y_INPUT_PORT_KEY), BT::InputPort<double>(MAX_Z_INPUT_PORT_KEY),
          BT::InputPort<bool>(LIVE_RECONSTRUCTION_INPUT_PORT_KEY) });
  }

  using SnpRosServiceNode<industrial_reconstruction_msgs::srv::StartReconstruction>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Calls a `industrial_reconstruction_msgs/StopReconstruction` service
 * @ingroup bt_plugins
 */
class StopReconstructionServiceNode : public SnpRosServiceNode<industrial_reconstruction_msgs::srv::StopReconstruction>
{
public:
  inline static const std::string ARCHIVE_DIRECTORY_INPUT_PORT_KEY = "archive_directory";
  inline static const std::string MESH_FILE_INPUT_PORT_KEY = "mesh_file";
  inline static const std::string MIN_FACES_INPUT_PORT_KEY = "min_faces";
  inline static const std::string NORMAL_ANGLE_TOLERANCE_INPUT_PORT_KEY = "normal_angle_tolerance";
  inline static const std::string NORMAL_X_INPUT_PORT_KEY = "normal_x";
  inline static const std::string NORMAL_Y_INPUT_PORT_KEY = "normal_y";
  inline static const std::string NORMAL_Z_INPUT_PORT_KEY = "normal_z";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<std::string>(ARCHIVE_DIRECTORY_INPUT_PORT_KEY),
          BT::InputPort<std::string>(MESH_FILE_INPUT_PORT_KEY), BT::InputPort<long>(MIN_FACES_INPUT_PORT_KEY),
          BT::InputPort<double>(NORMAL_ANGLE_TOLERANCE_INPUT_PORT_KEY), BT::InputPort<double>(NORMAL_X_INPUT_PORT_KEY),
          BT::InputPort<double>(NORMAL_Y_INPUT_PORT_KEY), BT::InputPort<double>(NORMAL_Z_INPUT_PORT_KEY) });
  }

  using SnpRosServiceNode<industrial_reconstruction_msgs::srv::StopReconstruction>::SnpRosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Publishes a `geometry_msgs/PoseArray` message (e.g., to visualize a tool path)
 * @ingroup bt_plugins
 */
class ToolPathsPubNode : public BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>
{
public:
  inline static const std::string TOOL_PATHS_INPUT_PORT_KEY = "tool_paths";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::vector<snp_msgs::msg::ToolPath>>(TOOL_PATHS_INPUT_PORT_KEY) });
  }
  using BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>::RosTopicPubNode;

  bool setMessage(geometry_msgs::msg::PoseArray& msg) override;
};

/**
 * @brief Publishes a `trajectory_msgs/JointTrajectory` message (e.g., for visualizing a planned motion)
 * @ingroup bt_plugins
 */
class MotionPlanPubNode : public BT::RosTopicPubNode<trajectory_msgs::msg::JointTrajectory>
{
public:
  inline static const std::string TRAJECTORY_INPUT_PORT_KEY = "trajectory";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY) });
  }
  using BT::RosTopicPubNode<trajectory_msgs::msg::JointTrajectory>::RosTopicPubNode;

  bool setMessage(trajectory_msgs::msg::JointTrajectory& msg) override;
};

/**
 * @brief Calls a `control_msgs/FollowJointTrajectory` action
 * @ingroup bt_plugins
 */
class FollowJointTrajectoryActionNode : public SnpRosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
  inline static const std::string TRAJECTORY_INPUT_PORT_KEY = "trajectory";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY) });
  }
  using SnpRosActionNode<control_msgs::action::FollowJointTrajectory>::SnpRosActionNode;

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
};

/**
 * @brief Updates the start state of the input trajectory with the current state of the robot, assuming the two are
 * within some threshold of one another
 * @ingroup bt_plugins
 */
class UpdateTrajectoryStartStateNode : public BT::SyncActionNode
{
public:
  inline static const std::string START_JOINT_STATE_INPUT_PORT_KEY = "joint_state";
  inline static const std::string TRAJECTORY_INPUT_PORT_KEY = "input_trajectory";
  inline static const std::string REPLACEMENT_TOLERANCE_INPUT_KEY = "replacement_tolerance";
  inline static const std::string TRAJECTORY_OUTPUT_PORT_KEY = "output";
  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<sensor_msgs::msg::JointState>(START_JOINT_STATE_INPUT_PORT_KEY),
             BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY),
             BT::InputPort<double>(REPLACEMENT_TOLERANCE_INPUT_KEY),
             BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) };
  }
  explicit UpdateTrajectoryStartStateNode(const std::string& instance_name, const BT::NodeConfig& config,
                                          rclcpp::Node::SharedPtr node);

protected:
  BT::NodeStatus tick() override;
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Reverses a trajectory
 * @ingroup bt_plugins
 */
class ReverseTrajectoryNode : public BT::SyncActionNode
{
public:
  inline static const std::string TRAJECTORY_INPUT_PORT_KEY = "input";
  inline static const std::string TRAJECTORY_OUTPUT_PORT_KEY = "output";
  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_INPUT_PORT_KEY),
             BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) };
  }
  explicit ReverseTrajectoryNode(const std::string& instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

/**
 * @brief Concatenates two trajectories into a single trajectory
 * @ingroup bt_plugins
 */
class CombineTrajectoriesNode : public BT::SyncActionNode
{
public:
  inline static const std::string FIRST_TRAJECTORY_INPUT_PORT_KEY = "first";
  inline static const std::string SECOND_TRAJECTORY_INPUT_PORT_KEY = "second";
  inline static const std::string TRAJECTORY_OUTPUT_PORT_KEY = "output";
  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>(FIRST_TRAJECTORY_INPUT_PORT_KEY),
             BT::InputPort<trajectory_msgs::msg::JointTrajectory>(SECOND_TRAJECTORY_INPUT_PORT_KEY),
             BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT_PORT_KEY) };
  }
  explicit CombineTrajectoriesNode(const std::string& instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

/**
 * @brief Subscribes to a `sensor_msgs/JointState` topic to extract the current joint state of the robot
 * @ingroup bt_plugins
 */
class GetCurrentJointStateNode : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{
public:
  inline static const std::string JOINT_STATE_OUTPUT_PORT_KEY = "current_state";
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::OutputPort<sensor_msgs::msg::JointState>(JOINT_STATE_OUTPUT_PORT_KEY) });
  }
  using BT::RosTopicSubNode<sensor_msgs::msg::JointState>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg) override;
};

/**
 * @brief Condition node for spinning the BT ROS node to keep it accessible for parameter updates, service calls, etc.
 * @ingroup bt_plugins
 */
class RosSpinnerNode : public BT::ConditionNode
{
public:
  inline static BT::PortsList providedPorts()
  {
    return {};
  }

  explicit RosSpinnerNode(const std::string& instance_name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

protected:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
};

class SplitMotionPlanNode : public BT::SyncActionNode
{
public:
  inline static const std::string MOTION_PLAN_INPUT_PORT_KEY = "motion_plan";
  inline static const std::string APPROACH_OUTPUT_PORT_KEY = "approach";
  inline static const std::string PROCESS_OUTPUT_PORT_KEY = "process";
  inline static const std::string DEPARTURE_OUTPUT_PORT_KEY = "departure";
  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<snp_msgs::msg::RasterMotionPlan>(MOTION_PLAN_INPUT_PORT_KEY),
             BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(APPROACH_OUTPUT_PORT_KEY),
             BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(PROCESS_OUTPUT_PORT_KEY),
             BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(DEPARTURE_OUTPUT_PORT_KEY) };
  }

  explicit SplitMotionPlanNode(const std::string& instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

class VectorToQueueNode : public BT::SyncActionNode
{
public:
  inline static const std::string VECTOR_INPUT_PORT_KEY = "vector";
  inline static const std::string QUEUE_OUTPUT_PORT_KEY = "queue";
  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<snp_msgs::msg::RasterMotionPlan>>(VECTOR_INPUT_PORT_KEY),
             BT::OutputPort<std::deque<snp_msgs::msg::RasterMotionPlan>>(QUEUE_OUTPUT_PORT_KEY) };
  }
  explicit VectorToQueueNode(const std::string* instance_name, const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

}  // namespace snp_application
