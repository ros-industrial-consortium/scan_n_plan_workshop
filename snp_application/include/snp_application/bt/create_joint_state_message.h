#pragma once

#include <behaviortree_cpp/action_node.h>
#include <sensor_msgs/msg/joint_state.hpp>

namespace snp_application
{
/**
 * @brief Creates a `sensor_msgs/JointState` message from a set of joint names and joint positions
 * @ingroup bt_plugins
 */
class CreateJointStateMessage : public BT::SyncActionNode
{
public:
  inline static const std::string JOINT_NAMES_INPUT_PORT_KEY = "joint_names";
  inline static const std::string JOINT_POSITIONS_INPUT_PORT_KEY = "joint_positions";
  inline static const std::string JOINT_STATE_OUTPUT_PORT_KEY = "joint_state";
  inline static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<std::string>>(JOINT_NAMES_INPUT_PORT_KEY),
             BT::InputPort<std::vector<double>>(JOINT_POSITIONS_INPUT_PORT_KEY),
             BT::OutputPort<sensor_msgs::msg::JointState>(JOINT_STATE_OUTPUT_PORT_KEY) };
  }

  using BT::SyncActionNode::SyncActionNode;

  BT::NodeStatus tick() override;
};

}  // namespace snp_application
