#include <snp_application/bt/create_joint_state_message.h>
#include <snp_application/bt/utils.h>

namespace snp_application
{
BT::NodeStatus CreateJointStateMessage::tick()
{
  sensor_msgs::msg::JointState msg;
  msg.name = getBTInput<std::vector<std::string>>(this, JOINT_NAMES_INPUT_PORT_KEY);
  msg.position = getBTInput<std::vector<double>>(this, JOINT_POSITIONS_INPUT_PORT_KEY);

  BT::Result res = setOutput(JOINT_STATE_OUTPUT_PORT_KEY, msg);
  if (!res)
  {
    config().blackboard->set(ERROR_MESSAGE_KEY, res.get_unexpected().error());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace snp_application
