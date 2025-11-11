#pragma once

#include <behaviortree_cpp/action_node.h>

namespace snp_application
{
/**
 * @brief Extracts an approach, process, and departure trajectory from a nominal trajectory
 * @details The approach trajectory consists of the first two points of the trajectory.
 * The process trajectory consists of points 1 through n-1.
 * The departure trajectory consists of points n-1 and n
 * @ingroup bt_plugins
 */
class ExtractApproachProcessDepartureTrajectoriesNode : public BT::SyncActionNode
{
public:
  inline static const std::string TRAJECTORY_INPUT_PORT_KEY = "trajectory";
  inline static const std::string APPROACH_OUTPUT_PORT_KEY = "approach";
  inline static const std::string PROCESS_OUTPUT_PORT_KEY = "process";
  inline static const std::string DEPARTURE_OUTPUT_PORT_KEY = "departure";
  static BT::PortsList providedPorts();

  explicit ExtractApproachProcessDepartureTrajectoriesNode(const std::string& instance_name,
                                                           const BT::NodeConfig& config);

protected:
  BT::NodeStatus tick() override;
};

}  // namespace snp_application
