#include <snp_tpp/snp_tpp.h>

#include <functional> // std::bind(), std::placeholders

namespace snp_tpp
{

TPPNode::TPPNode(const std::string& name)
  : rclcpp::Node(name)
{
  srvr_ = this->create_service<snp_msgs::srv::GenerateToolPaths>("generate_tool_paths",
                                                                 std::bind(&TPPNode::callPlanner,
                                                                           this,
                                                                           std::placeholders::_1,
                                                                           std::placeholders::_2));
  return;
}

void TPPNode::callPlanner(const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Request> request,
                          const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Response> response)
{
  // Unpack the request
  // Convert shape_msgs::Mesh to pcl::PolygonMesh
  // Create a planner
  // Configure the planner
  // Call the planner
  // Convert noether::ToolPaths to snp_msgs::msg::ToolPaths
  // Pack the response
  return;
}

} // namespace snp_tpp

int main(int argc, char **argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Instantiate a ROS2 node
  std::shared_ptr<rclcpp::Node> tpp_node = std::make_shared<snp_tpp::TPPNode>("tool_path_planning_server");

  // Notify that this node is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tool Path Planning Server is ready");

  // Spin to accept service calls until ROS shuts down.
  rclcpp::spin(tpp_node);
  rclcpp::shutdown();
  return 0;
}
