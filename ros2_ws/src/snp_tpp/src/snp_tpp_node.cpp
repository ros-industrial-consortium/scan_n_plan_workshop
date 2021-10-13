#include <memory> // std::make_shared(), std::shared_ptr

#include <rclcpp/rclcpp.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>

void callPlanner(const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Request> request,
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

int main(int argc, char **argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Instantiate a ROS2 node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tool_path_planning_server");

  // Initialize the service server
  rclcpp::Service<snp_msgs::srv::GenerateToolPaths>::SharedPtr srvr =
      node->create_service<snp_msgs::srv::GenerateToolPaths>("generate_tool_paths", &callPlanner);

  // Notify that this node is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tool Path Planning Server is ready");

  // Spin to accept service calls until ROS shuts down.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
