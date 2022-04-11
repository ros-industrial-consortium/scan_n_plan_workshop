#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>  // TODO: replace with snp_msgs service interface

void plan(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res)
{
  res->success = true;
  res->message = "TODO: implmement planning server";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("snp_planning_server");
  auto service = node->create_service<std_srvs::srv::Trigger>("tesseract_trigger_motion_plan", &plan);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started SNP motion planning server");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
