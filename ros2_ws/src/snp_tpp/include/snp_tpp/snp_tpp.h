#ifndef SNP_TPP_H
#define SNP_TPP_H

#include <memory> // std::make_shared(), std::shared_ptr
#include <string> // std::string

#include <rclcpp/rclcpp.hpp>

#include <snp_msgs/srv/generate_tool_paths.hpp>

namespace snp_tpp
{

class TPPNode : public rclcpp::Node
{
public:
  TPPNode(const std::string& name);

  void callPlanner(const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Request> request,
                   const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Response> response);

private:
  rclcpp::Service<snp_msgs::srv::GenerateToolPaths>::SharedPtr srvr_;


};  // class TPPNode

}  // namespace snp_tpp

#endif // SNP_TPP_H
