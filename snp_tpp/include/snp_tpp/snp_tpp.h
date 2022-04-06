/**
* @copyright Copyright (c) 2021, Southwest Research Institute
*
* @par License
* Software License Agreement (Apache License)
* @par
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* http://www.apache.org/licenses/LICENSE-2.0
* @par
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
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
