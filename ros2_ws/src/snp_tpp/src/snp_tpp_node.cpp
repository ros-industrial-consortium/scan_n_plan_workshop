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

#include <snp_tpp/snp_tpp.h>

#include <functional> // std::bind(), std::placeholders

#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>
#include <tf2_eigen/tf2_eigen.h>

#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

#include <snp_msgs/msg/tool_path.h>
#include <snp_msgs/msg/tool_paths.h>

namespace
{

geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses.reserve(segment.size());
  for (const auto& waypoint : segment)
    pose_array.poses.push_back(tf2::toMsg(waypoint));
  return pose_array;
}

snp_msgs::msg::ToolPath toMsg(const noether::ToolPath& path)
{
  snp_msgs::msg::ToolPath path_msg;
  path_msg.segments.reserve(path.size());
  for (const auto& segment : path)
    path_msg.segments.push_back(toMsg(segment));
  return path_msg;
}

snp_msgs::msg::ToolPaths toMsg(const noether::ToolPaths& paths)
{
  snp_msgs::msg::ToolPaths paths_msg;
  paths_msg.paths.reserve(paths.size());
  for (const auto& path : paths)
    paths_msg.paths.push_back(toMsg(path));
  return paths_msg;
}

}

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
  response->success = true;

  // Convert shape_msgs::Mesh to pcl::PolygonMesh
  pcl::PolygonMesh pcl_mesh;
  if (pcl::io::loadPolygonFile(request->mesh_filename, pcl_mesh) == 0)
  {
    response->success = false;
    response->message = "Could not open mesh file `" + request->mesh_filename + "`";
  }
  else
  {
    // Create a planner
    noether::PlaneSlicerRasterPlanner planner(std::make_unique<noether::PrincipalAxisDirectionGenerator>(),
                                              std::make_unique<noether::FixedOriginGenerator>());

    // Configure the planner
    planner.setLineSpacing(request->line_spacing);
    planner.setMinHoleSize(request->min_hole_size);
    planner.setMinSegmentSize(request->min_segment_length);
    planner.setPointSpacing(request->point_spacing);
    planner.setSearchRadius(request->search_radius);

    // Call the planner
    noether::ToolPaths paths = planner.plan(pcl_mesh);

    // Convert noether::ToolPaths to snp_msgs::msg::ToolPaths
    response->tool_paths = toMsg(paths);

    if (paths.empty())
    {
      response->success = false;
      response->message = "Path generation failed";
    }
  }

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
