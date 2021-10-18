/**
 * @file plotting.cpp
 * @brief Tesseract ROS plotting functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <future>
#include <thread>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/command_language.h>

#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_environment/environment.h>

#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/axis_marker.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
#include <tesseract_visualization/markers/geometry_marker.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

namespace tesseract_rosutils
{
ROSPlotting::ROSPlotting(std::string root_link, std::string topic_namespace)
  : root_link_(root_link), topic_namespace_(topic_namespace)
{
  ros::NodeHandle nh;

  trajectory_pub_ =
      nh.advertise<tesseract_msgs::Trajectory>(topic_namespace + "/display_tesseract_trajectory", 1, true);
  collisions_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_collisions", 1, true);
  arrows_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_arrows", 1, true);
  axes_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_axes", 1, true);
  tool_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_tool_path", 1, true);
}

bool ROSPlotting::isConnected() const { return true; }

void ROSPlotting::waitForConnection(long seconds) const
{
  const ros::WallTime start_time = ros::WallTime::now();
  const ros::WallDuration wall_timeout{ static_cast<double>(seconds) };
  while (ros::ok())
  {
    if (isConnected())
      return;

    if (wall_timeout >= ros::WallDuration(0))
    {
      const ros::WallTime current_time = ros::WallTime::now();
      if ((current_time - start_time) >= wall_timeout)
        return;
    }

    ros::WallDuration(0.02).sleep();
    ros::spinOnce();
  }

  return;
}

void ROSPlotting::plotEnvironment(const tesseract_environment::Environment& /*env*/, std::string /*ns*/) {}

void ROSPlotting::plotEnvironmentState(const tesseract_scene_graph::SceneState& /*state*/, std::string /*ns*/) {}

void ROSPlotting::plotTrajectory(const tesseract_msgs::Trajectory& traj, std::string /*ns*/)
{
  trajectory_pub_.publish(traj);
  ros::spinOnce();
}

void ROSPlotting::plotTrajectory(const tesseract_common::JointTrajectory& traj,
                                 const tesseract_scene_graph::StateSolver& /*state_solver*/,
                                 std::string /*ns*/)
{
  tesseract_msgs::Trajectory msg;

  // Set the joint trajectory message
  toMsg(msg.joint_trajectory, traj);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectory(const tesseract_environment::Environment& env,
                                 const tesseract_planning::Instruction& instruction,
                                 std::string /*ns*/)
{
  tesseract_msgs::Trajectory msg;

  // Set tesseract state information
  toMsg(msg.environment, env);

  // Convert to joint trajectory
  assert(tesseract_planning::isCompositeInstruction(instruction));
  const auto& ci = instruction.as<tesseract_planning::CompositeInstruction>();
  tesseract_common::JointTrajectory traj = tesseract_planning::toJointTrajectory(ci);

  // Set the joint trajectory message
  toMsg(msg.joint_trajectory, traj);

  plotTrajectory(msg);
}

void ROSPlotting::plotMarker(const tesseract_visualization::Marker& marker, std::string ns)
{
  switch (marker.getType())
  {
    case static_cast<int>(tesseract_visualization::MarkerType::ARROW):
    {
      const auto& m = dynamic_cast<const tesseract_visualization::ArrowMarker&>(marker);
      visualization_msgs::MarkerArray msg;
      auto arrow_marker_msg = getMarkerArrowMsg(marker_counter_, root_link_, topic_namespace_, ros::Time::now(), m);
      msg.markers.push_back(arrow_marker_msg);
      arrows_pub_.publish(msg);
      break;
    }
    case static_cast<int>(tesseract_visualization::MarkerType::AXIS):
    {
      const auto& m = dynamic_cast<const tesseract_visualization::AxisMarker&>(marker);
      visualization_msgs::MarkerArray msg =
          getMarkerAxisMsg(marker_counter_, root_link_, topic_namespace_, ros::Time::now(), m.axis, m.getScale());
      axes_pub_.publish(msg);
      break;
    }
    case static_cast<int>(tesseract_visualization::MarkerType::TOOLPATH):
    {
      const auto& m = dynamic_cast<const tesseract_visualization::ToolpathMarker&>(marker);
      std::string prefix_ns = topic_namespace_;
      if (!ns.empty())
        prefix_ns = topic_namespace_ + "/" + ns;

      visualization_msgs::MarkerArray msg;
      long cnt = 0;
      auto time = ros::Time::now();
      for (const auto& s : m.toolpath)
      {
        std::string segment_ns = prefix_ns + "/segment_" + std::to_string(cnt++) + "/poses";
        for (const auto& p : s)
        {
          visualization_msgs::MarkerArray msg_pose =
              getMarkerAxisMsg(marker_counter_, root_link_, segment_ns, time, p, m.scale);
          msg.markers.insert(msg.markers.end(), msg_pose.markers.begin(), msg_pose.markers.end());
        }
      }
      tool_path_pub_.publish(msg);
      break;
    }
    case static_cast<int>(tesseract_visualization::MarkerType::CONTACT_RESULTS):
    {
      const auto& m = dynamic_cast<const tesseract_visualization::ContactResultsMarker&>(marker);
      if (!m.dist_results.empty())
      {
        visualization_msgs::MarkerArray msg =
            getContactResultsMarkerArrayMsg(marker_counter_, root_link_, topic_namespace_, ros::Time::now(), m);
        collisions_pub_.publish(msg);
        ros::spinOnce();
      }
      break;
    }
  }

  ros::spinOnce();
}

void ROSPlotting::plotMarkers(const std::vector<tesseract_visualization::Marker::Ptr>& /*markers*/, std::string /*ns*/)
{
  ROS_ERROR("ROSPlotting: Plotting vector of markers is currently no implemented!");
}

void ROSPlotting::plotToolpath(const tesseract_environment::Environment& env,
                               const tesseract_planning::Instruction& instruction,
                               std::string ns)
{
  tesseract_common::Toolpath toolpath = toToolpath(instruction, env);
  tesseract_visualization::ToolpathMarker marker(toolpath);
  plotMarker(marker, ns);
}

visualization_msgs::MarkerArray ROSPlotting::getMarkerAxisMsg(int& id_counter,
                                                              const std::string& frame_id,
                                                              const std::string& ns,
                                                              const ros::Time& time_stamp,
                                                              const Eigen::Isometry3d& axis,
                                                              const Eigen::Vector3d& scale)
{
  visualization_msgs::MarkerArray msg;
  Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
  Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
  Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
  Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

  auto marker_msg = getMarkerCylinderMsg(
      id_counter, frame_id, ns, time_stamp, position, position + x_axis, Eigen::Vector4d(1, 0, 0, 1), scale(0));
  msg.markers.push_back(marker_msg);

  marker_msg = getMarkerCylinderMsg(
      id_counter, frame_id, ns, time_stamp, position, position + y_axis, Eigen::Vector4d(0, 1, 0, 1), scale(1));
  msg.markers.push_back(marker_msg);

  marker_msg = getMarkerCylinderMsg(
      id_counter, frame_id, ns, time_stamp, position, position + z_axis, Eigen::Vector4d(0, 0, 1, 1), scale(2));
  msg.markers.push_back(marker_msg);
  return msg;
}

void ROSPlotting::clear(std::string ns)
{
  // Remove old arrows
  marker_counter_ = 0;
  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker;
  marker.header.frame_id = root_link_;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(marker);
  collisions_pub_.publish(msg);
  arrows_pub_.publish(msg);
  axes_pub_.publish(msg);
  ros::spinOnce();
}

static void waitForInputAsync(std::string message)
{
  ROS_ERROR("%s", message.c_str());
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void ROSPlotting::waitForInput(std::string message)
{
  std::chrono::microseconds timeout(1);
  std::future<void> future = std::async(std::launch::async, [=]() { waitForInputAsync(message); });
  while (future.wait_for(timeout) != std::future_status::ready)
    ros::spinOnce();
}

const std::string& ROSPlotting::getRootLink() const { return root_link_; }

visualization_msgs::Marker ROSPlotting::getMarkerArrowMsg(int& id_counter,
                                                          const std::string& frame_id,
                                                          const std::string& ns,
                                                          const ros::Time& time_stamp,
                                                          const tesseract_visualization::ArrowMarker& marker)
{
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = time_stamp;
  marker_msg.ns = ns;
  marker_msg.id = ++id_counter;
  marker_msg.type = visualization_msgs::Marker::ARROW;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.pose.position.x = marker.pose.translation().x();
  marker_msg.pose.position.y = marker.pose.translation().y();
  marker_msg.pose.position.z = marker.pose.translation().z();

  Eigen::Quaterniond q(marker.pose.rotation());
  q.normalize();
  marker_msg.pose.orientation.x = q.x();
  marker_msg.pose.orientation.y = q.y();
  marker_msg.pose.orientation.z = q.z();
  marker_msg.pose.orientation.w = q.w();

  marker_msg.scale.x = marker.shaft_length + marker.head_length;
  marker_msg.scale.y = marker.shaft_radius;
  marker_msg.scale.z = marker.shaft_radius;

  marker_msg.color.r = static_cast<float>(marker.material->color(0));
  marker_msg.color.g = static_cast<float>(marker.material->color(1));
  marker_msg.color.b = static_cast<float>(marker.material->color(2));
  marker_msg.color.a = static_cast<float>(marker.material->color(3));

  return marker_msg;
}

visualization_msgs::Marker ROSPlotting::getMarkerCylinderMsg(int& id_counter,
                                                             const std::string& frame_id,
                                                             const std::string& ns,
                                                             const ros::Time& time_stamp,
                                                             const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                             const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                             const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                             double scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time_stamp;
  marker.ns = ns;
  marker.id = ++id_counter;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  double length = scale * std::abs((pt2 - pt1).norm());
  Eigen::Vector3d x, y, z, center;
  z = (pt2 - pt1).normalized();
  center = pt1 + (length / 2.0) * z;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);

  y = z.unitOrthogonal();
  x = (y.cross(z)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Quaterniond q(rot);
  q.normalize();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = length / 20.0;
  marker.scale.y = length / 20.0;
  marker.scale.z = length;

  marker.color.r = static_cast<float>(rgba(0));
  marker.color.g = static_cast<float>(rgba(1));
  marker.color.b = static_cast<float>(rgba(2));
  marker.color.a = static_cast<float>(rgba(3));

  return marker;
}

visualization_msgs::MarkerArray
ROSPlotting::getContactResultsMarkerArrayMsg(int& id_counter,
                                             const std::string& frame_id,
                                             const std::string& ns,
                                             const ros::Time& time_stamp,
                                             const tesseract_visualization::ContactResultsMarker& marker)
{
  visualization_msgs::MarkerArray msg;
  for (unsigned i = 0; i < marker.dist_results.size(); ++i)
  {
    const tesseract_collision::ContactResult& dist = marker.dist_results[i];
    double safety_distance{ 0 };
    if (marker.margin_fn != nullptr)
      safety_distance = marker.margin_fn(dist.link_names[0], dist.link_names[1]);
    else
      safety_distance = marker.margin_data.getPairCollisionMargin(dist.link_names[0], dist.link_names[1]);

    auto base_material = std::make_shared<tesseract_scene_graph::Material>("base_material");
    if (dist.distance < 0)
      base_material->color << 1.0, 0.0, 0.0, 1.0;
    else if (dist.distance < safety_distance)
      base_material->color << 1.0, 1.0, 0.0, 1.0;
    else
      base_material->color << 0.0, 1.0, 0.0, 1.0;

    if (dist.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    {
      tesseract_visualization::ArrowMarker am(dist.transform[0] * dist.nearest_points_local[0],
                                              dist.cc_transform[0] * dist.nearest_points_local[0]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("cc_material");
      am.material->color << 0.0, 0.0, 1.0, 1.0;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }

    if (dist.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
    {
      tesseract_visualization::ArrowMarker am(dist.transform[1] * dist.nearest_points_local[1],
                                              dist.cc_transform[1] * dist.nearest_points_local[1]);
      am.material = std::make_shared<tesseract_scene_graph::Material>("cc_material");
      am.material->color << 0.0, 0.0, 0.5, 1.0;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }

    auto it0 = std::find(marker.link_names.begin(), marker.link_names.end(), dist.link_names[0]);
    auto it1 = std::find(marker.link_names.begin(), marker.link_names.end(), dist.link_names[1]);

    if (it0 != marker.link_names.end() && it1 != marker.link_names.end())
    {
      tesseract_visualization::ArrowMarker am1(dist.nearest_points[0], dist.nearest_points[1]);
      am1.material = base_material;
      auto marker0 = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am1);
      msg.markers.push_back(marker0);

      tesseract_visualization::ArrowMarker am2(dist.nearest_points[1], dist.nearest_points[0]);
      am2.material = base_material;
      auto marker1 = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am2);
      msg.markers.push_back(marker1);
    }
    else if (it0 != marker.link_names.end())
    {
      tesseract_visualization::ArrowMarker am(dist.nearest_points[1], dist.nearest_points[0]);
      am.material = base_material;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }
    else
    {
      tesseract_visualization::ArrowMarker am(dist.nearest_points[0], dist.nearest_points[1]);
      am.material = base_material;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

}  // namespace tesseract_rosutils
