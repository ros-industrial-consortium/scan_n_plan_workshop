// Copyright 2019 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Currently works with ros2 client and ros1 server

#ifndef ACTION_BRIDGE__ACTION_BRIDGE_HPP_
#define ACTION_BRIDGE__ACTION_BRIDGE_HPP_

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/callback_queue.h"

#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

template<class ROS1_T, class ROS2_T>
class ActionBridge
{
public:
  using ROS2GoalHandle = typename rclcpp_action::ServerGoalHandle<ROS2_T>;

  using ROS1Goal = typename actionlib::ActionServer<ROS1_T>::Goal;
  using ROS1Feedback = typename actionlib::ActionServer<ROS1_T>::Feedback;
  using ROS1Result = typename actionlib::ActionServer<ROS1_T>::Result;

  using ROS2Goal = typename ROS2_T::Goal;
  using ROS2Feedback = typename ROS2_T::Feedback;
  using ROS2Result = typename ROS2_T::Result;

  using ROS2SendGoalOptions = typename rclcpp_action::Client<ROS2_T>::SendGoalOptions;
  using ROS2ServerSharedPtr = typename rclcpp_action::Server<ROS2_T>::SharedPtr;

  ActionBridge(ros::NodeHandle ros1_node,
               rclcpp::Node::SharedPtr ros2_node,
               const std::string action_name)
    : ros1_node_(ros1_node)
    , ros2_node_(ros2_node)
    , client_(action_name, true)
    , action_is_active_(false)
  {
    server_ = rclcpp_action::create_server<ROS2_T>(ros2_node_->get_node_base_interface(),
                                                   ros2_node_->get_node_clock_interface(),
                                                   ros2_node_->get_node_logging_interface(),
                                                   ros2_node_->get_node_waitables_interface(),
                                                   action_name,
                                                   std::bind(&ActionBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                   std::bind(&ActionBridge::handle_cancel, this, std::placeholders::_1),
                                                   std::bind(&ActionBridge::handle_accepted, this, std::placeholders::_1)
                                                   );

    std::cout << "Looking for a ROS1 action server named " << action_name << std::endl;
    if (!client_.waitForServer(ros::Duration(1,0)))
    {
      std::cout << "ROS1 action server not started yet" << std::endl;
    }
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ROS2Goal> goal)
  {
    (void)uuid;
    if(action_is_active_)
    {
      std::cout << "Already have an action goal, and current implementation only handles one action goal at a time: rejecting.";
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!client_.waitForServer(ros::Duration(1,0)))
    {
      std::cout << "Couldn't find the ROS1 action server: rejecting" << std::endl;
      return rclcpp_action::GoalResponse::REJECT;
    }

//    TODO: Ideally if the ROS1 action server was available but returned a rejected state we'd reject the ROS2 goal here.
//    Not sure how this would work with the Simple Action Server.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<ROS2GoalHandle> goal_handle)
  {
      (void)goal_handle;
      client_.cancelGoal();
      return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<ROS2GoalHandle> goal_handle)
  {
    std::thread{std::bind(&ActionBridge::goal_execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void goal_execute(const std::shared_ptr<ROS2GoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    action_is_active_ = true;
    std::cout << "Sending goal" << std::endl;
    goalhandle_ros2_ = goal_handle;

    ROS1Goal goal1;
    translate_goal_2_to_1(*(goalhandle_ros2_->get_goal()), goal1);

    client_.sendGoal(goal1,
                     std::bind(&ActionBridge::handle_done, this, std::placeholders::_1, std::placeholders::_2),
                     std::bind(&ActionBridge::handle_active, this),
                     std::bind(&ActionBridge::handle_feedback, this, std::placeholders::_1));
  }

  void handle_done(const actionlib::SimpleClientGoalState& state1,
                   const typename ROS1Result::ConstPtr& result1)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "Goal is done" << std::endl;

    auto result2 = std::make_shared<ROS2Result>();
    translate_result_1_to_2(*result2, *result1);

    if (state1.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      std::cout << "State is SUCCEEDED: succeeding" << std::endl;
      goalhandle_ros2_->succeed(result2);
    }
    else if (state1.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      std::cout << "State is PREEMPTED: abort" << std::endl;
      goalhandle_ros2_->canceled(result2);  // TODO: handle differently?
    }
    else if (state1.state_ == actionlib::SimpleClientGoalState::ABORTED)
    {
      std::cout << "State is ABORTED: abort" << std::endl;
      goalhandle_ros2_->abort(result2);
    }
    else if (state1.state_ == actionlib::SimpleClientGoalState::REJECTED)
    {
      std::cout << "State is REJECTED: abort" << std::endl;
      goalhandle_ros2_->abort(result2);  // TODO: This is kind of a weird case since the ROS2 side of the bridge has to accept the action before making a ROS1 client. Consider handling differently.
    }
    else
    {
      std::cout << "Encountered some other (probably bad) state: abort" << std::endl;
      goalhandle_ros2_->abort(result2);
    }

    action_is_active_ = false;
  }

  void handle_active()
  {
    std::cout << "ROS1 server is executing goal" << std::endl;
  }

  void handle_feedback(const typename ROS1Feedback::ConstPtr& feedback1)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto feedback2 = std::make_shared<ROS2Feedback>();
    translate_feedback_1_to_2(*feedback2, *feedback1);
    goalhandle_ros2_->publish_feedback(feedback2);
  }


  static int main(const std::string & action_name, int argc, char * argv[])
  {
    std::string node_name = "reverse_action_bridge_" + action_name;
    std::replace(node_name.begin(), node_name.end(), '/', '_');
    // ROS 1 node
    ros::init(argc, argv, node_name);
    ros::NodeHandle ros1_node;

    // ROS 2 node
    rclcpp::init(argc, argv);
    auto ros2_node = rclcpp::Node::make_shared(node_name);

    ActionBridge<ROS1_T, ROS2_T> action_bridge(ros1_node, ros2_node, action_name);

    // // ROS 1 asynchronous spinner
    ros::AsyncSpinner async_spinner(0);
    async_spinner.start();
    rclcpp::spin(ros2_node);
    ros::shutdown();
    return 0;
  }

private:
  bool action_is_active_;

  std::shared_ptr<ROS2GoalHandle> goalhandle_ros2_;

  ros::NodeHandle ros1_node_;
  rclcpp::Node::SharedPtr ros2_node_;

  //defining ROS1 server
  actionlib::SimpleActionClient<ROS1_T> client_;

  //defining ROS2 client
  ROS2ServerSharedPtr server_;

  std::mutex mutex_;

  static void translate_goal_2_to_1(const ROS2Goal &, ROS1Goal &);
  static void translate_result_1_to_2(ROS2Result &, const ROS1Result &);
  static void translate_feedback_1_to_2(ROS2Feedback &, const ROS1Feedback &);
};

#endif // ACTION_BRIDGE__ACTION_BRIDGE_HPP_
