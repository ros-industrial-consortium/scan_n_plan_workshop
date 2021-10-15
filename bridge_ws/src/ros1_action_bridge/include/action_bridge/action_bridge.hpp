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
// Currently works with ros2 server and ros1 client

#ifndef ACTION_BRIDGE__ACTION_BRIDGE_HPP_
#define ACTION_BRIDGE__ACTION_BRIDGE_HPP_

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
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
  using ROS1GoalHandle = typename actionlib::ActionServer<ROS1_T>::GoalHandle;
  ActionBridge(
    ros::NodeHandle ros1_node,
    rclcpp::Node::SharedPtr ros2_node,
    const std::string action_name)
  : ros1_node_(ros1_node), ros2_node_(ros2_node),
    server_(ros1_node, action_name,
      std::bind(&ActionBridge::goal_cb, this, std::placeholders::_1),
      std::bind(&ActionBridge::cancel_cb, this, std::placeholders::_1),
      false)
  {
    server_.start();
    //defining ROS2 client
    client_ = rclcpp_action::create_client<ROS2_T>(ros2_node, action_name);
    std::cout << "Looking for a ROS2 action server named " << action_name << std::endl;
    if (!client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      std::cout << "ROS2 action server not started yet" << std::endl;
    }
  }

  void cancel_cb(ROS1GoalHandle gh1)
  {
    // try to find goal and cancel it
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = goals_.find(gh1.getGoalID().id);
    if (it != goals_.end()) {
      std::thread([handler = it->second]() mutable {
          handler->cancel();
        }).detach();
    }
  }
  void goal_cb(ROS1GoalHandle gh1)
  {
    const std::string goal_id = gh1.getGoalID().id;

    // create a new handler for the goal
    std::shared_ptr<GoalHandler> handler;
    handler.reset(new GoalHandler(gh1, client_));
    std::lock_guard<std::mutex> lock(mutex_);
    goals_.insert(std::make_pair(goal_id, handler));

    RCLCPP_INFO(ros2_node_->get_logger(), "Sending goal");
    std::thread([handler, goal_id, this]() mutable {
        // execute the goal remotely
        handler->handle();

        // clean-up
        std::lock_guard<std::mutex> lock(mutex_);
        goals_.erase(goal_id);
      }).detach();
  }

  static int main(const std::string & action_name, int argc, char * argv[])
  {
    std::string node_name = "action_bridge_" + action_name;
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
  using ROS1Goal = typename actionlib::ActionServer<ROS1_T>::Goal;
  using ROS1Feedback = typename actionlib::ActionServer<ROS1_T>::Feedback;
  using ROS1Result = typename actionlib::ActionServer<ROS1_T>::Result;
  using ROS2Goal = typename ROS2_T::Goal;
  using ROS2Feedback = typename ROS2_T::Feedback;
  using ROS2Result = typename ROS2_T::Result;
  using ROS2GoalHandle = typename rclcpp_action::ClientGoalHandle<ROS2_T>::SharedPtr;
  using ROS2ClientSharedPtr = typename rclcpp_action::Client<ROS2_T>::SharedPtr;
  using ROS2SendGoalOptions = typename rclcpp_action::Client<ROS2_T>::SendGoalOptions;


//  using ROS1GoalHandle = typename rclcpp_action::ClientGoalHandle<ROS1_T>::SharedPtr;
//  using ROS1ClientSharedPtr = typename rclcpp_action::Client<ROS1_T>::SharedPtr;


  class GoalHandler
  {
  public:
    void cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      canceled_ = true;
      if (gh2_) { // cancel goal if possible
        auto fut = client_->async_cancel_goal(gh2_);
      }
    }
    void handle()
    {
      auto goal1 = gh1_.getGoal();
      ROS2Goal goal2;
      translate_goal_1_to_2(*gh1_.getGoal(), goal2);

      if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
        std::cout << "Action server not available after waiting" << std::endl;
        gh1_.setRejected();
        return;
      }

      //Changes as per Dashing
      auto send_goal_ops = ROS2SendGoalOptions();
      send_goal_ops.goal_response_callback =
        [this](auto gh2_future) mutable
      {
        auto goal_handle = gh2_future.get();
        if (!goal_handle) {
          std::cout << "Could not find goal_handle" <<std::endl;
          gh1_.setRejected(); // goal was not accepted by remote server
          return;
        }

        gh1_.setAccepted();

        {
          std::lock_guard<std::mutex> lock(mutex_);
          gh2_ = goal_handle;

          if (canceled_) { // cancel was called in between
            auto fut = client_->async_cancel_goal(gh2_);
          }
        }
      };

      send_goal_ops.feedback_callback =
        [this](ROS2GoalHandle, auto feedback2) mutable
      {
        ROS1Feedback feedback1;
        translate_feedback_2_to_1(feedback1, *feedback2);
        gh1_.publishFeedback(feedback1);
      };


      // send goal to ROS2 server, set-up feedback
      auto gh2_future = client_->async_send_goal(goal2, send_goal_ops);

      auto future_result = client_->async_get_result(gh2_future.get());
      auto res2 = future_result.get();

      ROS1Result res1;
      translate_result_2_to_1(res1, *(res2.result));

      std::lock_guard<std::mutex> lock(mutex_);
      if (res2.code == rclcpp_action::ResultCode::SUCCEEDED) {
        gh1_.setSucceeded(res1);
      } else if (res2.code == rclcpp_action::ResultCode::CANCELED) {
        gh1_.setCanceled(res1);
      } else {
        gh1_.setAborted(res1);
      }

    }

    GoalHandler(ROS1GoalHandle & gh1, ROS2ClientSharedPtr & client)
    : gh1_(gh1), gh2_(nullptr), client_(client), canceled_(false) {}

  private:
    ROS1GoalHandle gh1_;
    ROS2GoalHandle gh2_;
    ROS2ClientSharedPtr client_;
    bool canceled_; // cancel was called
    std::mutex mutex_;

  };

  ros::NodeHandle ros1_node_;
  rclcpp::Node::SharedPtr ros2_node_;

  //defining ROS1 server
  actionlib::ActionServer<ROS1_T> server_;
  //defining ROS2 client
  ROS2ClientSharedPtr client_;

  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<GoalHandler>> goals_;

  static void translate_goal_1_to_2(const ROS1Goal &, ROS2Goal &);
  static void translate_result_2_to_1(ROS1Result &, const ROS2Result &);
  static void translate_feedback_2_to_1(ROS1Feedback &, const ROS2Feedback &);

  static void translate_goal_2_to_1(const ROS2Goal &, ROS1Goal &);
  static void translate_result_1_to_2(ROS2Result &, const ROS1Result &);
  static void translate_feedback_1_to_2(ROS2Feedback &, const ROS1Feedback &);
};



#endif // ACTION_BRIDGE__ACTION_BRIDGE_HPP_
