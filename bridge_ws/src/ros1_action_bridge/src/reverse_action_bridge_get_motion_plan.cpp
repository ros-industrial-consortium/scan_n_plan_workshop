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

#include <reverse_action_bridge/reverse_action_bridge.hpp>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <tesseract_msgs/GetMotionPlanAction.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include <tesseract_msgs/action/get_motion_plan.hpp>

//template<typename T1, typename T2>
//static void copy_point(const T2 & pt2, T1 & pt1)
//{
//  pt1.positions = pt2.positions;
//  pt1.velocities = pt2.velocities;
//  pt1.accelerations = pt2.accelerations;
//}

//template<typename T1, typename T2>
//static void copy_tolerance(const T2 & tolerance2, T1 & tolerance1)
//{
//  tolerance1.name = tolerance2.name;
//  tolerance1.position = tolerance2.position;
//  tolerance1.velocity = tolerance2.velocity;
//  tolerance1.acceleration = tolerance2.acceleration;
//}

//template<typename T1, typename T2>
//static void copy_tolerances(const T2 & t2, T1 & t1)
//{
//  const size_t num = t2.size();
//  t1.resize(num);
//  for (size_t i = 0; i < num; ++i) {
//    copy_tolerance(t2[i], t1[i]);
//  }
//}

//static void copy_duration_2_to_1(
//  const builtin_interfaces::msg::Duration & duration2,
//  ros::Duration & duration1)
//{
//  duration1.sec = duration2.sec;
//  duration1.nsec = duration2.nanosec;
//}

using GetMotionPlanActionBridge = ActionBridge<tesseract_msgs::GetMotionPlanAction,
    tesseract_msgs::action::GetMotionPlan>;

template<>
void GetMotionPlanActionBridge::translate_goal_2_to_1(
  const ROS2Goal & goal2,
  ROS1Goal & goal1)
{
  goal1.request.environment_state;

//  goal1.trajectory.joint_names = goal2.trajectory.joint_names;
//  const size_t num = goal2.trajectory.points.size();
//  goal1.trajectory.points.resize(num);

//  for (size_t i = 0; i < num; ++i) {
//    copy_point(goal2.trajectory.points[i], goal1.trajectory.points[i]);
//    copy_duration_2_to_1(goal2.trajectory.points[i].time_from_start,
//      goal1.trajectory.points[i].time_from_start);
//  }

//  copy_tolerances(goal2.path_tolerance, goal1.path_tolerance);
//  copy_tolerances(goal2.goal_tolerance, goal1.goal_tolerance);

//  copy_duration_2_to_1(goal2.goal_time_tolerance, goal1.goal_time_tolerance);
}

template<>
void GetMotionPlanActionBridge::translate_result_1_to_2(
  ROS2Result & result2,
  const ROS1Result & result1)
{
//  result2.error_code = result1.error_code;
//  result2.error_string = result1.error_string;
}

template<>
void GetMotionPlanActionBridge::translate_feedback_1_to_2(
  ROS2Feedback & feedback2,
  const ROS1Feedback & feedback1)
{
//  feedback2.joint_names = feedback1.joint_names;
//  copy_point(feedback1.desired, feedback2.desired);
//  copy_point(feedback1.actual, feedback2.actual);
//  copy_point(feedback1.error, feedback2.error);
}

int main(int argc, char * argv[])
{
  return GetMotionPlanActionBridge::main("get_motion_plan", argc, argv);
}

