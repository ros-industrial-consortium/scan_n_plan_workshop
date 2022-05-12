#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <snp_msgs/srv/execute_motion_plan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>

static const std::string FJT_ACTION = "joint_trajectory_action";
static const std::string MOTION_EXEC_SERVICE = "execute_motion_plan";
static const std::string ENABLE_SERVICE = "robot_enable";
static const std::string JOINT_STATES_TOPIC = "joint_states";

using FJT = control_msgs::action::FollowJointTrajectory;
using FJT_Result = control_msgs::action::FollowJointTrajectory_Result;
using FJT_Goal = control_msgs::action::FollowJointTrajectory_Goal;

bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = FJT_Result::SUCCESSFUL;

void common_goal_response(
  std::shared_future<rclcpp_action::ClientGoalHandle<FJT>::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("SUCCEEDED result code\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Goal was canceled\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<FJT>::SharedPtr,
  const std::shared_ptr<const FJT::Feedback> feedback)
{
//  cout << "feedback->desired.positions :";
//  for (auto & x : feedback->desired.positions) {
//    cout << x << "\t";
//  }
//  cout << endl;
//  cout << "feedback->desired.velocities :";
//  for (auto & x : feedback->desired.velocities) {
//    cout << x << "\t";
//  }
//  cout << endl;
}

class MotionExecNode : public rclcpp::Node
{
public:
  explicit MotionExecNode()
  : Node("motion_execution_node"),
    cb_group_(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    this->fjt_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, FJT_ACTION);

    this->enable_client_ = this->create_client<std_srvs::srv::Trigger>(ENABLE_SERVICE);

    this->server_ = this->create_service<snp_msgs::srv::ExecuteMotionPlan>(
      MOTION_EXEC_SERVICE,
      std::bind(
        &MotionExecNode::executeMotionPlan, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);

    this->joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      JOINT_STATES_TOPIC, 1,
      std::bind(&MotionExecNode::callbackJointState, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "Motion execution node started");
  }

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState latest_joint_state_;
  rclcpp::Time joint_state_time_;

  rclcpp::Service<snp_msgs::srv::ExecuteMotionPlan>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::mutex mtx;

  void callbackJointState(
    const sensor_msgs::msg::JointState::SharedPtr state)
  {
    mtx.lock();
    std::vector<double> zero_vector(6, 0);
    if (state->name.size() > 0) {
      double sum_joints;
      for (int i = 0; i < state->position.size(); i++) {
        sum_joints += std::abs(state->position.at(i));
      }
      if (sum_joints > 0.00) {
        latest_joint_state_ = *state;
        joint_state_time_ = this->get_clock()->now();
      } else {
        RCLCPP_WARN(this->get_logger(), "/joint_states sum to zero");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "/joint_states contains no joint names");
    }
    mtx.unlock();
  }

  void executeMotionPlan(
    const std::shared_ptr<snp_msgs::srv::ExecuteMotionPlan::Request> empRequest,
    const std::shared_ptr<snp_msgs::srv::ExecuteMotionPlan::Response> empResult)
  {
    try {
      // enable robot
      {
        RCLCPP_INFO(this->get_logger(), "Enabling Robot");
        if (!enable_client_->service_is_ready()) {
          throw std::runtime_error("Robot enable server is not available");
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = enable_client_->async_send_request(request);
        future.wait();

        std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
        if (!response->success) {
          throw std::runtime_error("Failed to enable robot: '" + response->message + "'");
        }
      }

      // Check that the server exists
      if (!fjt_client_->action_server_is_ready()) {
        throw std::runtime_error("Action server not available after waiting");
      }

      // Send motion trajectory
      control_msgs::action::FollowJointTrajectory::Goal goal_msg;

      rclcpp_action::Client<FJT>::SendGoalOptions opt;
//      opt.goal_response_callback = bind(common_goal_response, std::placeholders::_1);
//      opt.result_callback = bind(common_result_response, std::placeholders::_1);
//      opt.feedback_callback = bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

      goal_msg.trajectory = empRequest->motion_plan;

      {
        mtx.lock();
        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Duration joint_state_age = current_time - joint_state_time_;
        rclcpp::Duration joint_state_age_thresh(0, 5e7);

        if (joint_state_age < joint_state_age_thresh) {
          goal_msg.trajectory.points.at(0).positions = latest_joint_state_.position;
        }
        mtx.unlock();
      }

      RCLCPP_INFO(
        this->get_logger(), "going to send goal with %d points", goal_msg.trajectory.points.size());
      RCLCPP_INFO(
        this->get_logger(), "multidof joint names size = %d",
        goal_msg.trajectory.joint_names.size());


//      auto trajstr = rosidl_generator_traits::to_yaml(goal_msg.trajectory);
//      std::cout << trajstr;
      RCLCPP_INFO(this->get_logger(), "*** TEST 4 ***");

      auto goal_handle_future = fjt_client_->async_send_goal(goal_msg, opt);
      RCLCPP_INFO(this->get_logger(), "Sending");


      goal_handle_future.wait();
      RCLCPP_INFO(this->get_logger(), "Got future");

      using FJTGoalHandle =
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;
      FJTGoalHandle::SharedPtr goal_handle = goal_handle_future.get();
      uint8_t status = goal_handle->get_status();
      switch (status) {
        case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
        case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
          break;
        default:
          throw std::runtime_error(
                  "Follow joint trajectory action goal was not accepted (code " +
                  std::to_string(status) + ")");
      }
      RCLCPP_INFO(this->get_logger(), "*** TEST 5 ***");

      // Wait for the trajectory to complete
      auto fjt_future = fjt_client_->async_get_result(goal_handle);
      fjt_future.wait();
      RCLCPP_INFO(this->get_logger(), "*** TEST 6 ***");

      // Handle the action result code
      FJTGoalHandle::WrappedResult fjt_wrapper = fjt_future.get();
      switch (static_cast<rclcpp_action::ResultCode>(fjt_wrapper.code)) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          throw std::runtime_error("Follow joint trajectory action call did not succeed");
      }
      RCLCPP_INFO(this->get_logger(), "*** TEST 7 ***");

      // Handle the FJT error code
      switch (fjt_wrapper.result->error_code) {
        case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
          break;
        default:
          throw std::runtime_error(
                  "Follow joint trajectory action did not succeed: '" +
                  fjt_wrapper.result->error_string + "'");
      }

      // Communicate success
      empResult->success = true;
    } catch (const std::exception & ex) {
      empResult->message = ex.what();
      empResult->success = false;
    }
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);


  auto node = std::make_shared<MotionExecNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);


  std::thread executor_thread(std::bind(
      &rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  //executor.spin();
  //rclcpp::shutdown();
  return 0;
}
