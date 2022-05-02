#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <snp_msgs/srv/execute_motion_plan.hpp>

static const std::string FJT_ACTION = "follow_joint_trajectory";
static const std::string MOTION_EXEC_SERVICE = "execute_motion_plan";
static const std::string ENABLE_SERVICE = "robot_enable";

class MotionExecNode : public rclcpp::Node
{
public:
  explicit MotionExecNode()
    : Node("motion_execution_node"), cb_group_(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    this->fjt_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, FJT_ACTION);

    this->enable_client_ = this->create_client<std_srvs::srv::Trigger>(ENABLE_SERVICE);

    this->server_ = this->create_service<snp_msgs::srv::ExecuteMotionPlan>(
        MOTION_EXEC_SERVICE,
        std::bind(&MotionExecNode::executeMotionPlan, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_);

    RCLCPP_INFO(this->get_logger(), "Motion execution node started");
  }

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client_;

  rclcpp::Service<snp_msgs::srv::ExecuteMotionPlan>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  void executeMotionPlan(const std::shared_ptr<snp_msgs::srv::ExecuteMotionPlan::Request> empRequest,
                         const std::shared_ptr<snp_msgs::srv::ExecuteMotionPlan::Response> empResult)
  {
    try
    {
      // enable robot
      {
        if (!enable_client_->service_is_ready())
          throw std::runtime_error("Robot enable server is not available");

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = enable_client_->async_send_request(request);
        future.wait();

        std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
        if (!response->success)
          throw std::runtime_error("Failed to enable robot: '" + response->message + "'");
      }

      // Check that the server exists
      if (!fjt_client_->action_server_is_ready())
        throw std::runtime_error("Action server not available after waiting");

      // Send motion trajectory
      control_msgs::action::FollowJointTrajectory::Goal fjt;
      fjt.trajectory = empRequest->motion_plan;
      auto fjt_accepted_future = fjt_client_->async_send_goal(fjt);

      // Wait for the goal to be accepted
      fjt_accepted_future.wait();
      using FJTGoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;
      FJTGoalHandle::SharedPtr goal_handle = fjt_accepted_future.get();
      uint8_t status = goal_handle->get_status();
      switch (status)
      {
        case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
        case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
          break;
        default:
          throw std::runtime_error("Follow joint trajectory action goal was not accepted (code " +
                                   std::to_string(status) + ")");
      }

      // Wait for the trajectory to complete
      auto fjt_future = fjt_client_->async_get_result(goal_handle);
      fjt_future.wait();

      // Handle the action result code
      FJTGoalHandle::WrappedResult fjt_wrapper = fjt_future.get();
      switch (static_cast<rclcpp_action::ResultCode>(fjt_wrapper.code))
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          throw std::runtime_error("Follow joint trajectory action call did not succeed");
      }

      // Handle the FJT error code
      switch (fjt_wrapper.result->error_code)
      {
        case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
          break;
        default:
          throw std::runtime_error("Follow joint trajectory action did not succeed: '" +
                                   fjt_wrapper.result->error_string + "'");
      }

      // Communicate success
      empResult->success = true;
    }
    catch (const std::exception& ex)
    {
      empResult->message = ex.what();
      empResult->success = false;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotionExecNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
