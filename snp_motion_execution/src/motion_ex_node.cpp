#include <memory>
#include <functional>
#include <thread>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <snp_msgs/srv/execute_motion_plan.hpp>

class MotionExecNode : public rclcpp::Node
{
public:
  explicit MotionExecNode(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    private_node_ =
        std::make_shared<rclcpp::Node>(name + "_private");  // I'm gonna assign the name name+"_private" to private node

    this->action_client_ =
        rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(private_node_, "follow_"
                                                                                                 "joint_"
                                                                                                 "trajector"
                                                                                                 "y");

    this->serv_client_ = private_node_->create_client<std_srvs::srv::Trigger>("robot_enable");

    this->motion_exec_service_ = this->create_service<snp_msgs::srv::ExecuteMotionPlan>(
        "execute_motion_plan",
        std::bind(&MotionExecNode::executeMotionPlan, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Node started2");
  }

  rclcpp::Node::SharedPtr getprivateNode() const
  {
    return private_node_;
  }

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr serv_client_;

  rclcpp::Service<snp_msgs::srv::ExecuteMotionPlan>::SharedPtr motion_exec_service_;
  rclcpp::Node::SharedPtr private_node_;  // declaration this is the fre-shared, undeined pointer

  void executeMotionPlan(const std::shared_ptr<snp_msgs::srv::ExecuteMotionPlan::Request> empRequest,
                         const std::shared_ptr<snp_msgs::srv::ExecuteMotionPlan::Response> empResult)
  {
    // get info from service
    control_msgs::action::FollowJointTrajectory::Goal fjt;
    fjt.trajectory = empRequest->motion_plan;

    // enable robot
    std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = serv_client_->async_send_request(request);

    result.wait();
    if (result.get()->success)
    {
      if (!action_client_->wait_for_action_server())
      {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }
      // send motion trajectory

      //      auto send_goal_options =
      //      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
      //      send_goal_options.goal_response_callback =
      //          std::bind(&MotionExecNode::goal_response_callback, this, std::placeholders::_1);
      //      send_goal_options.feedback_callback =
      //          std::bind(&MotionExecNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      //      send_goal_options.result_callback =
      //          std::bind(&MotionExecNode::result_callback, this, std::placeholders::_1);

      auto future = action_client_->async_send_goal(fjt);  // ,send_goal_options);

      if (future.wait_for(std::chrono::duration<double>(50)) == std::future_status::timeout)
      {
        throw std::runtime_error("Timed out waiting for goal response");
      }
      auto goal_handle = future.get();
      auto resultFuture = action_client_->async_get_result(goal_handle);

      if (resultFuture.wait_for(std::chrono::duration<double>(50)) == std::future_status::timeout)
      {
        throw std::runtime_error("Timed out waiting for goal result");
      }
      auto result = resultFuture.get().result;

      if (result->error_code == result->SUCCESSFUL)
      {
        empResult->success = true;
      }
      else
      {  // trajectory failure
        RCLCPP_ERROR(this->get_logger(), "Failed to execute trajectory");
        empResult->success = false;
        return;
      }
    }
    else
    {
      // enable failure
      RCLCPP_ERROR(this->get_logger(), "Failed to enable robot");
      empResult->success = false;
      return;
    }
  }

  void goal_response_callback(
      std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr>
          future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
      const std::shared_ptr<
          const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::Feedback> /*feedback*/)
  {
    RCLCPP_INFO(this->get_logger(), "feedback");
  }

  void result_callback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Result received");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotionExecNode>("execute_motion_plan_node");
  auto private_node = node->getprivateNode();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(private_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
