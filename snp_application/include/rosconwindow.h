#ifndef ROSCONWINDOW_H
#define ROSCONWINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_common/types.h>
// Messages
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <open3d_interface_msgs/srv/start_yak_reconstruction.hpp>
#include <open3d_interface_msgs/srv/stop_yak_reconstruction.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <snp_msgs/srv/execute_motion_plan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Ui
{
class ROSConWindow;
}

class QPushButton;

class ROSConWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit ROSConWindow(QWidget* parent = nullptr);
  ~ROSConWindow();

private:
  Ui::ROSConWindow* ui_;
  std::shared_ptr<rclcpp::Node> node_;
  bool past_calibration_;

  // joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr toolpath_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scan_mesh_pub_;

  // service clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr observe_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr run_calibration_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_correlation_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr install_calibration_client_;

  rclcpp::Client<open3d_interface_msgs::srv::StartYakReconstruction>::SharedPtr start_reconstruction_client_;
  rclcpp::Client<open3d_interface_msgs::srv::StopYakReconstruction>::SharedPtr stop_reconstruction_client_;

  rclcpp::Client<snp_msgs::srv::GenerateToolPaths>::SharedPtr tpp_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr motion_planning_client_;

  rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedPtr motion_execution_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_client_;

  void update_status(bool success, std::string current_process, QPushButton* current_button, std::string next_process,
                     QPushButton* next_button, int step);

  const std::string mesh_filepath_;
  tesseract_common::Toolpath tool_paths_;
  trajectory_msgs::msg::JointTrajectory motion_plan_;

  void update_calibration_requirement();
  void observe();
  void run_calibration();
  void get_correlation();
  void install_calibration();
  void reset_calibration();

  // Scan motion and reconstruction
  using FJTResult = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult;
  using StartScanFuture = rclcpp::Client<open3d_interface_msgs::srv::StartYakReconstruction>::SharedFuture;
  using StopScanFuture = rclcpp::Client<open3d_interface_msgs::srv::StopYakReconstruction>::SharedFuture;
  void scan();
  void onScanApproachDone(const FJTResult& result);
  void onScanStartDone(StartScanFuture result);
  void onScanDone(const FJTResult& result);
  void onScanStopDone(StopScanFuture result);
  void onScanDepartureDone(const FJTResult& result);

  void plan_tool_paths();
  void plan_motion();
  void execute();
  void reset();
};

#endif  // ROSCONWINDOW_H
