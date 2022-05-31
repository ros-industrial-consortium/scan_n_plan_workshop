#ifndef ROSCONWINDOW_H
#define ROSCONWINDOW_H

#include <QMainWindow>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
// Messages
#include <geometry_msgs/msg/pose_array.hpp>
#include <open3d_interface_msgs/srv/start_yak_reconstruction.hpp>
#include <open3d_interface_msgs/srv/stop_yak_reconstruction.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
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

  rclcpp::Node::SharedPtr getNode() const
  {
    return node_;
  }

private:
  Ui::ROSConWindow* ui_;
  rclcpp::Node::SharedPtr node_;
  bool past_calibration_;

  const std::string mesh_file_;
  const std::string motion_group_;
  const std::string reference_frame_;
  const std::string tcp_frame_;
  const std::string camera_frame_;
  const trajectory_msgs::msg::JointTrajectory scan_traj_;

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
  rclcpp::Client<snp_msgs::srv::GenerateMotionPlan>::SharedPtr motion_planning_client_;

  rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedPtr motion_execution_client_;

  /**
   * @brief Updates the GUI to reflect the status of the internal state machine
   * @details This method updates elements of the GUI and can only be called from the Qt thread, not in ROS callbacks.
   * To invoke this method from a ROS callback, emit the `updateStatus` signal
   */
  void onUpdateStatus(bool success, QString current_process, QString next_process, unsigned step);
  void onUpdateLog(const QString& message);

  bool scan_complete_{ false };
  snp_msgs::msg::ToolPaths::SharedPtr tool_paths_{ nullptr };
  trajectory_msgs::msg::JointTrajectory::SharedPtr motion_plan_{ nullptr };

  void update_calibration_requirement();
  void observe();
  void run_calibration();
  void get_correlation();
  void install_calibration();
  void reset_calibration();

  // Scan motion and reconstruction
  using FJTResult = rclcpp::Client<snp_msgs::srv::ExecuteMotionPlan>::SharedFuture;
  using StartScanFuture = rclcpp::Client<open3d_interface_msgs::srv::StartYakReconstruction>::SharedFuture;
  using StopScanFuture = rclcpp::Client<open3d_interface_msgs::srv::StopYakReconstruction>::SharedFuture;
  void scan();
  void onScanApproachDone(FJTResult result);
  void onScanStartDone(StartScanFuture result);
  void onScanDone(FJTResult result);
  void onScanStopDone(StopScanFuture result);
  void onScanDepartureDone(FJTResult result);

  void plan_tool_paths();

  void planMotion();
  void onPlanMotionDone(rclcpp::Client<snp_msgs::srv::GenerateMotionPlan>::SharedFuture result);

  void execute();

  void reset();

signals:
  void log(const QString& message);
  void updateStatus(bool success, QString current_process, QString next_process, unsigned step);
};

#endif  // ROSCONWINDOW_H
