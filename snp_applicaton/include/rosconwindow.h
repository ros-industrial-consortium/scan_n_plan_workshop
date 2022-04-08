#ifndef ROSCONWINDOW_H
#define ROSCONWINDOW_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <QMainWindow>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <open3d_interface_msgs/srv/start_yak_reconstruction.hpp>
#include <open3d_interface_msgs/srv/stop_yak_reconstruction.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <snp_msgs/srv/generate_robot_program.hpp>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_common/types.h>
#include <tesseract_msgs/srv/get_motion_plan.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>

namespace Ui
{
class ROSConWindow;
}

class ROSConWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit ROSConWindow(QWidget* parent = nullptr);
  ~ROSConWindow();

private:
  Ui::ROSConWindow* ui_;
  std::shared_ptr<rclcpp::Node> node_;
  bool sim_robot_;
  bool past_calibration_;

  // joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr toolpath_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scan_mesh_pub_;
  rclcpp::Publisher<tesseract_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

  // service clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr observe_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr run_calibration_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_correlation_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr install_calibration_client_;

  rclcpp::Client<open3d_interface_msgs::srv::StartYakReconstruction>::SharedPtr start_reconstruction_client_;
  rclcpp::Client<open3d_interface_msgs::srv::StopYakReconstruction>::SharedPtr stop_reconstruction_client_;

  rclcpp::Client<snp_msgs::srv::GenerateToolPaths>::SharedPtr tpp_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr motion_planning_client_;

  rclcpp::Client<snp_msgs::srv::GenerateRobotProgram>::SharedPtr program_generation_client_;

  void update_status(bool success, std::string current_process, QPushButton* current_button, std::string next_process,
                     QPushButton* next_button, int step);

  std::string mesh_filepath_;
  tesseract_common::Toolpath tool_paths_;
  tesseract_planning::CompositeInstruction motion_plan_;

  //  std::unique_ptr<QTimer> timer_;

public slots:
  void update_calibration_requirement();
  void observe();
  void run_calibration();
  void get_correlation();
  void install_calibration();
  void reset_calibration();
  void scan();
  void plan_tool_paths();
  void plan_motion();
  void execute();
  void reset();
};

#endif  // ROSCONWINDOW_H
