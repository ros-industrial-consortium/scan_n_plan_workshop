#include "rosconwindow.h"
#include "ui_rosconwindow.h"

#include <sstream>
#include <memory>
#include <functional>
#include <thread>
#include <QMessageBox>
#include <tf2_eigen/tf2_eigen.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <snp_msgs/msg/tool_paths.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_visualization/trajectory_player.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/utils/utils.h>

namespace  // anonymous restricts visibility to this file
{
tesseract_common::Toolpath fromMsg(const snp_msgs::msg::ToolPaths& msg)
{
  tesseract_common::Toolpath tps;
  tps.reserve(msg.paths.size());
  for (const auto& path : msg.paths)
  {
    for (const auto& segment : path.segments)
    {
      tesseract_common::VectorIsometry3d seg;
      seg.reserve(segment.poses.size());
      for (const auto& pose : segment.poses)
      {
        Eigen::Isometry3d p;
        tf2::fromMsg(pose, p);
        seg.push_back(p);
      }
      tps.push_back(seg);
    }
  }
  return tps;
}

}  // namespace

ROSConWindow::ROSConWindow(QWidget* parent) : QMainWindow(parent), ui_(new Ui::ROSConWindow), past_calibration_(false)
{
  ui_->setupUi(this);

  connect(ui_->calibration_group_box, SIGNAL(clicked()), this, SLOT(update_calibration_requirement()));
  connect(ui_->observe_button, SIGNAL(clicked()), this, SLOT(observe()));
  connect(ui_->run_calibration_button, SIGNAL(clicked()), this, SLOT(run_calibration()));
  connect(ui_->get_correlation_button, SIGNAL(clicked()), this, SLOT(get_correlation()));
  connect(ui_->install_calibration_button, SIGNAL(clicked()), this, SLOT(install_calibration()));
  connect(ui_->reset_calibration_button, SIGNAL(clicked()), this, SLOT(reset_calibration()));
  connect(ui_->scan_button, SIGNAL(clicked()), this, SLOT(scan()));
  connect(ui_->tpp_button, SIGNAL(clicked()), this, SLOT(plan_tool_paths()));
  connect(ui_->motion_plan_button, SIGNAL(clicked()), this, SLOT(plan_motion()));
  connect(ui_->motion_execution_button, SIGNAL(clicked()), this, SLOT(execute()));

  node_ = rclcpp::Node::make_shared("roscon_app_node");

  node_->declare_parameter("sim_robot");
  node_->get_parameter<bool>("sim_robot", sim_robot_);

  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("robot_joint_states", 10);
  toolpath_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("toolpath", 10);
  scan_mesh_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("scan_mesh", 10);

  // TODO register all service/action clients
  observe_client_ = node_->create_client<std_srvs::srv::Trigger>("observe");
  run_calibration_client_ = node_->create_client<std_srvs::srv::Trigger>("run");
  get_correlation_client_ = node_->create_client<std_srvs::srv::Trigger>("correlation");
  install_calibration_client_ = node_->create_client<std_srvs::srv::Trigger>("install");

  start_reconstruction_client_ = node_->create_client<open3d_interface_msgs::srv::StartYakReconstruction>("start_"
                                                                                                          "reconstructi"
                                                                                                          "on");
  stop_reconstruction_client_ = node_->create_client<open3d_interface_msgs::srv::StopYakReconstruction>("stop_"
                                                                                                        "reconstructio"
                                                                                                        "n");

  tpp_client_ = node_->create_client<snp_msgs::srv::GenerateToolPaths>("generate_tool_paths");

  motion_planning_client_ = node_->create_client<std_srvs::srv::Trigger>("/snp_planning_server/"
                                                                         "tesseract_trigger_motion_plan");

  motion_execution_client_ = node_->create_client<snp_msgs::srv::ExecuteMotionPlan>("execute_motion_plan");

  follow_joint_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, "follow_"
                                                                                                          "joint_"
                                                                                                          "trajectory");

  enable_client_ = node_->create_client<std_srvs::srv::Trigger>("robot_enable");
}

ROSConWindow::~ROSConWindow()
{
  delete ui_;
}

void ROSConWindow::update_status(bool success, std::string current_process, QPushButton* current_button,
                                 std::string next_process, QPushButton* next_button, int step)
{
  std::stringstream status_stream;
  if (success)
  {
    status_stream << current_process << " completed!";
    if (next_process != "")
    {
      status_stream << "\nWaiting to " << next_process << "...";
    }

    ui_->progress_bar->setValue(step / 5.0 * 100);

    if (next_button != nullptr)
    {
      next_button->setEnabled(true);
    }

    if (current_button != nullptr)
    {
      current_button->setEnabled(false);
    }
  }
  else
  {
    status_stream << current_process << " failed\nWaiting to attempt again...";
  }

  ui_->status_label->setText(status_stream.str().c_str());
}

void ROSConWindow::update_calibration_requirement()
{
  if (!ui_->calibration_group_box->isChecked() && !past_calibration_)
  {
    update_status(true, "Calibration", nullptr, "scan", ui_->scan_button, 1);
  }
  else
  {
    reset();
  }
}

void ROSConWindow::observe()
{
  bool success;
  std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = observe_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    success = response->success;
  }

  if (success)
  {
    ui_->run_calibration_button->setEnabled(true);
    ui_->status_label->setText("Gathered observation.");
  }
  else
  {
    ui_->status_label->setText("Failed to get observation.");
  }
}

void ROSConWindow::run_calibration()
{
  bool success;
  std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = run_calibration_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    success = response->success;
  }

  if (success)
  {
    ui_->install_calibration_button->setEnabled(true);
    ui_->status_label->setText("Calibration run.");
  }
  else
  {
    ui_->status_label->setText("Calibration attempt failed.");
  }
}

void ROSConWindow::get_correlation()
{
  bool success;
  std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = get_correlation_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    success = response->success;
  }

  if (success)
  {
    ui_->status_label->setText("Correlation written to file.");
  }
  else
  {
    ui_->status_label->setText("Failed to write correlation to file.");
  }
}

void ROSConWindow::install_calibration()
{
  bool success;
  std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = install_calibration_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    success = response->success;
  }

  past_calibration_ = success;
  update_status(success, "Calibration", nullptr, "scan", ui_->scan_button, 1);
}

void ROSConWindow::reset_calibration()
{
  bool success;
  std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = install_calibration_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    success = response->success;
  }
  else
  {
    success = false;
  }

  if (success)
  {
    ui_->run_calibration_button->setEnabled(false);
    ui_->get_correlation_button->setEnabled(false);
    ui_->install_calibration_button->setEnabled(false);
    ui_->reset_calibration_button->setEnabled(false);
  }
}

void ROSConWindow::scan()
{
  bool success;

  ui_->status_label->setText("Performing scan...");
  ui_->status_label->repaint();

  // call reconstruction start
  open3d_interface_msgs::srv::StartYakReconstruction::Request::SharedPtr start_request =
      std::make_shared<open3d_interface_msgs::srv::StartYakReconstruction::Request>();

  start_request->tracking_frame = "camera_color_optical_frame";
  start_request->relative_frame = "floor";

  // TODO parameters
  start_request->translation_distance = 0;
  start_request->rotational_distance = 0;
  start_request->live = true;

  // TODO other params (currently set to recommended default)
  start_request->tsdf_params.voxel_length = 0.01;
  start_request->tsdf_params.sdf_trunc = 0.04;

  start_request->rgbd_params.depth_scale = 1000.0;
  start_request->rgbd_params.depth_trunc = 3.0;
  start_request->rgbd_params.convert_rgb_to_intensity = false;

  auto start_result = start_reconstruction_client_->async_send_request(start_request);
  if (rclcpp::spin_until_future_complete(node_, start_result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto start_response = start_result.get();
    success = start_response->success;
  }
  else
  {
    success = false;
  }

  if (!success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Start reconstruction call failed");
    update_status(success, "Reconstruction", ui_->scan_button, "plan tool paths", ui_->tpp_button, 2);
  }
}

void ROSConWindow::stopScan()
{
  // call reconstruction stop
  open3d_interface_msgs::srv::StopYakReconstruction::Request::SharedPtr stop_request =
      std::make_shared<open3d_interface_msgs::srv::StopYakReconstruction::Request>();

  mesh_filepath_ = "/tmp/results_mesh.ply";
  stop_request->archive_directory = "";
  stop_request->mesh_filepath = mesh_filepath_;

  auto stop_result = stop_reconstruction_client_->async_send_request(stop_request);
  if (rclcpp::spin_until_future_complete(node_, stop_result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto stop_response = stop_result.get();
    bool success = stop_response->success;

    RCLCPP_INFO(node_->get_logger(), "Mesh saved to '%s'.", mesh_filepath_.c_str());

    visualization_msgs::msg::Marker mesh_marker;
    mesh_marker.header.frame_id = "floor";

    mesh_marker.color.r = 200;
    mesh_marker.color.g = 200;
    mesh_marker.color.b = 0;
    mesh_marker.color.a = 1;

    mesh_marker.scale.x = 1;
    mesh_marker.scale.y = 1;
    mesh_marker.scale.z = 1;

    mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_marker.mesh_resource = "file://" + mesh_filepath_;

    scan_mesh_pub_->publish(mesh_marker);

    success = true;
    update_status(success, "Scanning and reconstruction", ui_->scan_button, "plan tool paths", ui_->tpp_button, 2);
  }
}

void ROSConWindow::plan_tool_paths()
{
  bool success = true;
  tool_paths_ = tesseract_common::Toolpath();

  // do tpp things
  if (!tpp_client_->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not find TPP server");
    success = false;
  }
  else
  {
    // Fill out the service call
    std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Request> request =
        std::make_shared<snp_msgs::srv::GenerateToolPaths::Request>();
    request->mesh_filename = mesh_filepath_;
    request->line_spacing = 0.1;
    request->min_hole_size = 0.225;
    request->min_segment_length = 0.25;
    request->point_spacing = 0.05;
    request->search_radius = 0.0125;

    // Call the service
    auto result = tpp_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TPP call failed");
      success = false;
    }
    else
    {
      snp_msgs::msg::ToolPaths toolpaths_msg = result.get()->tool_paths;
      tool_paths_ = fromMsg(toolpaths_msg);
      geometry_msgs::msg::PoseArray flat_toolpath_msg;
      flat_toolpath_msg.header.frame_id = "floor";
      for (const auto& toolpath : toolpaths_msg.paths)
      {
        for (const auto& segment : toolpath.segments)
        {
          flat_toolpath_msg.poses.insert(flat_toolpath_msg.poses.end(), segment.poses.begin(), segment.poses.end());
        }
      }

      toolpath_pub_->publish(flat_toolpath_msg);
    }
  }

  update_status(success, "Tool path planning", ui_->tpp_button, "plan motion", ui_->motion_plan_button, 3);
}

void ROSConWindow::startScan()
{
  if (!this->follow_joint_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node_->get_logger(), "Sending approach goal");

  // TODO: fill out later
  control_msgs::action::FollowJointTrajectory::Goal approachGoal;

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.result_callback = std::bind(&ROSConWindow::onApproachDone, this, std::placeholders::_1);

  follow_joint_client_->async_send_goal(approachGoal, send_goal_options);
}

void ROSConWindow::onApproachDone(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Approach was aborted");
      rclcpp::shutdown();
      return;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Approach was canceled");
      rclcpp::shutdown();
      return;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }

  if (!this->follow_joint_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node_->get_logger(), "Sending Trajectory goal");

  // TODO replace
  control_msgs::action::FollowJointTrajectory::Goal trajGoal;

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.result_callback =
      std::bind(&ROSConWindow::onTrajDone, this, std::placeholders::_1);  // bind different callback to it
  this->follow_joint_client_->async_send_goal(trajGoal, send_goal_options);

  ROSConWindow::scan();  // calls the actual scanner?
}

void ROSConWindow::onTrajDone(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
{
  // Error checking about action
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Scan Trajectory was aborted");
      rclcpp::shutdown();
      return;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Scan Trajectory was canceled");
      rclcpp::shutdown();
      return;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }

  if (!this->follow_joint_client_->wait_for_action_server())
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  control_msgs::action::FollowJointTrajectory::Goal departGoal;

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.result_callback =
      std::bind(&ROSConWindow::onDepartDone, this, std::placeholders::_1);  // bind different callback to it
  this->follow_joint_client_->async_send_goal(departGoal, send_goal_options);

  ROSConWindow::stopScan();  // stops the actual scanner
}

// this is just winding down the action node once the departure is handled.
void ROSConWindow::onDepartDone(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Departure was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Departure was canceled");
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }
  RCLCPP_INFO(node_->get_logger(), "Departure Successful, shutting down node");
  rclcpp::shutdown();
}

void ROSConWindow::plan_motion()
{
  bool success = true;

  motion_plan_ = trajectory_msgs::msg::JointTrajectory();

  // do motion planning things
  if (tool_paths_.size() > 0)
  {
    // TODO: Fill a motion planning service request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call the service
    auto result = motion_planning_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success)
    {
      // motion_plan_ = result.get()->motion_plan
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motion Planning call failed");
      success = false;
    }

    //    std::shared_ptr<tesseract_msgs::srv::GetMotionPlan::Request> request =
    //        std::make_shared<tesseract_msgs::srv::GetMotionPlan::Request>();
    //    request->request.name = request->request.RASTER_G_FT_PLANNER_NAME;  // TODO: use correct planner
    //    request->request.instructions =
    //    tesseract_planning::Serialization::toArchiveStringXML<tesseract_planning::CompositeInstruction>(program);

    //    // Call the service
    //    auto result = motion_planning_client_->async_send_request(request);
    //    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    //    {
    //      motion_plan_ =
    //      tesseract_planning::Serialization::fromArchiveStringXML<tesseract_planning::CompositeInstruction>(result.get()->response.results);
    //    }
    //    else
    //    {
    //      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motion Planning call failed");
    //      success = false;
    //    }
  }
  else
  {
    success = false;
  }
  update_status(success, "Motion planning", ui_->motion_plan_button, "execute", ui_->motion_execution_button, 4);
}

void ROSConWindow::execute()
{
  bool success = true;

  // do execution things
  snp_msgs::srv::ExecuteMotionPlan::Request::SharedPtr request =
      std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->motion_plan = motion_plan_;
  request->use_tool = true;

  auto future = motion_execution_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    success = result->success;
    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Motion execution failed '%s'", result->message.c_str());
    }
    // program is run via teach pendant
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Call to motion execution failed");
    success = false;
  }

  update_status(success, "Execution", ui_->motion_execution_button, "", nullptr, 5);
}

void ROSConWindow::reset()
{
  ui_->status_label->setText("Waiting to calibrate...");

  // reset button states
  ui_->run_calibration_button->setEnabled(false);
  ui_->get_correlation_button->setEnabled(false);
  ui_->install_calibration_button->setEnabled(false);
  ui_->scan_button->setEnabled(false);
  ui_->tpp_button->setEnabled(false);
  ui_->motion_plan_button->setEnabled(false);
  ui_->motion_execution_button->setEnabled(false);

  ui_->progress_bar->setValue(0);
}
