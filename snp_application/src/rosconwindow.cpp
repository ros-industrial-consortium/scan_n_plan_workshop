#include "rosconwindow.h"
#include "ui_rosconwindow.h"

#include <QMessageBox>
#include <QScrollBar>
#include <QTextStream>
#include <rclcpp_action/create_client.hpp>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_visualization/trajectory_player.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/utils/utils.h>
#include <tf2_eigen/tf2_eigen.h>

static const std::string JOINT_STATES_TOPIC = "robot_joint_states";
static const std::string TOOL_PATH_TOPIC = "toolpath";
static const std::string MESH_TOPIC = "scan_mesh";

static const std::string CALIBRATION_OBSERVE_SERVICE = "observe";
static const std::string CALIBRATION_RUN_SERVICE = "run";
static const std::string CALIBRATION_CORRELATION_SERVICE = "correlation";
static const std::string CALIBRATION_INSTALL_SERVICE = "install";
static const std::string START_RECONSTRUCTION_SERVICE = "start_reconstruction";
static const std::string STOP_RECONSTRUCTION_SERVICE = "stop_reconstruction";
static const std::string GENERATE_TOOL_PATHS_SERVICE = "generate_tool_paths";
static const std::string MOTION_PLAN_SERVICE = "create_motion_plan";
static const std::string MOTION_EXECUTION_SERVICE = "execute_motion_plan";

static const QString CALIBRATION_ST = "calibrate";
static const QString SCAN_APPROACH_ST = "execute scan approach";
static const QString START_RECONSTRUCTION_ST = "start reconstruction";
static const QString SCAN_EXECUTION_ST = "execute scan";
static const QString STOP_RECONSTRUCTION_ST = "stop reconstruction";
static const QString SCAN_DEPARTURE_ST = "execute scan departure";
static const QString TPP_ST = "plan tool paths";
static const QString MOTION_PLANNING_ST = "perform motion planning";
static const QString MOTION_EXECUTION_ST = "execute process motion";

static const std::map<QString, unsigned> STATES = {
  { CALIBRATION_ST, 0 },
  { SCAN_APPROACH_ST, 1 },
  { START_RECONSTRUCTION_ST, 2 },
  { SCAN_EXECUTION_ST, 3 },
  { STOP_RECONSTRUCTION_ST, 4 },
  { SCAN_DEPARTURE_ST, 5 },
  { TPP_ST, 6 },
  { MOTION_PLANNING_ST, 7 },
  { MOTION_EXECUTION_ST, 8 },
};

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

ROSConWindow::ROSConWindow(QWidget* parent)
  : QMainWindow(parent)
  , ui_(new Ui::ROSConWindow)
  , node_(rclcpp::Node::make_shared("roscon_app_node"))
  , past_calibration_(false)
  , mesh_filepath_("/tmp/results_mesh.ply")
{
  ui_->setupUi(this);

  connect(ui_->calibration_group_box, &QGroupBox::clicked, this, &ROSConWindow::update_calibration_requirement);
  connect(ui_->observe_button, &QPushButton::clicked, this, &ROSConWindow::observe);
  connect(ui_->run_calibration_button, &QPushButton::clicked, this, &ROSConWindow::run_calibration);
  connect(ui_->get_correlation_button, &QPushButton::clicked, this, &ROSConWindow::get_correlation);
  connect(ui_->install_calibration_button, &QPushButton::clicked, this, &ROSConWindow::install_calibration);
  connect(ui_->reset_calibration_button, &QPushButton::clicked, this, &ROSConWindow::reset_calibration);
  connect(ui_->scan_button, &QPushButton::clicked, this, &ROSConWindow::scan);
  connect(ui_->tpp_button, &QPushButton::clicked, this, &ROSConWindow::plan_tool_paths);
  connect(ui_->motion_plan_button, &QPushButton::clicked, this, &ROSConWindow::plan_motion);
  connect(ui_->motion_execution_button, &QPushButton::clicked, this, &ROSConWindow::execute);
  connect(ui_->reset_button, &QPushButton::clicked, this, &ROSConWindow::reset);

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_log->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_log->verticalScrollBar()->setSliderPosition(ui_->text_edit_log->verticalScrollBar()->maximum());
  });

  connect(this, &ROSConWindow::updateStatus, this, &ROSConWindow::onUpdateStatus);

  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATES_TOPIC, 10);
  toolpath_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(TOOL_PATH_TOPIC, 10);
  scan_mesh_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(MESH_TOPIC, 10);

  // TODO register all service/action clients
  observe_client_ = node_->create_client<std_srvs::srv::Trigger>(CALIBRATION_OBSERVE_SERVICE);
  run_calibration_client_ = node_->create_client<std_srvs::srv::Trigger>(CALIBRATION_RUN_SERVICE);
  get_correlation_client_ = node_->create_client<std_srvs::srv::Trigger>(CALIBRATION_CORRELATION_SERVICE);
  install_calibration_client_ = node_->create_client<std_srvs::srv::Trigger>(CALIBRATION_INSTALL_SERVICE);

  start_reconstruction_client_ =
      node_->create_client<open3d_interface_msgs::srv::StartYakReconstruction>(START_RECONSTRUCTION_SERVICE);
  stop_reconstruction_client_ =
      node_->create_client<open3d_interface_msgs::srv::StopYakReconstruction>(STOP_RECONSTRUCTION_SERVICE);
  tpp_client_ = node_->create_client<snp_msgs::srv::GenerateToolPaths>(GENERATE_TOOL_PATHS_SERVICE);
  motion_planning_client_ = node_->create_client<std_srvs::srv::Trigger>(MOTION_PLAN_SERVICE);
  motion_execution_client_ = node_->create_client<snp_msgs::srv::ExecuteMotionPlan>(MOTION_EXECUTION_SERVICE);
}

void ROSConWindow::onUpdateStatus(bool success, QString current_process, QPushButton* current_button,
                                  QString next_process, QPushButton* next_button, unsigned step)
{
  QString status;
  QTextStream status_stream(&status);
  if (success)
  {
    status_stream << current_process << " completed!";
    if (next_process != "")
    {
      status_stream << "\nWaiting to " << next_process << "...";
    }

    const double progress = (static_cast<double>(step) / static_cast<double>(STATES.size())) * 100.0;
    ui_->progress_bar->setValue(static_cast<int>(progress));

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

  ui_->text_edit_log->append(status);
}

void ROSConWindow::update_calibration_requirement()
{
  if (!ui_->calibration_group_box->isChecked() && !past_calibration_)
  {
    emit updateStatus(true, CALIBRATION_ST, nullptr, SCAN_APPROACH_ST, ui_->scan_button, STATES.at(SCAN_APPROACH_ST));
  }
  else
  {
    reset();
  }
}

void ROSConWindow::observe()
{
  if (!observe_client_->service_is_ready())
  {
    ui_->text_edit_log->append("Observation service is not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = observe_client_->async_send_request(request);
  future.wait();

  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
  if (response->success)
  {
    ui_->run_calibration_button->setEnabled(true);
    ui_->text_edit_log->append("Gathered observation.");
  }
  else
  {
    ui_->text_edit_log->append("Failed to get observation.");
  }
}

void ROSConWindow::run_calibration()
{
  if (!run_calibration_client_->service_is_ready())
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = run_calibration_client_->async_send_request(request);
  future.wait();
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

  if (response->success)
  {
    ui_->install_calibration_button->setEnabled(true);
    ui_->text_edit_log->append("Calibration run.");
  }
  else
  {
    ui_->text_edit_log->append("Calibration attempt failed.");
  }
}

void ROSConWindow::get_correlation()
{
  if (!get_correlation_client_->service_is_ready())
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = get_correlation_client_->async_send_request(request);
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

  if (response->success)
  {
    ui_->text_edit_log->append("Correlation written to file.");
  }
  else
  {
    ui_->text_edit_log->append("Failed to write correlation to file.");
  }
}

void ROSConWindow::install_calibration()
{
  if (!install_calibration_client_->service_is_ready())
  {
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = install_calibration_client_->async_send_request(request);
  future.wait();
  std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

  past_calibration_ = response->success;
  emit updateStatus(response->success, CALIBRATION_ST, nullptr, SCAN_APPROACH_ST, ui_->scan_button,
                    STATES.at(SCAN_APPROACH_ST));
}

void ROSConWindow::reset_calibration()
{
  past_calibration_ = false;

  // Update the UI
  ui_->run_calibration_button->setEnabled(false);
  ui_->get_correlation_button->setEnabled(false);
  ui_->install_calibration_button->setEnabled(false);
  ui_->reset_calibration_button->setEnabled(false);
}

void ROSConWindow::scan()
{  // TODO: fill out trajectory later
  control_msgs::action::FollowJointTrajectory::Goal approachGoal;

  snp_msgs::srv::ExecuteMotionPlan::Request::SharedPtr request =
      std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->motion_plan = approachGoal.trajectory;
  request->use_tool = false;

  if (!motion_execution_client_->service_is_ready())
  {
    RCLCPP_INFO(node_->get_logger(),
                "service not ready yet");  // we aren't going to wait, instead we will just reset the buttons
    // motion_execution_client_->wait_for_service();
    emit updateStatus(false, SCAN_APPROACH_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    // ^ does the above reset the options?
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "send scan request");

  auto cb = std::bind(&ROSConWindow::onScanApproachDone, this, std::placeholders::_1);
  motion_execution_client_->async_send_request(request, cb);
}

void ROSConWindow::onScanApproachDone(FJTResult result)
{
  if (result.get()->success)
  {
    emit updateStatus(true, SCAN_APPROACH_ST, ui_->scan_button, START_RECONSTRUCTION_ST, ui_->scan_button,
                      STATES.at(START_RECONSTRUCTION_ST));
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute scan approach motion");
    emit updateStatus(false, SCAN_APPROACH_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully executed scan approach motion");
  // call reconstruction start
  auto start_request = std::make_shared<open3d_interface_msgs::srv::StartYakReconstruction::Request>();

  start_request->tracking_frame = "camera_color_optical_frame";
  start_request->relative_frame = "floor";

  // TODO parameters
  start_request->translation_distance = 0;
  start_request->rotational_distance = 0;
  start_request->live = true;

  // TODO other params (currently set to recommended default)
  start_request->tsdf_params.voxel_length = 0.01f;
  start_request->tsdf_params.sdf_trunc = 0.04f;

  start_request->rgbd_params.depth_scale = 1000.0;
  start_request->rgbd_params.depth_trunc = 3.0;
  start_request->rgbd_params.convert_rgb_to_intensity = false;

  auto cb = std::bind(&ROSConWindow::onScanStartDone, this, std::placeholders::_1);
  start_reconstruction_client_->async_send_request(start_request, cb);
}

void ROSConWindow::onScanStartDone(StartScanFuture result)
{
  if (!result.get()->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start surface reconstruction");
    emit updateStatus(false, START_RECONSTRUCTION_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }

  if (!motion_execution_client_->service_is_ready())
  {
    RCLCPP_INFO(node_->get_logger(),
                "service not ready yet");  // we aren't going to wait, instead we will just reset the buttons
    emit updateStatus(false, START_RECONSTRUCTION_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }

  emit updateStatus(true, START_RECONSTRUCTION_ST, ui_->scan_button, SCAN_EXECUTION_ST, ui_->scan_button,
                    STATES.at(SCAN_EXECUTION_ST));

  RCLCPP_INFO(node_->get_logger(), "Sending scan trajectory goal");

  // TODO replace
  control_msgs::action::FollowJointTrajectory::Goal trajGoal;

  snp_msgs::srv::ExecuteMotionPlan::Request::SharedPtr request =
      std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->motion_plan = trajGoal.trajectory;
  request->use_tool = false;

  auto cb = std::bind(&ROSConWindow::onScanDone, this, std::placeholders::_1);
  motion_execution_client_->async_send_request(request, cb);
}

void ROSConWindow::onScanDone(FJTResult result)
{
  // Error checking about action
  if (result.get()->success)
  {
    emit updateStatus(true, SCAN_EXECUTION_ST, ui_->scan_button, STOP_RECONSTRUCTION_ST, ui_->scan_button,
                      STATES.at(STOP_RECONSTRUCTION_ST));
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute scan motion");
    emit updateStatus(false, SCAN_EXECUTION_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Successfully executed scan motion");

  // call reconstruction stop
  auto stop_request = std::make_shared<open3d_interface_msgs::srv::StopYakReconstruction::Request>();
  stop_request->archive_directory = "";
  stop_request->mesh_filepath = mesh_filepath_;

  auto cb = std::bind(&ROSConWindow::onScanStopDone, this, std::placeholders::_1);
  stop_reconstruction_client_->async_send_request(stop_request, cb);
}

void ROSConWindow::onScanStopDone(StopScanFuture stop_result)
{
  if (!stop_result.get()->success)
  {
    RCLCPP_INFO(node_->get_logger(), "Failed to stop surface reconstruction");
    emit updateStatus(false, STOP_RECONSTRUCTION_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }

  // Publish the mesh
  {
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
  }

  if (!motion_execution_client_->service_is_ready())
  {
    RCLCPP_INFO(node_->get_logger(),
                "service not ready yet");  // we aren't going to wait, instead we will just reset the buttons

    emit updateStatus(false, START_RECONSTRUCTION_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }

  emit updateStatus(true, STOP_RECONSTRUCTION_ST, ui_->scan_button, SCAN_DEPARTURE_ST, ui_->scan_button,
                    STATES.at(SCAN_DEPARTURE_ST));

  RCLCPP_INFO(node_->get_logger(), "Sending scan departure motion goal");

  // TODO: fill out later
  control_msgs::action::FollowJointTrajectory::Goal departGoal;

  snp_msgs::srv::ExecuteMotionPlan::Request::SharedPtr request =
      std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->motion_plan = departGoal.trajectory;
  request->use_tool = false;

  auto cb = std::bind(&ROSConWindow::onScanDepartureDone, this, std::placeholders::_1);
  motion_execution_client_->async_send_request(request, cb);
}

void ROSConWindow::onScanDepartureDone(FJTResult result)
{
  if (!result.get()->success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute scan motion departure");
    emit updateStatus(false, SCAN_DEPARTURE_ST, ui_->scan_button, SCAN_APPROACH_ST, ui_->scan_button,
                      STATES.at(SCAN_APPROACH_ST));
    return;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Successfully completed scan and surface reconstruction");
    emit updateStatus(true, SCAN_DEPARTURE_ST, ui_->scan_button, TPP_ST, ui_->tpp_button, STATES.at(TPP_ST));
  }
}

void ROSConWindow::plan_tool_paths()
{
  bool success = true;
  tool_paths_ = tesseract_common::Toolpath();

  // do tpp things
  if (!tpp_client_->service_is_ready())
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not find TPP server");
    success = false;
  }
  else
  {
    // Fill out the service call
    auto request = std::make_shared<snp_msgs::srv::GenerateToolPaths::Request>();
    request->mesh_filename = mesh_filepath_;
    request->line_spacing = 0.1;
    request->min_hole_size = 0.225;
    request->min_segment_length = 0.25;
    request->point_spacing = 0.05;
    request->search_radius = 0.0125;

    // Call the service
    auto future = tpp_client_->async_send_request(request);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    future.wait();
    QApplication::restoreOverrideCursor();

    snp_msgs::srv::GenerateToolPaths::Response::SharedPtr response = future.get();
    if (!response->success)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "TPP Error: '" << response->message << "'");
      success = false;
    }
    else
    {
      tool_paths_ = fromMsg(response->tool_paths);
      geometry_msgs::msg::PoseArray flat_toolpath_msg;
      flat_toolpath_msg.header.frame_id = "floor";
      for (const auto& toolpath : response->tool_paths.paths)
      {
        for (const auto& segment : toolpath.segments)
        {
          flat_toolpath_msg.poses.insert(flat_toolpath_msg.poses.end(), segment.poses.begin(), segment.poses.end());
        }
      }

      toolpath_pub_->publish(flat_toolpath_msg);
    }
  }

  emit updateStatus(success, TPP_ST, ui_->tpp_button, MOTION_PLANNING_ST, ui_->motion_plan_button,
                    STATES.at(MOTION_PLANNING_ST));
}

void ROSConWindow::plan_motion()
{
  bool success = true;

  motion_plan_ = trajectory_msgs::msg::JointTrajectory();

  // do motion planning things
  if (tool_paths_.empty())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "No tool paths exist");
    success = false;
  }
  else if (!motion_planning_client_->service_is_ready())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Motion planning server is not available");
    success = false;
  }
  else
  {
    // TODO: Fill a motion planning service request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call the service
    auto future = motion_planning_client_->async_send_request(request);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    future.wait();
    QApplication::restoreOverrideCursor();

    std_srvs::srv::Trigger::Response::SharedPtr response = future.get();
    if (response->success)
    {
      // motion_plan_ = result.get()->motion_plan
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Motion planning error: '" << response->message << "'");
      success = false;
    }
  }

  emit updateStatus(success, MOTION_PLANNING_ST, ui_->motion_plan_button, MOTION_EXECUTION_ST,
                    ui_->motion_execution_button, STATES.at(MOTION_EXECUTION_ST));
}

void ROSConWindow::execute()
{
  if (!motion_execution_client_->service_is_ready())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Motion execution service is not available");
    updateStatus(false, MOTION_EXECUTION_ST, ui_->motion_execution_button, MOTION_EXECUTION_ST,
                 ui_->motion_execution_button, STATES.at(MOTION_EXECUTION_ST));
    return;
  }

  // do execution things
  auto request = std::make_shared<snp_msgs::srv::ExecuteMotionPlan::Request>();
  request->motion_plan = motion_plan_;
  request->use_tool = true;

  auto future = motion_execution_client_->async_send_request(request);
  QApplication::setOverrideCursor(Qt::WaitCursor);
  future.wait();
  QApplication::restoreOverrideCursor();

  snp_msgs::srv::ExecuteMotionPlan::Response::SharedPtr response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Motion execution error: '" << response->message << "'");
    updateStatus(response->success, MOTION_EXECUTION_ST, ui_->motion_execution_button, MOTION_EXECUTION_ST,
                 ui_->motion_execution_button, STATES.at(MOTION_EXECUTION_ST));
    return;
  }
  else
  {
    emit updateStatus(response->success, MOTION_EXECUTION_ST, ui_->motion_execution_button, "", nullptr,
                      static_cast<unsigned>(STATES.size()));
  }
}

void ROSConWindow::reset()
{
  ui_->text_edit_log->setText("Waiting to calibrate...");

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
