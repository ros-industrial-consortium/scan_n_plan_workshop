#include "rosconwindow.h"
#include "ui_rosconwindow.h"

#include <QMessageBox>
#include <sstream>
#include "tesseract_visualization/trajectory_player.h"

ROSConWindow::ROSConWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::ROSConWindow),
    past_calibration_(false)
{
    ui_->setupUi(this);

    connect(ui_->calibration_needed_checkbox, SIGNAL(clicked()), this, SLOT(update_calibration_requirement()));
    connect(ui_->observe_button, SIGNAL(clicked()), this, SLOT(observe()));
    connect(ui_->run_calibration_button, SIGNAL(clicked()), this, SLOT(run_calibration()));
    connect(ui_->install_calibration_button, SIGNAL(clicked()), this, SLOT(install_calibration()));
    connect(ui_->reset_calibration_button, SIGNAL(clicked()), this, SLOT(reset_calibration()));
    connect(ui_->scan_button, SIGNAL(clicked()), this, SLOT(scan()));
    connect(ui_->tpp_button, SIGNAL(clicked()), this, SLOT(plan_tool_paths()));
    connect(ui_->motion_plan_button, SIGNAL(clicked()), this, SLOT(plan_motion()));
    connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(execute()));

    node_ = rclcpp::Node::make_shared("roscon_app_node");

    node_->declare_parameter<bool>("sim_robot");
    node_->get_parameter("sim_robot", sim_robot_);

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_state_update", 10);

    // TODO register all service/action clients
    observe_client_ = node_->create_client<std_srvs::srv::Trigger>("observe");
    run_calibration_client_ = node_->create_client<std_srvs::srv::Trigger>("run");
    get_correlation_client_ = node_->create_client<std_srvs::srv::Trigger>("correlation");
    install_calibration_client_ = node_->create_client<std_srvs::srv::Trigger>("install");

    start_reconstruction_client_ = node_->create_client<open3d_interface_msgs::srv::StartYakReconstruction>("start_reconstruction");
    stop_reconstruction_client_ = node_->create_client<open3d_interface_msgs::srv::StopYakReconstruction>("stop_reconstruction");

    tpp_client_ = node_->create_client<snp_msgs::srv::GenerateToolPaths>("generate_tool_paths");

    program_generation_client_ = node_->create_client<snp_msgs::srv::GenerateRobotProgram>("generate_robot_program");
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

        ui_->progress_bar->setValue(step/5.0 * 100);

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
    RCLCPP_INFO_STREAM(node_->get_logger(), ui_->calibration_needed_checkbox->isChecked());
    RCLCPP_INFO_STREAM(node_->get_logger(), past_calibration_);
    RCLCPP_INFO_STREAM(node_->get_logger(), !ui_->calibration_needed_checkbox->isChecked() && !past_calibration_);

    if (!ui_->calibration_needed_checkbox->isChecked() && !past_calibration_)
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
    start_request->translation_distance = 0.0; 
    start_request->rotational_distance = 0.0;
    start_request->live = false;

    // TODO other params (currently set to recommended default)
    start_request->tsdf_params.voxel_length = 0.01;
    start_request->tsdf_params.sdf_trunc = 0.04;

    start_request->rgbd_params.depth_scale = 1000.0;
    start_request->rgbd_params.depth_trunc = 3.0;
    start_request->rgbd_params.convert_rgb_to_intensity = true;

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

    if (sim_robot_)
    {
        std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        
        std::vector<std::vector<double> > trajectory_positions = { { 0.2, 0.0,  0.0, 0.0, 0.0, 0.0},
                                                                   {-0.2, 0.0,  0.0, 0.0, 0.0, 0.0},
                                                                   {-0.2, 0.1, -0.1, 0.0, 0.0, 0.0},
                                                                   { 0.2, 0.1, -0.1, 0.0, 0.0, 0.0},
                                                                   { 0.2, 0.1, -0.1, 0.0, 0.0, 0.0},
                                                                   {-0.2, 0.2, -0.2, 0.0, 0.0, 0.0} };

        tesseract_common::JointTrajectory scan_trajectory;
        for (std::size_t i = 0; i < trajectory_positions.size(); i++)
        {
            tesseract_common::JointState joint_state;
            joint_state.joint_names = joint_names;
            joint_state.position = Eigen::Matrix<double, 6, 1>(trajectory_positions[i].data());
            joint_state.time = i * 0.1;

            scan_trajectory.push_back(joint_state);
        }
        tesseract_visualization::TrajectoryPlayer trajectory_player;
        trajectory_player.setTrajectory(scan_trajectory);

        rclcpp::Rate rate(30);
        while (!trajectory_player.isFinished())
        {
            tesseract_common::JointState current_state = trajectory_player.getNext();

            sensor_msgs::msg::JointState current_state_msg;

            current_state_msg.name = joint_names;
            current_state_msg.position = std::vector<double>(current_state.position.data(), 
                                                             current_state.position.data() + current_state.position.size());

            joint_state_pub_->publish(current_state_msg);

            rate.sleep();
        }
    }
    else
    {
        QMessageBox confirmation_box;
        confirmation_box.setWindowTitle("Scan Confirmation");
        confirmation_box.setText("The robot is currently scanning.");
        confirmation_box.setInformativeText("Click ok when the robot has completed the scan path.");
        confirmation_box.exec();

    }

   // call reconstruction stop
   open3d_interface_msgs::srv::StopYakReconstruction::Request::SharedPtr stop_request =
       std::make_shared<open3d_interface_msgs::srv::StopYakReconstruction::Request>();

   stop_request->archive_directory = "";
   stop_request->results_directory = "scan";

   auto stop_result = stop_reconstruction_client_->async_send_request(stop_request);
   if (rclcpp::spin_until_future_complete(node_, stop_result) == rclcpp::FutureReturnCode::SUCCESS)
   {
       auto stop_response = stop_result.get();
       success = stop_response->success;
       mesh_filepath_ = stop_response->mesh_filepath;
   }
   else
   {
       success = false;
   }


    update_status(success, "Scanning and reconstruction", ui_->scan_button, "plan tool paths", ui_->tpp_button, 2);
    
}

void ROSConWindow::plan_tool_paths()
{
    bool success;

     // do tpp things
     snp_msgs::srv::GenerateToolPaths::Request::SharedPtr request =
       std::make_shared<snp_msgs::srv::GenerateToolPaths::Request>();

     request->mesh_filename = mesh_filepath_;

     // TODO what parameters? will they be configurable?
     request->line_spacing = 0.05;
     request->min_hole_size = 0.02;
     request->min_segment_length = 0.01;
     request->point_spacing = 0.02;
     request->search_radius = 0.02;

     // TODO wait on a timer instead of indefinitely?
    
     auto result = tpp_client_->async_send_request(request);
     if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
     {
         auto response = result.get();
         success = response->success;
         tool_paths_ = std::shared_ptr<snp_msgs::msg::ToolPaths>(&result.get()->tool_paths);
     }
     else
     {
         RCLCPP_ERROR(node_->get_logger(), "TPP call failed");
         success = false;
     }

    update_status(success, "Tool path planning", ui_->tpp_button, "plan motion", ui_->motion_plan_button, 3);
}

void ROSConWindow::plan_motion()
{
    bool success = true; // TODO make this change based on result

    // do motion planning things

    // TODO Tesseract things...

    update_status(success, "Motion planning", ui_->motion_plan_button, "execute", ui_->execute_button, 4);
}

void ROSConWindow::execute()
{
    bool success;

    // do execution things
    snp_msgs::srv::GenerateRobotProgram::Request::SharedPtr request = 
        std::make_shared<snp_msgs::srv::GenerateRobotProgram::Request>();

    request->instructions = {"TODO"};

    // TODO get this working
    

    if (sim_robot_)
    {
        std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        
        // TODO be variable
        std::vector<std::vector<double> > trajectory_positions = { { 0.2, 0.0,  0.0, 0.0, 0.0, 0.0},
                                                                   {-0.2, 0.0,  0.0, 0.0, 0.0, 0.0},
                                                                   {-0.2, 0.1, -0.1, 0.0, 0.0, 0.0},
                                                                   { 0.2, 0.1, -0.1, 0.0, 0.0, 0.0},
                                                                   { 0.2, 0.1, -0.1, 0.0, 0.0, 0.0},
                                                                   {-0.2, 0.2, -0.2, 0.0, 0.0, 0.0} };

        tesseract_common::JointTrajectory scan_trajectory;
        for (std::size_t i = 0; i < trajectory_positions.size(); i++)
        {
            tesseract_common::JointState joint_state;
            joint_state.joint_names = joint_names;
            joint_state.position = Eigen::Matrix<double, 6, 1>(trajectory_positions[i].data());
            joint_state.time = i * 1;

            scan_trajectory.push_back(joint_state);
        }
        tesseract_visualization::TrajectoryPlayer trajectory_player;
        trajectory_player.setTrajectory(scan_trajectory);

        rclcpp::Rate rate(30);
        while (!trajectory_player.isFinished())
        {
            tesseract_common::JointState current_state = trajectory_player.getNext();

            sensor_msgs::msg::JointState current_state_msg;

            current_state_msg.name = joint_names;
            current_state_msg.position = std::vector<double>(current_state.position.data(), 
                                                             current_state.position.data() + current_state.position.size());

            joint_state_pub_->publish(current_state_msg);

            rate.sleep();
        }

        success = true; // TODO fake data?
    }
    else 
    {
        // auto result  = program_generation_client_->async_send_request(request);
        // if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     success = result.get()->success;

        //     // program is run via teach pendant
        // }
        // else
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Program generation call failed");
        //     success = false;
        // }

        success = true; // TODO get rid of this
    }

    update_status(success, "Execution", ui_->execute_button, "", nullptr, 5);
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
    ui_->execute_button->setEnabled(false);

    ui_->progress_bar->setValue(0);
}
