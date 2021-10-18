#include "rosconwindow.h"
#include "ui_rosconwindow.h"

#include <sstream>

ROSConWindow::ROSConWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::ROSConWindow), node_("roscon_app_node")
{
    ui_->setupUi(this);

    connect(ui_->calibrate_button, SIGNAL(clicked()), this, SLOT(calibrate_camera()));
    connect(ui_->scan_button, SIGNAL(clicked()), this, SLOT(scan()));
    connect(ui_->reconstruct_button, SIGNAL(clicked()), this, SLOT(reconstruct()));
    connect(ui_->tpp_button, SIGNAL(clicked()), this, SLOT(plan_tool_paths()));
    connect(ui_->motion_plan_button, SIGNAL(clicked()), this, SLOT(plan_motion()));
    connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(execute()));
    connect(ui_->reset_button, SIGNAL(clicked()), this, SLOT(reset()));

    // TODO register all service/action clients
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
        status_stream << current_process << " completed!\nWaiting to " << next_process << "...";

        ui_->progress_bar->setValue(step/6.0 * 100);
        current_button->setEnabled(false);
        next_button->setEnabled(true);
        ui_->reset_button->setEnabled(true);
    }
    else
    {
        status_stream << current_process << " failed\nWaiting to attempt again...";
    }

    ui_->status_label->setText(status_stream.str().c_str());
}

void ROSConWindow::calibrate_camera()
{
    bool success = true; // TODO make this change based on result

    // do calibration things

    update_status(success, "Calibration", ui_->calibrate_button, "scan", ui_->scan_button, 1);
}

void ROSConWindow::scan()
{
    bool success = true; // TODO make this change based on result

    // do scan things

    update_status(success, "Scan", ui_->scan_button, "reconstruct", ui_->reconstruct_button, 2);
}

void ROSConWindow::reconstruct()
{
    bool success = true; // TODO make this change based on result

    // do reconstruction things

    update_status(success, "Reconstruction", ui_->reconstruct_button, "plan tool paths", ui_->tpp_button, 3);
}

void ROSConWindow::plan_tool_paths()
{
    bool success = true; // TODO make this change based on result

    // do tpp things

    update_status(success, "Tool path planning", ui_->tpp_button, "plan motion", ui_->motion_plan_button, 4);
}

void ROSConWindow::plan_motion()
{
    bool success = true; // TODO make this change based on result

    // do motion planning things

    update_status(success, "Motion planning", ui_->motion_plan_button, "execute", ui_->execute_button, 5);
}

void ROSConWindow::execute()
{
    bool success = true; // TODO make this change based on result

    // do execution things

    update_status(success, "Execution", ui_->execute_button, "reset", ui_->reset_button, 6);
}

void ROSConWindow::reset()
{
    ui_->status_label->setText("Waiting to calibrate...");

    // reset button states
    ui_->calibrate_button->setEnabled(true);
    
    ui_->scan_button->setEnabled(false);
    ui_->reconstruct_button->setEnabled(false);
    ui_->tpp_button->setEnabled(false);
    ui_->motion_plan_button->setEnabled(false);
    ui_->execute_button->setEnabled(false);
    ui_->reset_button->setEnabled(false);

    ui_->progress_bar->setValue(0);
}