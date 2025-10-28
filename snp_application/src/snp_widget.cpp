#include <snp_application/snp_widget.h>
#include "ui_snp_widget.h"
// BT
#include <snp_application/bt/bt_thread.h>
#include <snp_application/bt/text_edit_logger.h>
#include <snp_application/bt/utils.h>

#include <behaviortree_ros2/plugins.hpp>
#include <filesystem>
#include <noether_gui/widgets/configurable_tpp_pipeline_widget.h>
#include <QMessageBox>
#include <QTextStream>
#include <QScrollBar>
#include <QTextEdit>
#include <QToolBar>
#include <QAction>
#include <QIcon>
#include <QSettings>
#include <QStackedWidget>
#include <QCloseEvent>
#include <QShowEvent>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_preview/trajectory_preview_widget.h>

static const char* BT_FILES_PARAM = "bt_files";
static const char* BT_PLUGIN_LIBS_PARAM = "bt_plugin_libs";
static const char* BT_ROS_PLUGIN_LIBS_PARAM = "bt_ros_plugin_libs";
static const char* BT_PARAM = "tree";
static const char* BT_FREESPACE_PARAM = "freespace_tree";
static const char* BT_TIMEOUT_PARAM = "bt_timeout";
static const char* FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory_action";
static const char* HOME_STATE_NAME = "home_state";

class TPPDialog : public QDialog
{
public:
  TPPDialog(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr)
    : QDialog(parent), factory_(std::make_shared<noether::WidgetFactory>()), node_(node)
  {
    setWindowTitle("Tool Path Planner");

    // Set non-modal, so it can launch the load and save dialogs within itself
    setModal(false);

    auto tpp_config_file = snp_application::get_parameter<std::string>(node_, snp_application::TPP_CONFIG_FILE_PARAM);
    widget_ = new noether::ConfigurableTPPPipelineWidget(
        factory_, std::filesystem::path(tpp_config_file).parent_path().string(), this);
    widget_->configure(QString::fromStdString(tpp_config_file));

    // Add tool bar with save and load actions
    auto tool_bar = new QToolBar(this);
    auto action_load = tool_bar->addAction(QIcon::fromTheme("document-open"), "Load");
    auto action_save = tool_bar->addAction(QIcon::fromTheme("document-save"), "Save");
    connect(action_load, &QAction::triggered, widget_, &noether::ConfigurableTPPPipelineWidget::onLoadConfiguration);
    connect(action_save, &QAction::triggered, widget_, &noether::ConfigurableTPPPipelineWidget::onSaveConfiguration);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(tool_bar);
    layout->addWidget(widget_);
  }

  void showEvent(QShowEvent* event) override
  {
    QSettings settings;
    QString last_file = settings.value(noether::ConfigurableTPPPipelineWidget::SETTINGS_KEY_LAST_FILE).toString();

    // Configure the widget from file specified in the parameter
    auto tpp_config_file = snp_application::get_parameter<std::string>(node_, snp_application::TPP_CONFIG_FILE_PARAM);
    widget_->configure(QString::fromStdString(tpp_config_file));

    event->accept();
  }

  void closeEvent(QCloseEvent* event) override
  {
    switch (QMessageBox::question(this, "Exit", "Save before exiting?"))
    {
      case QMessageBox::StandardButton::Yes:
        widget_->onSaveConfiguration(true);
        break;
      default:
        break;
    }

    QSettings settings;
    QString last_file = settings.value(noether::ConfigurableTPPPipelineWidget::SETTINGS_KEY_LAST_FILE).toString();
    node_->set_parameter(rclcpp::Parameter(snp_application::TPP_CONFIG_FILE_PARAM, last_file.toStdString()));

    event->accept();
  }

protected:
  std::shared_ptr<noether::WidgetFactory> factory_;
  noether::ConfigurableTPPPipelineWidget* widget_;
  rclcpp::Node::SharedPtr node_;
};

namespace snp_application
{
SNPWidget::SNPWidget(rclcpp::Node::SharedPtr rviz_node, QWidget* parent)
  : QWidget(parent)
  , bt_node_(std::make_shared<rclcpp::Node>("snp_application_bt"))
  , ui_(new Ui::SNPWidget())
  , board_(BT::Blackboard::create())
{
  // Declare parameters
  bt_node_->declare_parameter<std::vector<std::string>>(BT_FILES_PARAM, std::vector<std::string>{});
  bt_node_->declare_parameter<std::vector<std::string>>(BT_PLUGIN_LIBS_PARAM, std::vector<std::string>{});
  bt_node_->declare_parameter<std::vector<std::string>>(BT_ROS_PLUGIN_LIBS_PARAM, std::vector<std::string>{});
  bt_node_->declare_parameter<std::string>(BT_PARAM, "");
  bt_node_->declare_parameter<int>(BT_TIMEOUT_PARAM, 6000);  // seconds
  bt_node_->declare_parameter<std::string>(FOLLOW_JOINT_TRAJECTORY_ACTION, "follow_joint_trajectory");
  bt_node_->declare_parameter<std::string>(MESH_FILE_PARAM, "");
  bt_node_->declare_parameter<std::string>(REF_FRAME_PARAM, "");
  bt_node_->declare_parameter<std::string>(TPP_CONFIG_FILE_PARAM, "");
  bt_node_->declare_parameter<std::string>(SCAN_TRAJ_FILE_PARAM, "");
  bt_node_->declare_parameter<std::string>(SCAN_MESH_FILE_PARAM, "");
  bt_node_->declare_parameter<std::string>(SCAN_REF_FRAME_PARAM, "");
  bt_node_->declare_parameter<std::string>(SCAN_CONFIG_FILE_PARAM, "");
  bt_node_->declare_parameter<std::string>(SCAN_TCP_FRAME_PARAM, "");
  bt_node_->declare_parameter<std::string>(TCP_FRAME_PARAM, "");
  // Home state
  bt_node_->declare_parameter<std::string>(BT_FREESPACE_PARAM, "");
  bt_node_->declare_parameter<std::vector<double>>(HOME_STATE_JOINT_VALUES_PARAM, std::vector<double>{});
  bt_node_->declare_parameter<std::vector<std::string>>(HOME_STATE_JOINT_NAMES_PARAM, std::vector<std::string>{});

  ui_->setupUi(this);
  ui_->group_box_operation->setEnabled(false);
  ui_->push_button_reset->setEnabled(false);
  ui_->push_button_home->setEnabled(true);

  // Add the TPP widget
  {
    auto* tpp_dialog = new TPPDialog(bt_node_, this);
    tpp_dialog->hide();
    connect(ui_->tool_button_tpp, &QToolButton::clicked, tpp_dialog, &QWidget::show);
  }

  // Add the trajectory preview widget
  {
    auto* preview = new trajectory_preview::TrajectoryPreviewWidget(this);
    preview->initializeROS(rviz_node, "motion_plan", "preview");

    auto* layout = new QVBoxLayout(ui_->frame_preview_widget);
    layout->addWidget(preview);
  }

  // Reset
  connect(ui_->push_button_reset, &QPushButton::clicked, [this]() {
    ui_->push_button_reset->setEnabled(false);
    ui_->push_button_start->setEnabled(true);
    ui_->push_button_home->setEnabled(true);
  });

  // Start
  connect(ui_->push_button_start, &QPushButton::clicked, [this]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->push_button_home->setEnabled(false);
    ui_->text_edit_log->clear();
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);

    try
    {
      runTreeWithThread(snp_application::get_parameter<std::string>(bt_node_, BT_PARAM));
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  });

  // Go Home
  connect(ui_->push_button_home, &QPushButton::clicked, [this]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->push_button_home->setEnabled(false);
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);

    try
    {
      runTreeWithThread(snp_application::get_parameter<std::string>(bt_node_, BT_FREESPACE_PARAM));
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  });

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_log->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_log->verticalScrollBar()->setSliderPosition(ui_->text_edit_log->verticalScrollBar()->maximum());
  });

  // Set the error message key in the blackboard
  board_->set(ERROR_MESSAGE_KEY, "");

  // Populate the blackboard with buttons
  board_->set("stacked_widget", ui_->stacked_widget);
  board_->set("progress_bar", ui_->progress_bar);
  board_->set("reset", static_cast<QAbstractButton*>(ui_->push_button_reset));
  board_->set("halt", static_cast<QAbstractButton*>(ui_->push_button_halt));

  board_->set("back", static_cast<QAbstractButton*>(ui_->push_button_back));
  board_->set("scan", static_cast<QAbstractButton*>(ui_->push_button_scan));
  board_->set("tpp", static_cast<QAbstractButton*>(ui_->push_button_tpp));
  board_->set("plan", static_cast<QAbstractButton*>(ui_->push_button_motion_plan));
  board_->set("execute", static_cast<QAbstractButton*>(ui_->push_button_motion_execution));
  board_->set("tpp_config", static_cast<QAbstractButton*>(ui_->tool_button_tpp));
  board_->set("skip_scan", false);
}

std::unique_ptr<BT::BehaviorTreeFactory> SNPWidget::createBTFactory(int ros_timeout)
{
  auto bt_factory = std::make_unique<BT::BehaviorTreeFactory>();

  // Register non-ROS plugins
  {
    auto bt_plugins = get_parameter<std::vector<std::string>>(bt_node_, BT_PLUGIN_LIBS_PARAM);
    for (const std::string& plugin : bt_plugins)
      bt_factory->registerFromPlugin(std::filesystem::path(plugin));
  }

  // Register ROS plugins
  {
    BT::RosNodeParams ros_params;
    ros_params.nh = bt_node_;
    ros_params.wait_for_server_timeout = std::chrono::seconds(0);
    ros_params.server_timeout = std::chrono::seconds(ros_timeout);

    auto bt_ros_plugins = get_parameter<std::vector<std::string>>(bt_node_, BT_ROS_PLUGIN_LIBS_PARAM);
    for (const std::string& plugin : bt_ros_plugins)
      RegisterRosNode(*bt_factory, std::filesystem::path(plugin), ros_params);
  }

  // Get joint trajectory action topic name from parameter and store it in the blackboard
  board_->set(FOLLOW_JOINT_TRAJECTORY_ACTION,
              snp_application::get_parameter<std::string>(bt_node_, FOLLOW_JOINT_TRAJECTORY_ACTION));

  sensor_msgs::msg::JointState home_state;
  home_state.name = snp_application::get_parameter<std::vector<std::string>>(bt_node_, HOME_STATE_JOINT_NAMES_PARAM);
  home_state.position = snp_application::get_parameter<std::vector<double>>(bt_node_, HOME_STATE_JOINT_VALUES_PARAM);
  board_->set(HOME_STATE_NAME, home_state);

  // Set scan toolpath generation parameters
  board_->set(SCAN_TRAJ_FILE_PARAM, snp_application::get_parameter<std::string>(bt_node_, SCAN_TRAJ_FILE_PARAM));
  board_->set(SCAN_CONFIG_FILE_PARAM, snp_application::get_parameter<std::string>(bt_node_, SCAN_CONFIG_FILE_PARAM));
  board_->set(SCAN_MESH_FILE_PARAM, snp_application::get_parameter<std::string>(bt_node_, SCAN_MESH_FILE_PARAM));
  board_->set(SCAN_REF_FRAME_PARAM, snp_application::get_parameter<std::string>(bt_node_, SCAN_REF_FRAME_PARAM));
  board_->set(SCAN_TCP_FRAME_PARAM, snp_application::get_parameter<std::string>(bt_node_, SCAN_TCP_FRAME_PARAM));

  board_->set(TCP_FRAME_PARAM, snp_application::get_parameter<std::string>(bt_node_, TCP_FRAME_PARAM));
  board_->set(TPP_CONFIG_FILE_PARAM, snp_application::get_parameter<std::string>(bt_node_, TPP_CONFIG_FILE_PARAM));
  board_->set(MESH_FILE_PARAM, snp_application::get_parameter<std::string>(bt_node_, MESH_FILE_PARAM));
  board_->set(REF_FRAME_PARAM, snp_application::get_parameter<std::string>(bt_node_, REF_FRAME_PARAM));

  return bt_factory;
}

void SNPWidget::runTreeWithThread(const std::string& bt_tree_name)
{
  try
  {
    auto* thread = new BTThread(this);

    // Create the BT factory
    std::unique_ptr<BT::BehaviorTreeFactory> bt_factory =
        createBTFactory(get_parameter<int>(bt_node_, BT_TIMEOUT_PARAM));

    auto bt_files = get_parameter<std::vector<std::string>>(bt_node_, BT_FILES_PARAM);
    if (bt_files.empty())
      throw std::runtime_error("Parameter '" + std::string(BT_FILES_PARAM) + "' is empty");

    for (const std::string& file : bt_files)
      bt_factory->registerBehaviorTreeFromFile(file);

    thread->tree = bt_factory->createTree(bt_tree_name, board_);
    logger_ = std::make_shared<TextEditLogger>(thread->tree.rootNode(), ui_->text_edit_log);

    connect(thread, &BTThread::finished, [thread, this]() {
      QString message;
      QTextStream stream(&message);
      switch (thread->result)
      {
        case BT::NodeStatus::SUCCESS:
          stream << "Behavior tree completed successfully";
          break;
        default:
          stream << "Behavior tree did not complete successfully";
          break;
      }

      if (!thread->message.isEmpty())
        stream << ": '" << thread->message << "'";

      QMetaObject::invokeMethod(ui_->text_edit_log, "append", Qt::QueuedConnection, Q_ARG(QString, message));
    });

    thread->start();
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, QString::fromStdString("Error"), QString::fromStdString(ex.what()));
    return;
  }
}

QStackedWidget* SNPWidget::getStackedWidget()
{
  return ui_->stacked_widget;
}

QTextEdit* SNPWidget::getTextEdit()
{
  return ui_->text_edit_log;
}

}  // namespace snp_application
