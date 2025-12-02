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

using namespace snp_application;

class TPPDialog : public QDialog
{
public:
  TPPDialog(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr)
    : QDialog(parent), factory_(std::make_shared<noether::WidgetFactory>()), node_(node)
  {
    setWindowTitle("Tool Path Planner");

    // Set non-modal, so it can launch the load and save dialogs within itself
    setModal(false);

    auto tpp_config_file = snp_application::get_parameter<std::string>(node_, PROCESS_TPP_CONFIG_FILE_PARAM);
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
    // Configure the widget from file specified in the parameter
    auto tpp_config_file = snp_application::get_parameter<std::string>(node_, PROCESS_TPP_CONFIG_FILE_PARAM);
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

    // Get the last saved TPP configuration file from the QSettings
    QSettings settings;
    const QString last_file = settings.value(noether::ConfigurableTPPPipelineWidget::SETTINGS_KEY_LAST_FILE).toString();

    if (last_file.isNull() || last_file.isEmpty())
      QMessageBox::warning(this, "QSettings Error",
                           "Failed to extract last saved TPP configuration file from QSettings."
                           "Please ensure the directory '$HOME/.config' is read/write accessible.");
    else
      node_->set_parameter(rclcpp::Parameter(PROCESS_TPP_CONFIG_FILE_PARAM, last_file.toStdString()));

    event->accept();
  }

protected:
  std::shared_ptr<noether::WidgetFactory> factory_;
  noether::ConfigurableTPPPipelineWidget* widget_;
  rclcpp::Node::SharedPtr node_;
};

namespace snp_application
{
SNPWidget::SNPWidget(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr blackboard,
                     BehaviorTreeFactoryGenerator bt_factory_generator, QWidget* parent)
  : QWidget(parent), blackboard_(blackboard), bt_factory_generator_(bt_factory_generator), ui_(new Ui::SNPWidget())
{
  using namespace snp_application;

  ui_->setupUi(this);
  ui_->group_box_operation->setEnabled(false);
  ui_->push_button_reset->setEnabled(false);
  ui_->push_button_home->setEnabled(true);

  // Add the TPP widget
  {
    auto* tpp_dialog = new TPPDialog(node, this);
    tpp_dialog->hide();
    connect(ui_->tool_button_tpp, &QToolButton::clicked, tpp_dialog, &QWidget::show);
  }

  // Add the trajectory preview widget
  {
    auto* preview = new trajectory_preview::TrajectoryPreviewWidget(this);
    preview->initializeROS(node, "motion_plan", "preview");

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
  connect(ui_->push_button_start, &QPushButton::clicked, [this, node]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->push_button_home->setEnabled(false);
    ui_->text_edit_log->clear();
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);

    try
    {
      runTreeWithThread(snp_application::get_parameter<std::string>(node, BT_PARAM));
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  });

  // Go Home
  connect(ui_->push_button_home, &QPushButton::clicked, [this, node]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->push_button_home->setEnabled(false);
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);

    try
    {
      runTreeWithThread(snp_application::get_parameter<std::string>(node, BT_FREESPACE_PARAM));
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

  // Populate the blackboard with buttons
  blackboard_->set("stacked_widget", ui_->stacked_widget);
  blackboard_->set("progress_bar", ui_->progress_bar);
  blackboard_->set("reset", static_cast<QAbstractButton*>(ui_->push_button_reset));
  blackboard_->set("halt", static_cast<QAbstractButton*>(ui_->push_button_halt));

  blackboard_->set("back", static_cast<QAbstractButton*>(ui_->push_button_back));
  blackboard_->set("scan", static_cast<QAbstractButton*>(ui_->push_button_scan));
  blackboard_->set("tpp", static_cast<QAbstractButton*>(ui_->push_button_tpp));
  blackboard_->set("plan", static_cast<QAbstractButton*>(ui_->push_button_motion_plan));
  blackboard_->set("execute", static_cast<QAbstractButton*>(ui_->push_button_motion_execution));
  blackboard_->set("tpp_config", static_cast<QAbstractButton*>(ui_->tool_button_tpp));
  blackboard_->set("skip_scan", false);
}

void SNPWidget::runTreeWithThread(const std::string& bt_tree_name)
{
  try
  {
    auto* thread = new BTThread(this);

    // Create the BT factory
    std::unique_ptr<BT::BehaviorTreeFactory> bt_factory = bt_factory_generator_();

    // Create the behavior tree
    // Use the blackboard from the BT node directly because it can receive updates when ROS2 parameters are changed
    thread->tree = bt_factory->createTree(bt_tree_name, blackboard_);

    // Create a logger to capture output from the behavior tree for display to the user through the text edit in the UI
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
