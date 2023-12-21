#include "snp_widget.h"
#include "ui_snp_widget.h"
// BT
#include "bt/bt_thread.h"
#include "bt/button_approval_node.h"
#include "bt/button_monitor_node.h"
#include "bt/progress_decorator_node.h"
#include "bt/set_page_decorator_node.h"
#include "bt/snp_bt_service_node.h"
#include "bt/snp_sequence_with_memory_node.h"

#include <QMessageBox>
#include <QTextStream>

static const std::string BT_FILES_PARAM = "bt_files";
static const std::string BT_PARAM = "tree";

namespace snp_application
{
SNPWidget::SNPWidget(rclcpp::Node::SharedPtr node, QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::SNPWidget())
  , node_(node)
  , board_(BT::Blackboard::create())
{
  ui_->setupUi(this);

  // Reset
  connect(ui_->push_button_reset, &QPushButton::clicked, [this](){
    ui_->stacked_widget->setCurrentIndex(0);
    runTreeWithThread();
  });

  // Declare parameters
  node->declare_parameter<std::string>(MOTION_GROUP_PARAM, "");
  node->declare_parameter<std::string>(REF_FRAME_PARAM, "");
  node->declare_parameter<std::string>(TCP_FRAME_PARAM, "");
  node->declare_parameter<std::string>(CAMERA_FRAME_PARAM, "");
  node->declare_parameter<std::string>(MESH_FILE_PARAM, "");
  node->declare_parameter<std::vector<std::string>>(BT_FILES_PARAM, {});
  node->declare_parameter<std::string>(BT_PARAM, "");

  // Populate the blackboard with buttons
  board_->set(SetPageDecoratorNode::STACKED_WIDGET_KEY, ui_->stacked_widget);
  board_->set(ProgressDecoratorNode::PROGRESS_BAR_KEY, ui_->progress_bar);
  board_->set("reset", static_cast<QAbstractButton*>(ui_->push_button_reset));
  board_->set("back", static_cast<QAbstractButton*>(ui_->push_button_back));

  board_->set("scan", static_cast<QAbstractButton*>(ui_->push_button_scan));
  board_->set("tpp", static_cast<QAbstractButton*>(ui_->push_button_tpp));
  board_->set("plan", static_cast<QAbstractButton*>(ui_->push_button_motion_plan));
  board_->set("execute", static_cast<QAbstractButton*>(ui_->push_button_motion_execution));

  // Register custom nodes
  factory_.registerNodeType<ButtonMonitorNode>("ButtonMonitor");
  factory_.registerNodeType<ButtonApprovalNode>("ButtonApproval");
  factory_.registerNodeType<ProgressDecoratorNode>("Progress");
  factory_.registerNodeType<SetPageDecoratorNode>("SetPage");
  factory_.registerNodeType<SNPSequenceWithMemory>("SNPSequenceWithMemory");

  BT::RosNodeParams ros_params;
  ros_params.nh = node;
  ros_params.server_timeout = std::chrono::seconds(120);

  factory_.registerNodeType<TriggerServiceNode>("TriggerService", ros_params);
  factory_.registerNodeType<ExecuteMotionPlanServiceNode>("ExecuteMotionPlanService", ros_params);
  factory_.registerNodeType<GenerateMotionPlanServiceNode>("GenerateMotionPlanService", ros_params);
  factory_.registerNodeType<GenerateScanMotionPlanServiceNode>("GenerateScanMotionPlanService", ros_params);
  factory_.registerNodeType<GenerateToolPathsServiceNode>("GenerateToolPathsService", ros_params);
  factory_.registerNodeType<StartReconstructionServiceNode>("StartReconstructionService", ros_params);
  factory_.registerNodeType<StopReconstructionServiceNode>("StopReconstructionService", ros_params);

  auto bt_files = get_parameter<std::vector<std::string>>(node, BT_FILES_PARAM);
  for(const std::string& file : bt_files)
    factory_.registerBehaviorTreeFromFile(file);
}

void SNPWidget::runTreeWithThread()
{
  auto thread = new BTThread(this);

  try
  {
    thread->tree = factory_.createTree(get_parameter<std::string>(node_, BT_PARAM), board_);
  }
  catch(const std::exception& ex)
  {
    QMessageBox::warning(this, QString::fromStdString("Error"), QString::fromStdString(ex.what()));
    return;
  }

  connect(thread, &BTThread::finished, [thread](){
    QString message;
    QTextStream stream(&message);
    switch (thread->result)
    {
      case BT::NodeStatus::SUCCESS:
        stream << "BT completed successfully: '" << thread->message << "'";
        std::cout << message.toStdString() << std::endl;
        break;
      default:
        stream << "BT did not complete successfully: '" << thread->message << "'";
        std::cout << message.toStdString() << std::endl;
        break;
    }
    thread->deleteLater();
  });

  thread->start();
}

} // namespace snp_application

