#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/plugins.hpp>
#include <gtest/gtest.h>
#include <QApplication>
#include <QPushButton>
#include <QProgressBar>
#include <QStackedWidget>
#include <rclcpp/rclcpp.hpp>

class BehaviorTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("snp_application_test");

    // Register non-ROS plugins
    {
      std::vector<std::string> bt_plugins;
      bt_plugins.push_back(BT_PLUGIN_LIB);
      for (const std::string& plugin : bt_plugins)
        factory.registerFromPlugin(std::filesystem::path(plugin));
    }

    // Register ROS plugins
    {
      BT::RosNodeParams ros_params;
      ros_params.nh = node;
      ros_params.wait_for_server_timeout = std::chrono::seconds(0);
      ros_params.server_timeout = std::chrono::seconds(1);
      ros_params.default_port_value = "topic";

      std::vector<std::string> bt_ros_plugins;
      bt_ros_plugins.push_back(BT_PLUGIN_LIB);
      for (const std::string& plugin : bt_ros_plugins)
        RegisterRosNode(factory, std::filesystem::path(plugin), ros_params);
    }

    // Register BT file
    {
      const std::string package_dir = ament_index_cpp::get_package_share_directory("snp_application");
      factory.registerBehaviorTreeFromFile(std::filesystem::path(package_dir) / "config" / "snp.xml");
    }

    // Create the blackboard
    {
      board = BT::Blackboard::create();

      // Set the error message key in the blackboard
      board->set("error_message", "");

      // Populate the blackboard with buttons
      stacked_widget = new QStackedWidget();
      progress_bar = new QProgressBar();
      button = new QPushButton();

      board->set("stacked_widget", stacked_widget);
      board->set("progress_bar", progress_bar);
      board->set("reset", button);
      board->set("halt", button);

      board->set("back", button);
      board->set("scan", button);
      board->set("tpp", button);
      board->set("plan", button);
      board->set("execute", button);
      board->set("tpp_config", button);
      board->set("skip_scan", false);
    }
  }

  void TearDown() override
  {
    delete stacked_widget;
    delete progress_bar;
    delete button;
  }

  rclcpp::Node::SharedPtr node;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;

  // Dummy Qt Widgets for use by the behavior trees
  QStackedWidget* stacked_widget;
  QProgressBar* progress_bar;
  QAbstractButton* button;
};

TEST_F(BehaviorTreeTest, LoadBehaviorTrees)
{
  for (const std::string& tree_name : factory.registeredBehaviorTrees())
  {
    std::cout << "Running BT: " << tree_name << std::endl;

    // Create the tree
    BT::Tree tree;
    ASSERT_NO_THROW(tree = factory.createTree(tree_name, board));

    // Add a logger for test debugging
    BT::StdCoutLogger logger(tree);

    // Tick the tree once
    EXPECT_NO_THROW(tree.tickOnce());
  }
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
