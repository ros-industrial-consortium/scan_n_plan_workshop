#include <snp_application/snp_widget.h>
#include <QApplication>

#include <rclcpp/rclcpp.hpp>

void handleSignal(int)
{
  QApplication::instance()->quit();
}

int main(int argc, char* argv[])
{
  try
  {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    // Handle signals to terminate the Qt Application
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);

    auto node = std::make_shared<rclcpp::Node>("snp_application");
    auto blackboard = std::make_shared<snp_application::SnpBlackboard>(node);
    snp_application::BehaviorTreeFactoryGenerator bt_factory_gen =
        std::bind(snp_application::generateBehaviorTreeFactory, node);
    snp_application::SNPWidget w(node, blackboard, bt_factory_gen);
    w.show();

    // Run the Qt application, which is also sychronous
    auto ret = app.exec();

    // Shut down ROS to terminate call to `spin`
    rclcpp::shutdown();

    return ret;
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }
}
