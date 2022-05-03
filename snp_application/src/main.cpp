#include "rosconwindow.h"
#include <QApplication>

#include <rclcpp/rclcpp.hpp>

void handleSignal(int)
{
  QApplication::instance()->quit();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);

  // Handle signals to terminate the Qt Application
  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  ROSConWindow w;
  w.show();

  // Move the ROS spinning into a separate thread since the call to `spin` is synchronous
  std::thread t{ [&w]() { rclcpp::spin(w.getNode()); } };

  // Run the Qt application, which is also sychronous
  auto ret = app.exec();

  // Shut down ROS to terminate call to `spin`
  rclcpp::shutdown();

  // Wait for the thread to finish
  t.join();

  return ret;
}
