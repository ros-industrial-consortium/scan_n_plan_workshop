#include "rosconwindow.h"
#include <QApplication>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  QApplication a(argc, argv);
  ROSConWindow w;
  w.show();

  std::thread t{ [&w]() {
    rclcpp::spin(w.getNode());
    rclcpp::shutdown();
  } };

  auto ret = a.exec();
  t.join();

  return ret;
}
