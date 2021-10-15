#include "rosconwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication a(argc, argv);
    ROSConWindow w;
    w.show();

    return a.exec();
}
