#ifndef ROSCONWINDOW_H
#define ROSCONWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>

namespace Ui {
class ROSConWindow;
}

class ROSConWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ROSConWindow(QWidget *parent = nullptr);
    ~ROSConWindow();

private:
    Ui::ROSConWindow *ui_;
    rclcpp::Node node_;

    void update_status(bool success, std::string current_process, QPushButton* current_button,
                       std::string next_process, QPushButton* next_button, int step);

public slots:
    void calibrate_camera();
    void scan();
    void reconstruct();
    void plan_tool_paths();
    void plan_motion();
    void execute();
    void reset();

};

#endif // ROSCONWINDOW_H
