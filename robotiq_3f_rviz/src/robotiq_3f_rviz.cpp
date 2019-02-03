# include <rviz/panel.h>
#include <ui_panel.h>   // generated from "panel.ui" using CMAKE_AUTOMOC
#include <ros/ros.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>

namespace robotiq_3f_rviz {

class Robotiq3FingerPanel : public rviz::Panel {
    Q_OBJECT
public:
    Robotiq3FingerPanel(QWidget* parent = nullptr);

private slots:
    // auto-connected slots: on_<object_name>_<signal_name>(<signal parameters>)

    // buttons
    void on_button_send_clicked();

    void on_button_on_clicked();
    void on_button_off_clicked();

    void on_button_basic_clicked();
    void on_button_wide_clicked();
    void on_button_pinch_clicked();
    void on_button_scissor_clicked();

    void on_button_open_clicked();
    void on_button_close_clicked();

    // sliders
    void on_slider_position_valueChanged();
    void on_slider_force_valueChanged();
    void on_slider_speed_valueChanged();

    void on_check_send_clicked(bool checked);

private:
    typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput RQ3Fout;

    ros::NodeHandle n;
    ros::Publisher pub_out;

    RQ3Fout msg;    // current state

};

Robotiq3FingerPanel::Robotiq3FingerPanel(QWidget* parent) : rviz::Panel(parent) {
    // setup ROS
    pub_out = n.advertise<RQ3Fout>("Robotiq3FGripperRobotOutput", 1);

    // load ui form and auto-connect slots
    Ui::Robotiq3FingerForm ui;
    ui.setupUi(this);
}

void Robotiq3FingerPanel::on_button_send_clicked() {
    pub_out.publish(msg);
}

void Robotiq3FingerPanel::on_button_on_clicked() {}

void Robotiq3FingerPanel::on_button_off_clicked() {}

void Robotiq3FingerPanel::on_button_basic_clicked() {}

void Robotiq3FingerPanel::on_button_wide_clicked() {}

void Robotiq3FingerPanel::on_button_pinch_clicked() {}

void Robotiq3FingerPanel::on_button_scissor_clicked() {}

void Robotiq3FingerPanel::on_button_open_clicked() {}

void Robotiq3FingerPanel::on_button_close_clicked() {}

void Robotiq3FingerPanel::on_slider_position_valueChanged() {}

void Robotiq3FingerPanel::on_slider_force_valueChanged() {}

void Robotiq3FingerPanel::on_slider_speed_valueChanged() {}

void Robotiq3FingerPanel::on_check_send_clicked(bool checked) {}

} // robotiq_3f_rviz

#include "robotiq_3f_rviz.moc"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotiq_3f_rviz::Robotiq3FingerPanel,rviz::Panel )
