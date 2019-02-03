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
    void on_slider_position_valueChanged(const int value);
    void on_slider_force_valueChanged(const int value);
    void on_slider_speed_valueChanged(const int value);

    void on_check_send_clicked(const bool checked);

private:
    void send();    // publish message
    void auto_send_check(); // check for auto send
    void set_mode(const uint8_t mode);
    void set_PRA(const uint8_t PRA);

private:
    Ui::Robotiq3FingerForm ui;

    typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput RQ3Fout;

    ros::NodeHandle n;
    ros::Publisher pub_command;

    RQ3Fout command;    // current state

    std::atomic_bool auto_send = ATOMIC_FLAG_INIT;

};

Robotiq3FingerPanel::Robotiq3FingerPanel(QWidget* parent) : rviz::Panel(parent) {
    // setup ROS
    pub_command = n.advertise<RQ3Fout>("Robotiq3FGripperRobotOutput", 1);

    // load ui form and auto-connect slots
    ui.setupUi(this);
}

void Robotiq3FingerPanel::on_button_send_clicked() {
    send();
}

void Robotiq3FingerPanel::on_button_on_clicked() {
    command = RQ3Fout();
    command.rACT = 1;
    command.rGTO = 1;
    command.rSPA = 255;
    command.rFRA = 150;
    auto_send_check();
}

void Robotiq3FingerPanel::on_button_off_clicked() {
    command = RQ3Fout();
    command.rACT = 0;
    auto_send_check();
}

void Robotiq3FingerPanel::on_button_basic_clicked() { set_mode(0); }

void Robotiq3FingerPanel::on_button_wide_clicked() { set_mode(2); }

void Robotiq3FingerPanel::on_button_pinch_clicked() { set_mode(1); }

void Robotiq3FingerPanel::on_button_scissor_clicked() { set_mode(3); }

void Robotiq3FingerPanel::on_button_open_clicked() { set_PRA(0); }

void Robotiq3FingerPanel::on_button_close_clicked() { set_PRA(255); }

void Robotiq3FingerPanel::on_slider_position_valueChanged(const int value) {
    set_PRA(uint8_t(value));
}

void Robotiq3FingerPanel::on_slider_force_valueChanged(const int value) {
    command.rFRA = uint8_t(value);
    auto_send_check();
}

void Robotiq3FingerPanel::on_slider_speed_valueChanged(const int value) {
    command.rSPA = uint8_t(value);
    auto_send_check();
}

void Robotiq3FingerPanel::on_check_send_clicked(const bool checked) {
    ui.button_send->setDown(checked);
    ui.button_send->setDisabled(checked);
    auto_send = checked;
}

void Robotiq3FingerPanel::send() { pub_command.publish(command); }

void Robotiq3FingerPanel::auto_send_check() { if(auto_send) { send(); } }

void Robotiq3FingerPanel::set_mode(const uint8_t mode) {
    command.rMOD = mode;
    auto_send_check();
}

void Robotiq3FingerPanel::set_PRA(const uint8_t PRA) {
    command.rPRA = PRA;
    auto_send_check();
}

} // robotiq_3f_rviz

#include "robotiq_3f_rviz.moc"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotiq_3f_rviz::Robotiq3FingerPanel,rviz::Panel )
