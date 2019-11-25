# include <rviz/panel.h>
#include <ui_panel.h>   // generated from "panel.ui" using CMAKE_AUTOMOC
#include <ros/ros.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>
#include <atomic>

namespace robotiq_3f_rviz {

class Robotiq3FingerPanel : public rviz::Panel {
    Q_OBJECT
public:
    Robotiq3FingerPanel(QWidget* parent = nullptr);

// slots
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

// typdefs
private:
    typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput RQ3Fout;
    typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput RQ3Fin;

// methods
private:
    void send();    // publish message
    void auto_send_check(); // check for auto send
    void set_mode(const uint8_t mode);
    void set_PRA(const uint8_t PRA);
    void on_status(const RQ3Fin &status);
    void set_button_active(QPushButton *button, const bool active);

// members
private:
    Ui::Robotiq3FingerForm ui;

    ros::NodeHandle n;
    ros::Publisher pub_command;
    ros::Subscriber sub_status;

    RQ3Fout command;    // current state

    std::atomic_bool auto_send = ATOMIC_FLAG_INIT;
};

Robotiq3FingerPanel::Robotiq3FingerPanel(QWidget* parent) : rviz::Panel(parent) {
    // setup ROS
    pub_command = n.advertise<RQ3Fout>("Robotiq3FGripperRobotOutput", 1);
    sub_status = n.subscribe("Robotiq3FGripperRobotInput", 1, &Robotiq3FingerPanel::on_status, this);

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
    send();
}

void Robotiq3FingerPanel::on_button_off_clicked() {
    command = RQ3Fout();
    command.rACT = 0;
    send();
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

    // force: 15 + value * 0.175 N (max: 60 N)
    static constexpr double min_force = 15;
    static constexpr double delta_force = 0.175;
    ui.force_value->setNum(int(min_force+value*delta_force));
}

void Robotiq3FingerPanel::on_slider_speed_valueChanged(const int value) {
    command.rSPA = uint8_t(value);
    auto_send_check();

    // speed: 22 + value * 0.34 mm/s (max: 110 mm/s)
    static constexpr double min_speed = 22;
    static constexpr double delta_speed = 0.34;
    ui.speed_value->setNum(int(min_speed+value*delta_speed));
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

void Robotiq3FingerPanel::on_status(const RQ3Fin &status) {
    set_button_active(ui.button_on, status.gACT==1);
    set_button_active(ui.button_off, status.gACT==0);

    set_button_active(ui.button_basic, status.gMOD==0);
    set_button_active(ui.button_pinch, status.gMOD==1);
    set_button_active(ui.button_wide, status.gMOD==2);
    set_button_active(ui.button_scissor, status.gMOD==3);

    // individual finger position
    ui.pos_a->setNum(status.gPOA);
    ui.pos_b->setNum(status.gPOB);
    ui.pos_c->setNum(status.gPOC);
    ui.pos_s->setNum(status.gPOS);

    // individual finger current
    // current: 0.1 * value mA
    ui.cur_a->setNum(0.1*status.gCUA);
    ui.cur_b->setNum(0.1*status.gCUB);
    ui.cur_c->setNum(0.1*status.gCUC);
    ui.cur_s->setNum(0.1*status.gCUS);
}

void Robotiq3FingerPanel::set_button_active(QPushButton *button, const bool active) {
    const QString style = active ? "QPushButton {color: red;}" : "QPushButton {}";
    button->setStyleSheet(style);
}

} // robotiq_3f_rviz

#include "robotiq_3f_rviz.moc"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotiq_3f_rviz::Robotiq3FingerPanel,rviz::Panel )
