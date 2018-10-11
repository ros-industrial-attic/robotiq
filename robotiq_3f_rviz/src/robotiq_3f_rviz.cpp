# include <rviz/panel.h>
#include <ui_panel.h>   // generated from "panel.ui" using CMAKE_AUTOMOC

namespace robotiq_3f_rviz {

class Robotiq3FingerPanel : public rviz::Panel {
public:
    Robotiq3FingerPanel(QWidget* parent = 0);
};

Robotiq3FingerPanel::Robotiq3FingerPanel(QWidget* parent) : rviz::Panel(parent) {
    // load ui form
    Ui::Robotiq3FingerForm ui;
    ui.setupUi(this);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotiq_3f_rviz::Robotiq3FingerPanel,rviz::Panel )
