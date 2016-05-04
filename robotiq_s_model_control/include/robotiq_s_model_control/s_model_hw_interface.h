#ifndef S_MODEL_HW_INTERFACE_H
#define S_MODEL_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robotiq_s_model_control/s_model_api.h>
#include <robotiq_s_model_control/s_model_diagnostics.h>
#include <robotiq_s_model_control/s_model_ros.h>

namespace robotiq_s_model_control
{

class SModelHWInterface : public hardware_interface::RobotHW
{
public:
    SModelHWInterface(ros::NodeHandle nh, boost::shared_ptr<SModelAPI> driver);

    void configure(hardware_interface::JointStateInterface &joint_state_interface, 
                   hardware_interface::PositionJointInterface &joint_position_interface);
    void read(ros::Duration d);
    void write(ros::Duration d);

protected:
    std::vector<std::string> joint_names_;

    std::vector<double> j_curr_pos_;
    std::vector<double> j_curr_vel_;
    std::vector<double> j_curr_eff_;
    std::vector<double> j_cmd_pos_;

    boost::shared_ptr<SModelAPI> hw_driver_;
    boost::shared_ptr<SModelDiagnostics> hw_diagnostics_;
    boost::shared_ptr<SModelROS> hw_ros_;
};
}

#endif // S_MODEL_HW_INTERFACE_H
