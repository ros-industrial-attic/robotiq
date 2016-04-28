#include "robotiq_s_model_control/s_model_hw_interface.h"

using namespace robotiq_s_model_control;

SModelHWInterface::SModelHWInterface(ros::NodeHandle nh, boost::shared_ptr<SModelAPI> driver)
    :hw_driver_(driver)
{
    double rate;
    nh.param<double>("update_rate", rate, 20);

    std::string hw_name;
    hw_name = nh.param<std::string>("hw_name", "robotiq_s");

    std::string prefix;
    prefix = nh.param<std::string>("prefix", "");

    joint_names_.push_back(prefix+"finger_1_joint_1");
    joint_names_.push_back(prefix+"finger_2_joint_1");
    joint_names_.push_back(prefix+"finger_middle_joint_1");
    joint_names_.push_back(prefix+"palm_finger_1_joint");

    joint_names_ = nh.param< std::vector<std::string> >("joint_names", joint_names_);

    if(joint_names_.size()!=4)
    {
        throw std::runtime_error("There must be 4 joint names");
    }

    j_prev_pos.resize(4, 0);
    j_curr_pos.resize(4, 0);
    j_curr_vel.resize(4, 0);
    j_curr_eff.resize(4, 0);

    j_cmd_pos.resize(4, 0);

    hw_diagnostics_.reset(new SModelDiagnostics(hw_driver_, hw_name));
    hw_ros_.reset(new SModelROS(nh, hw_driver_, joint_names_, ros::Duration(1/rate)));
}

void SModelHWInterface::configure(hardware_interface::JointStateInterface &joint_state_interface, hardware_interface::PositionJointInterface &joint_position_interface)
{
    //! Connect and register jonit state interface
    for (std::size_t joint_id = 0; joint_id < 4; ++joint_id)
    {
        // Create joint state interface
        joint_state_interface.registerHandle(hardware_interface::JointStateHandle(
                                                  joint_names_[joint_id],
                                                  &j_curr_pos[joint_id],
                                                  &j_curr_vel[joint_id],
                                                  &j_curr_eff[joint_id]));

    }

    //! Connect and register joint position interface
    for(std::size_t joint_id = 0; joint_id < 4; ++joint_id)
    {
        // Create joint position interface
        joint_position_interface.registerHandle(hardware_interface::JointHandle(
                                                     joint_state_interface.getHandle(joint_names_[joint_id]),
                                                     &j_cmd_pos[joint_id]));
    }
}

void SModelHWInterface::read(ros::Duration d)
{
    hw_driver_->read();
    hw_driver_->getPosition(j_curr_pos[0], j_curr_pos[1], j_curr_pos[2], j_curr_pos[3]);
    for(std::size_t joint_id = 0; joint_id < 4; ++joint_id)
    {
        j_curr_vel[joint_id] = (j_curr_pos[joint_id] - j_prev_pos[joint_id])/d.toSec();
    }
    j_prev_pos = j_curr_pos;
    hw_driver_->getCurrent(j_curr_eff[0], j_curr_eff[1], j_curr_eff[2], j_curr_eff[3]);
    hw_diagnostics_->update();
    hw_ros_->publish();
}

void SModelHWInterface::write(ros::Duration d)
{
    hw_driver_->setPosition(j_cmd_pos[0], j_cmd_pos[1], j_cmd_pos[2], j_cmd_pos[3]);
    hw_driver_->write();
}

