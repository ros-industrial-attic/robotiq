// Copyright (c) 2016, Toyota Research Institute. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_hw_interface.h"

using namespace robotiq_3f_gripper_control;

Robotiq3FGripperHWInterface::Robotiq3FGripperHWInterface(ros::NodeHandle nh, boost::shared_ptr<Robotiq3FGripperAPI> driver)
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

    j_curr_pos_.resize(4, 0);
    j_curr_vel_.resize(4, 0);
    j_curr_eff_.resize(4, 0);

    j_cmd_pos_.resize(4, 0);

    hw_diagnostics_.reset(new Robotiq3FGripperDiagnostics(hw_driver_, hw_name));
    hw_ros_.reset(new Robotiq3FGripperROS(nh, hw_driver_, joint_names_, ros::Duration(1/rate)));
}

void Robotiq3FGripperHWInterface::configure(hardware_interface::JointStateInterface &joint_state_interface, hardware_interface::PositionJointInterface &joint_position_interface)
{
    //! Connect and register jonit state interface
    for (std::size_t joint_id = 0; joint_id < 4; ++joint_id)
    {
        // Create joint state interface
        joint_state_interface.registerHandle(hardware_interface::JointStateHandle(
                                                  joint_names_[joint_id],
                                                  &j_curr_pos_[joint_id],
                                                  &j_curr_vel_[joint_id],
                                                  &j_curr_eff_[joint_id]));

    }

    //! Connect and register joint position interface
    for(std::size_t joint_id = 0; joint_id < 4; ++joint_id)
    {
        // Create joint position interface
        joint_position_interface.registerHandle(hardware_interface::JointHandle(
                                                     joint_state_interface.getHandle(joint_names_[joint_id]),
                                                     &j_cmd_pos_[joint_id]));
    }
}

void Robotiq3FGripperHWInterface::read(ros::Duration d)
{
    hw_driver_->read();
    hw_driver_->getPosition(&j_curr_pos_[0], &j_curr_pos_[1], &j_curr_pos_[2], &j_curr_pos_[3]);
    hw_driver_->getCommandPos(&j_cmd_pos_[0], &j_cmd_pos_[1], &j_cmd_pos_[2], &j_cmd_pos_[3]);

    hw_diagnostics_->update();
    hw_ros_->publish();
}

void Robotiq3FGripperHWInterface::write(ros::Duration d)
{
    hw_driver_->setPosition(j_cmd_pos_[0], j_cmd_pos_[1], j_cmd_pos_[2], j_cmd_pos_[3]);
    hw_driver_->write();
}
