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

#ifndef ROBOTIQ_3F_GRIPPER_HW_INTERFACE_H
#define ROBOTIQ_3F_GRIPPER_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robotiq_3f_gripper_control/robotiq_3f_gripper_api.h>
#include <robotiq_3f_gripper_control/robotiq_3f_gripper_diagnostics.h>
#include <robotiq_3f_gripper_control/robotiq_3f_gripper_ros.h>

namespace robotiq_3f_gripper_control
{

class Robotiq3FGripperHWInterface : public hardware_interface::RobotHW
{
public:
    Robotiq3FGripperHWInterface(ros::NodeHandle nh, boost::shared_ptr<Robotiq3FGripperAPI> driver);

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

    boost::shared_ptr<Robotiq3FGripperAPI> hw_driver_;
    boost::shared_ptr<Robotiq3FGripperDiagnostics> hw_diagnostics_;
    boost::shared_ptr<Robotiq3FGripperROS> hw_ros_;
};
}

#endif //ROBOTIQ_3F_GRIPPER_HW_INTERFACE_H
