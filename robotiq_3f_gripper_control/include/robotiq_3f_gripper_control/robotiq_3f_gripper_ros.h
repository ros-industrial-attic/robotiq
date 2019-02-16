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

#ifndef ROBOTIQ_3F_GRIPPER_ROS_H
#define ROBOTIQ_3F_GRIPPER_ROS_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_control/robotiq_3f_gripper_api.h>
#include <dynamic_reconfigure/server.h>
#include <robotiq_3f_gripper_control/Robotiq3FGripperConfig.h>

namespace robotiq_3f_gripper_control
{
class Robotiq3FGripperROS
{
public:
    Robotiq3FGripperROS(ros::NodeHandle& nh,
                  boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperAPI> driver, std::vector<std::string> joint_names,
                  ros::Duration desired_update_freq);

    void publish();

    bool handleInit(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleReset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleHalt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleEmergRelease(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleShutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);

    void handleReconfigure(robotiq_3f_gripper_control::Robotiq3FGripperConfig &config, uint32_t level=0);

    void handleRawCmd(const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput::ConstPtr &msg);

    void updateConfig(const robotiq_3f_gripper_control::Robotiq3FGripperConfig &config);
    void getCurrentConfig(robotiq_3f_gripper_control::Robotiq3FGripperConfig &config);

private:
    ros::NodeHandle nh_;
    boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperAPI> driver_;

    //! Services
    ros::ServiceServer init_srv_;
    ros::ServiceServer reset_srv_;
    ros::ServiceServer halt_srv_;
    ros::ServiceServer emerg_release_srv_;
    ros::ServiceServer shutdown_srv_;

    //! Topics
    ros::Publisher input_status_pub_;
    ros::Subscriber output_sub_;

    //! Settings
    ros::Duration desired_update_freq_;

    //! Reconfigure
    typedef dynamic_reconfigure::Server<robotiq_3f_gripper_control::Robotiq3FGripperConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_;
    boost::recursive_mutex reconfigure_mutex_;
    robotiq_3f_gripper_control::Robotiq3FGripperConfig config_;

    robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput input_status_msg_;

};
} //end namespace robotiq_3f_gripper_control

#endif // ROBOTIQ_3F_GRIPPER_ROS_H
