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

#include "robotiq_s_model_control/s_model_ros.h"

using namespace robotiq_s_model_control;

SModelROS::SModelROS(ros::NodeHandle &nh, boost::shared_ptr<SModelAPI> driver, std::vector<std::string> joint_names, ros::Duration desired_update_freq)
    :nh_(nh)
    ,driver_(driver)
    ,desired_update_freq_(desired_update_freq)
{
    //! advertise services
    init_srv_ = nh_.advertiseService("init", &SModelROS::handleInit, this);
    reset_srv_ = nh_.advertiseService("reset", &SModelROS::handleReset, this);
    halt_srv_ = nh_.advertiseService("halt", &SModelROS::handleHalt, this);
    emerg_release_srv_ = nh_.advertiseService("emergency_release", &SModelROS::handleEmergRelease, this);
    shutdown_srv_ = nh_.advertiseService("shutdown", &SModelROS::handleShutdown, this);

    //! advertise topics
    input_status_pub_ = nh.advertise<robotiq_s_model_control::SModel_robot_input>("input", 10);

    //! subscribers
    output_sub_ = nh.subscribe("output", 10, &SModelROS::handleRawCmd, this);

    //! setup dynamic reconfigure
    reconfigure_.reset(new ReconfigureServer(reconfigure_mutex_, nh_));
    ReconfigureServer::CallbackType f = boost::bind(&SModelROS::handleReconfigure, this, _1, _2);
    reconfigure_->setCallback(f);

    if(joint_names.size() != 4)
    {
        ROS_FATAL("Joint name size must be 4");
    }
}

void SModelROS::publish()
{
    driver_->getRaw(&input_status_msg_);
    input_status_pub_.publish(input_status_msg_);
}

bool SModelROS::handleInit(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_init");
    //! set controller state (INIT_ACTIVATION)
    driver_->setInitialization(INIT_ACTIVATION);
    //! wait for controller state (GRIPPER_READY)
    while(!driver_->isReady())
    {
        desired_update_freq_.sleep();
    }
    resp.message += "Init succeeded. ";
    //! set action mode to go (ACTION_GO)
    driver_->setActionMode(ACTION_GO);
    //! wait for controller state (ACTION_GO)
    while(driver_->isHalted())
    {
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Ready to command. ";
    return true;
}

bool SModelROS::handleReset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_reset");
    //! shutdown
    handleShutdown(req, resp);
    //! init
    handleInit(req, resp);
    return true;
}

bool SModelROS::handleHalt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_halt");
    //! check for activation
    if(!driver_->isInitialized())
    {
        resp.success = false;
        resp.message = "Not initialized. ";
        return true;
    }
    //! set controller state (ACTION_STOP)
    driver_->setActionMode(ACTION_STOP);
    //! wait for controller state (ACTION_STOP)
    while(!driver_->isHalted())
    {
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Device halted. ";
    return true;
}

bool SModelROS::handleEmergRelease(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_emerg_release");
    //! halt
    handleHalt(req, resp);
    //! set controller state (EMERGENCY_RELEASE_ENGAGED)
    driver_->setEmergencyRelease(EMERGENCY_RELEASE_ENGAGED);
    //! wait for controller state (ERROR_AUTOMATIC_RELEASE_COMPLETED)
    while(!driver_->isEmergReleaseComplete())
    {
        desired_update_freq_.sleep();
    }
    driver_->setEmergencyRelease(EMERGENCY_RELEASE_IDLE);
    resp.success = true;
    resp.message += "Emergency release complete. ";

}

bool SModelROS::handleShutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_shutdown");
    //! halt
    handleHalt(req, resp);
    //! set controller state (INIT_RESET)
    driver_->setInitialization(INIT_RESET);
    //! wait for controller state (INIT_RESET)
    while(driver_->isInitialized())
    {
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Shutdown complete. ";
    return true;

}

void SModelROS::handleReconfigure(robotiq_s_model_control::SModelConfig &config, uint32_t level)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_reconfigure");
    driver_->setInidividualControlMode((robotiq::IndividualControl)config.ind_control_fingers, (robotiq::IndividualControl)config.ind_control_scissor);
    if (!config.ind_control_scissor)
    {
        driver_->setGraspingMode((robotiq::GraspingMode)config.mode);

        while(!driver_->isModeSet((robotiq::GraspingMode)config.mode))
        {
            desired_update_freq_.sleep();
            ROS_DEBUG_STREAM("waiting for mode "<<config.mode<<" to be set");
        }
    }
    //! @todo automatically switch controller based on mode?
    //! @todo add individual velocity and force control

    driver_->setVelocity(config.velocity, config.velocity, config.velocity, config.velocity);
    driver_->setForce(config.force, config.force, config.force, config.force);

    config_ = config;
}

void SModelROS::updateConfig(const robotiq_s_model_control::SModelConfig &config)
{
    reconfigure_->updateConfig(config);
}

void SModelROS::getCurrentConfig(robotiq_s_model_control::SModelConfig &config)
{
    config = config_;
}

void SModelROS::handleRawCmd(const robotiq_s_model_control::SModel_robot_output::ConstPtr &msg)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_raw_cmd");
    driver_->setRaw(*msg);
}
