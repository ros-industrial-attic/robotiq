#include "robotiq_s_model_control/s_model_ros.h"

using namespace robotiq_s_model_control;

SModelROS::SModelROS(ros::NodeHandle &nh, boost::shared_ptr<SModelAPI> driver, std::vector<std::string> joint_names, ros::Duration desired_update_freq)
    :nh_(nh)
    ,driver_(driver)
    ,desired_update_freq_(desired_update_freq)
{
    //! advertise services
    init_srv_ = nh_.advertiseService("init", &SModelROS::handle_init, this);
    reset_srv_ = nh_.advertiseService("reset", &SModelROS::handle_reset, this);
    halt_srv_ = nh_.advertiseService("halt", &SModelROS::handle_halt, this);
    emerg_release_srv_ = nh_.advertiseService("emergency_release", &SModelROS::handle_emerg_release, this);
    shutdown_srv_ = nh_.advertiseService("shutdown", &SModelROS::handle_shutdown, this);

    //! advertise topics
    pos_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_cmd_states", 10);
    input_status_pub_ = nh.advertise<robotiq_s_model_control::SModel_robot_input>("input", 10);

    //! subscribers
    output_sub_ = nh.subscribe("output", 10, &SModelROS::handle_raw_cmd, this);

    //! setup dynamic reconfigure
    reconfigure_.reset(new ReconfigureServer(reconfigure_mutex_, nh_));
    ReconfigureServer::CallbackType f = boost::bind(&SModelROS::handle_reconfigure, this, _1, _2);
    reconfigure_->setCallback(f);

    pos_cmd_msg_.name = joint_names;
    pos_cmd_msg_.position.resize(joint_names.size(), 0);
    if(joint_names.size() != 4)
    {
        ROS_FATAL("Joint name size must be 4");
    }
}

void SModelROS::publish()
{
    driver_->getPositionCmd(pos_cmd_msg_.position[0], pos_cmd_msg_.position[1], pos_cmd_msg_.position[2], pos_cmd_msg_.position[3]);
    pos_cmd_pub_.publish(pos_cmd_msg_);

    driver_->getRaw(input_status_msg_);
    input_status_pub_.publish(input_status_msg_);
}

bool SModelROS::handle_init(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
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
    while(driver_->isStopped())
    {
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Ready to command. ";
    return true;
}

bool SModelROS::handle_reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_reset");
    //! shutdown
    handle_shutdown(req, resp);
    //! init
    handle_init(req, resp);
    return true;
}

bool SModelROS::handle_halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
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
    while(!driver_->isStopped())
    {
        desired_update_freq_.sleep();
    }
    resp.success = true;
    resp.message += "Device halted. ";
    return true;
}

bool SModelROS::handle_emerg_release(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_emerg_release");
    //! halt
    handle_halt(req, resp);
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

bool SModelROS::handle_shutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_shutdown");
    //! halt
    handle_halt(req, resp);
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

void SModelROS::handle_reconfigure(robotiq_s_model_control::SModelConfig &config, uint32_t level)
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

}

void SModelROS::handle_raw_cmd(const robotiq_s_model_control::SModel_robot_output::ConstPtr &msg)
{
    ROS_DEBUG_NAMED("RobotiqCANROS", "entered handle_raw_cmd");
    driver_->setRaw(*msg);
}
