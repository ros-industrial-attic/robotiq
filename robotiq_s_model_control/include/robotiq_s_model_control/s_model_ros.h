#ifndef S_MODEL_ROS_H
#define S_MODEL_ROS_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/s_model_api.h>
#include <dynamic_reconfigure/server.h>
#include <robotiq_s_model_control/SModelConfig.h>

namespace robotiq_s_model_control
{
class SModelROS
{
public:
    SModelROS(ros::NodeHandle& nh,
                  boost::shared_ptr<robotiq_s_model_control::SModelAPI> driver, std::vector<std::string> joint_names,
                  ros::Duration desired_update_freq);

    void publish();

    bool handleInit(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleReset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleHalt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleEmergRelease(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
    bool handleShutdown(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);

    void handleReconfigure(robotiq_s_model_control::SModelConfig &config, uint32_t level=0);

    void handleRawCmd(const robotiq_s_model_control::SModel_robot_output::ConstPtr &msg);

    void updateConfig(const robotiq_s_model_control::SModelConfig &config);
    void getCurrentConfig(robotiq_s_model_control::SModelConfig &config);

private:
    ros::NodeHandle nh_;
    boost::shared_ptr<robotiq_s_model_control::SModelAPI> driver_;

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
    typedef dynamic_reconfigure::Server<robotiq_s_model_control::SModelConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_;
    boost::recursive_mutex reconfigure_mutex_;
    robotiq_s_model_control::SModelConfig config_;

    robotiq_s_model_control::SModel_robot_input input_status_msg_;

};
} //end namespace robotiq_s_model_control

#endif // S_MODEL_ROS_H
