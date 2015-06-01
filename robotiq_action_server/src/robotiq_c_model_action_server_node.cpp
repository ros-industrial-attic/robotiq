#include "robotiq_action_server/robotiq_c_model_action_server.h"

namespace
{
  // Defines a default for the c2 model 85 gripper
  robotiq_action_server::CModelGripperParams c2_85_defaults()
  {
    robotiq_action_server::CModelGripperParams params;
    params.min_gap_ = -.017;
    params.max_gap_ = 0.085;
    params.min_effort_ = 40.0; // This is a guess. Could not find data with quick search.
    params.max_effort_ = 100.0;

    return params;
  }
}

int main(int argc, char** argv)
{
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "gripper_action_server");
  
  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string gripper_name;
  private_nh.param<std::string>("gripper_name", gripper_name, "gripper");

  // Fill out C-Model Params
  robotiq_action_server::CModelGripperParams cparams = c2_85_defaults();
  
  // Min because fingers can push forward before the mechanical stops are reached
  private_nh.param<double>("min_gap", cparams.min_gap_, cparams.min_gap_);
  private_nh.param<double>("max_gap", cparams.max_gap_, cparams.max_gap_);
  private_nh.param<double>("min_effort", cparams.min_effort_, cparams.min_effort_);
  private_nh.param<double>("max_effort", cparams.max_effort_, cparams.max_effort_);

  ROS_INFO("Initializing Robotiq action server for gripper: %s", gripper_name.c_str());

  // The name of the gripper -> this server communicates over name/inputs and name/outputs
  robotiq_action_server::CModelGripperActionServer gripper (gripper_name, cparams);

  ROS_INFO("Robotiq action-server spinning for gripper: %s", gripper_name.c_str());
  ros::spin();
}
