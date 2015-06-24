/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq C-Model (2 finger) device
 */

#ifndef ROBOTIQ_C_MODEL_ACTION_SERVER_H
#define ROBOTIQ_C_MODEL_ACTION_SERVER_H

// STL
#include <string>
// ROS standard
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
// Repo specific includes
#include <robotiq_c_model_control/CModel_robot_input.h>
#include <robotiq_c_model_control/CModel_robot_output.h>


namespace robotiq_action_server
{

typedef robotiq_c_model_control::CModel_robot_input GripperInput;
typedef robotiq_c_model_control::CModel_robot_output GripperOutput;

typedef control_msgs::GripperCommandGoal GripperCommandGoal;
typedef control_msgs::GripperCommandFeedback GripperCommandFeedback;
typedef control_msgs::GripperCommandResult GripperCommandResult;

/**
 * @brief Structure containing the parameters necessary to translate
 *        GripperCommand actions to register-based commands to a
 *        particular gripper (and vice versa).
 *
 *        The min gap can be less than zero. This represents the case where the 
 *        gripper fingers close and then push forward.
 */
struct CModelGripperParams
{
  double min_gap_; // meters
  double max_gap_;
  double min_effort_; // N / (Nm)
  double max_effort_;
};

/**
 * @brief The CModelGripperActionServer class. Takes as arguments the name of the gripper it is to command,
 *        and a set of parameters that define the physical characteristics of the particular gripper.
 *        
 *        Listens for messages on input and publishes on output. Remap these.
 */
class CModelGripperActionServer
{
public:
  CModelGripperActionServer(const std::string& name, const CModelGripperParams& params);

  // These functions are meant to be called by simple action server
  void goalCB();
  void preemptCB();
  void analysisCB(const GripperInput::ConstPtr& msg);

private:
  void issueActivation();

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;

  ros::Subscriber state_sub_; // Subs to grippers "input" topic
  ros::Publisher goal_pub_; // Pubs to grippers "output" topic

  GripperOutput goal_reg_state_; // Goal information in gripper-register form
  GripperInput current_reg_state_; // State info in gripper-register form

  /* Used to translate GripperCommands in engineering units
   * to/from register states understood by gripper itself. Different
   * for different models/generations of Robotiq grippers */
  CModelGripperParams gripper_params_;

  std::string action_name_;
};

}
#endif
