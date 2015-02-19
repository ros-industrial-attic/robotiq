/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq C-Model device
 */

#include "robotiq_action_server/robotiq_c_model_action_server.h"

// To keep the fully qualified names managable

//Anonymous namespaces are file local -> sort of like global static objects
namespace
{
  using namespace robotiq_action_server;

  /*  This struct is declared for the sole purpose of being used as an exception internally
      to keep the code clean (i.e. no output params). It is caught by the action_server and 
      should not propogate outwards. If you use these functions yourself, beware.
  */
  struct BadArgumentsError {};


  GripperOutput goalToRegisterState(const GripperCommandGoal& goal, const CModelGripperParams& params)
  {
    GripperOutput result;
    result.rACT = 0x1; // active gripper
    result.rGTO = 0x1; // go to position
    result.rATR = 0x0; // No emergency release
    result.rSP = 128; // Middle ground speed
    
    if (goal.command.position > params.max_gap_ || goal.command.position < params.min_gap_)
    {
      ROS_WARN("Goal gripper gap size is out of range(%f to %f): %f m",
               params.min_gap_, params.max_gap_, goal.command.position);
      throw BadArgumentsError();
    }
    
    if (goal.command.max_effort < params.min_effort_ || goal.command.max_effort > params.max_effort_)
    {
      ROS_WARN("Goal gripper effort out of range (%f to %f N): %f N",
               params.min_effort_, params.max_effort_, goal.command.max_effort);
      throw BadArgumentsError();
    }

    double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.rPR = static_cast<uint8_t>((params.max_gap_ - goal.command.position) / dist_per_tick);
    result.rFR = static_cast<uint8_t>((goal.command.max_effort - params.min_effort_) / eff_per_tick);

    ROS_INFO("Setting goal position register to %hhu", result.rPR);

    return result;
  }

  /*  This function is templatized because both GripperCommandResult and GripperCommandFeedback consist
      of the same fields yet have different types. Templates here act as a "duck typing" mechanism to avoid
      code duplication.
  */
  template<typename T>
  T registerStateToResultT(const GripperInput& input, const CModelGripperParams& params, uint8_t goal_pos)
  {
    T result;
    double dist_per_tick = (params.max_gap_ - params.min_gap_) / 255;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.position = input.gPO * dist_per_tick + params.min_gap_;
    result.effort = input.gCU * eff_per_tick + params.min_effort_;
    result.stalled = input.gOBJ == 0x1 || input.gOBJ == 0x2;
    result.reached_goal = input.gPO == goal_pos;

    return result;
  }

  // Inline api-transformers to avoid confusion when reading the action_server source
  inline
  GripperCommandResult registerStateToResult(const GripperInput& input, const CModelGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandResult>(input, params, goal_pos);
  }

  inline
  GripperCommandFeedback registerStateToFeedback(const GripperInput& input, const CModelGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandFeedback>(input, params, goal_pos);
  }

} // end of anon namespace

namespace robotiq_action_server
{

CModelGripperActionServer::CModelGripperActionServer(const std::string& name, const CModelGripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
{
  as_.registerGoalCallback(boost::bind(&CModelGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&CModelGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe("input", 1, &CModelGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>("output", 1);

  as_.start();
}

void CModelGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
  if (current_reg_state_.gSTA != 0x3)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
    return;
  }

  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    goal_reg_state_ = goalToRegisterState(current_goal, gripper_params_);
    goal_pub_.publish(goal_reg_state_);
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
  }
}

void CModelGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void CModelGripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
{
  current_reg_state_ = *msg;

  if (!as_.isActive()) return;

  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gSTA != 0x3)
  {
    // Check to see if the gripper is active or if it has been asked to be active
    if (current_reg_state_.gSTA == 0x0 && goal_reg_state_.rACT != 0x1)
    {
      // If it hasn't been asked, active it
      issueActivation();
    }

    // Otherwise wait for the gripper to activate
    // TODO: If message delivery isn't guaranteed, then we may want to resend activate
    return;
  }

  // Check for errors
  if (current_reg_state_.gFLT)
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
    as_.setAborted(registerStateToResult(current_reg_state_,
                                         gripper_params_,
                                         goal_reg_state_.rPR));
  }
  else if (current_reg_state_.gGTO && current_reg_state_.gOBJ && current_reg_state_.gPR == goal_reg_state_.rPR)
  {
    // If commanded to move and if at a goal state and if the position request matches the echo'd PR, we're
    // done with a move
    ROS_INFO("%s succeeded", action_name_.c_str());
    as_.setSucceeded(registerStateToResult(current_reg_state_,
                                           gripper_params_,
                                           goal_reg_state_.rPR));
  }
  else
  {
    // Publish feedback
    as_.publishFeedback(registerStateToFeedback(current_reg_state_,
                                                gripper_params_,
                                                goal_reg_state_.rPR));
  }
}

void CModelGripperActionServer::issueActivation()
{
  ROS_INFO("Activating gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  // other params should be zero
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}
} // end robotiq_action_server namespace