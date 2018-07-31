#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_robotiq_2f_gripper_action_server");

  ros::NodeHandle pnh("~");

  std::string gripper_name;
  pnh.param<std::string>("gripper_name", gripper_name, "gripper");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac(gripper_name, true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  control_msgs::GripperCommandGoal goal;
  goal.command.position = -0.01;
  goal.command.max_effort = 100.0;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
