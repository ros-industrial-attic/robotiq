//  The MIT License (MIT)
//
//  Copyright (c) 2014 Jack Thompson
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.

/** @file s_model_joint_states.cpp
 *  Subscribes to Robotiq state messages on "SModelRobotInput" topic, converts the data to joint values,
 *  and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq S-model.
 * 
 *  'rosrun robotiq_joint_state_publisher s_model_joint_states <gripper_prefix>'
 * 
 *  @author jack.thompson(at)utexas.edu
 *  @maintainer karl.kruusamae(at)utexas.edu
 */

#include <vector>
#include <string>
#include <csignal>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>


const int BASIC   = 0;
const int PINCH   = 1;
const int WIDE    = 2;
const int SCISSOR = 3;

const double DEG_TO_RAD = M_PI/180.0;

/**
 * Finger class for single Robotiq S-model finger.
 */
class Finger {
 public:
  Finger() { position = 0; }				///< Default constructor for creating Finger, position is set 0
  Finger(int pos) { position = pos; }			///< Create Finger, position is @param pos
  Finger(const Finger &f) { position = f.position; }	///< Create Finger, position is taken form @param Finger
  inline double joint_1() const;			///< joint_1 value for Finger
  inline double joint_2() const;			///< joint_2 value for Finger
  inline double joint_3() const;			///< joint_3 value for Finger
  int position;						///< Position of the Finger
};

/**
 * Calculate joint_1 based on the Finger position.
 */
inline double Finger::joint_1() const {
  if(0 <= position && position <= 140) return (70.0/148.0 * DEG_TO_RAD) * position;
  else return 70.0 * DEG_TO_RAD;
}

/**
 * Calculate joint_2 based on the Finger position.
 */
inline double Finger::joint_2() const {
  if(0 <= position && position <= 140) return 0.0;
  else if(140 < position && position <= 240) return (90.0/100.0 * DEG_TO_RAD) * (position-140.0);
  else return 90.0 * DEG_TO_RAD;
}

/**
 * Calculate joint_3 based on the Finger position.
 */
inline double Finger::joint_3() const {
  if(0 <= position && position <= 140) return (-70.0/140.0 * DEG_TO_RAD) * position;
  else return -55.0 * DEG_TO_RAD;
}

/**
 * Robotiq S-model with three Fingers.
 */
class Robotiq3 {
 public:
  Robotiq3() { mode = BASIC; scissor = 137; joint_positions.resize(11, 0.0); }		///< Default constructor for creating Robotiq3
  inline double scissor_joint() const;							///< Joint values for so-called scissor joints
  void callback(const robotiq_s_model_control::SModel_robot_input::ConstPtr &msg);	///< Callback function for "SModelRobotInput" topic
  Finger finger_left;									///< Robotiq FINGER A
  Finger finger_right;									///< Robotiq FINGER B
  Finger finger_middle;									///< Robotiq FINGER C
  int scissor;										///< Scissor position
  int mode;										///< Robotiq grasping mode
  std::vector<double> joint_positions;							///< All the joint values
};

/**
 * Calculate scissor_joint based on the scissor position.
 */
inline double Robotiq3::scissor_joint() const {
  // fully open is +20, fully closed is -10
  // std::cout << scissor << std::endl;
  if(0 <= scissor && scissor <= 15) return 20.0*DEG_TO_RAD;
  else if(15 < scissor && scissor <= 240) return (20.0-30.0*((scissor-15.0)/225.0))*DEG_TO_RAD;
  else return -10.0*DEG_TO_RAD;
}

/**
 * Callback function for "SModelRobotInput" topic.
 * */
void Robotiq3::callback(const robotiq_s_model_control::SModel_robot_input::ConstPtr &msg) {
  finger_left = Finger(msg->gPOA);		// set left finger position
  finger_right = Finger(msg->gPOB);		// set right finger position
  finger_middle = Finger(msg->gPOC);		// set middle finger position
  scissor = msg->gPOS;				// set scissor position
  mode = msg->gMOD;				// set mode

  // if (mode == BASIC)        scissor = 137;
  // else if (mode == PINCH)   scissor = 255;
  // else if (mode == WIDE)    scissor = 0;
  // else if (mode == SCISSOR) scissor = 0;

  // Set all the joint values
  joint_positions.at(0)  =  scissor_joint();
  joint_positions.at(1)  =  finger_left.joint_1();
  joint_positions.at(2)  =  finger_left.joint_2();
  joint_positions.at(3)  =  finger_left.joint_3();
  joint_positions.at(4)  = -joint_positions.at(0);
  joint_positions.at(5)  =  finger_right.joint_1();
  joint_positions.at(6)  =  finger_right.joint_2();
  joint_positions.at(7)  =  finger_right.joint_3();
  joint_positions.at(8)  =  finger_middle.joint_1();
  joint_positions.at(9)  =  finger_middle.joint_2();
  joint_positions.at(10) =  finger_middle.joint_3();
}

/**
 * Main method.
 */
int main(int argc, char *argv[]) {

  // Create Robotiq3
  Robotiq3 robotiq;

  // ROS init, nodehandle, and rate
  ros::init(argc, argv, "s_model_joint_states");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);  // Hz

  // joint state publisher
  ros::Publisher joint_pub;
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  // robotiq state message subscriber
  ros::Subscriber joint_sub;
  joint_sub = nh.subscribe("SModelRobotInput", 10, &Robotiq3::callback, &robotiq);

  // wait until the URDF has been loaded to avoid publishing joints prematurely
  while (!ros::isShuttingDown() && !nh.hasParam("/robot_description")) {
    ROS_ERROR("[s_model_joint_states] Waiting for URDF");
    ros::Duration(0.1).sleep();
  }

  // set pseudo-namespace
  std::string gripper_prefix;
  if (argc > 1) gripper_prefix = argv[1];
  else gripper_prefix = "";

  // joint names for sensor_msgs::JointState message
  // order matters!
  std::vector<std::string> joint_names(11, "");
  joint_names.at(0).assign(gripper_prefix + "palm_finger_1_joint");
  joint_names.at(1).assign(gripper_prefix + "finger_1_joint_1");
  joint_names.at(2).assign(gripper_prefix + "finger_1_joint_2");
  joint_names.at(3).assign(gripper_prefix + "finger_1_joint_3");
  joint_names.at(4).assign(gripper_prefix + "palm_finger_2_joint");
  joint_names.at(5).assign(gripper_prefix + "finger_2_joint_1");
  joint_names.at(6).assign(gripper_prefix + "finger_2_joint_2");
  joint_names.at(7).assign(gripper_prefix + "finger_2_joint_3");
  joint_names.at(8).assign(gripper_prefix + "finger_middle_joint_1");
  joint_names.at(9).assign(gripper_prefix + "finger_middle_joint_2");
  joint_names.at(10).assign(gripper_prefix + "finger_middle_joint_3");

  // std::cout << "GRIPPER JOINT NAMES: ";
  // std::copy(joint_names.begin(), joint_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Output JointState message
  sensor_msgs::JointState joint_msg;
  // Joint names to JointState message
  joint_msg.name = joint_names;

  while (!ros::isShuttingDown()) {
    joint_msg.position = robotiq.joint_positions;
    joint_msg.header.stamp = ros::Time::now();
    joint_pub.publish(joint_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
