// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/** @file robotiq_3f_gripper_joint_states.cpp
 *  Subscribes to Robotiq state messages on "Robotiq3FGripperRobotInput" topic, converts the data to joint values,
 *  and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq 3F gripper.
 * 
 *  'rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states <gripper_prefix>'
 * 
 *  @author jack.thompson(at)utexas.edu
 *  @author karl.kruusamae(at)utexas.edu
 */

#include <vector>
#include <string>
#include <csignal>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>


const double DEG_TO_RAD = M_PI/180.0;

/**
 * Finger class for single Robotiq 3F gripper finger.
 */
class Finger {
 public:
  Finger() { position = 0; }				///< Default constructor for creating Finger, position is set 0
  Finger(int pos) { position = pos; }			///< Create Finger, position is @param pos
  Finger(const Finger &f) { position = f.position; }	///< Create Finger, position is taken form @param Finger
  inline double joint1() const;				///< joint_1 value for Finger
  inline double joint2() const;				///< joint_2 value for Finger
  inline double joint3() const;				///< joint_3 value for Finger
  int position;						///< Position of the Finger
};

/**
 * Calculate joint1 based on the Finger position. Assumes that fingers are in a non-interfering state.
 * See Section 4.1 in https://www.cs.indiana.edu/ftp/techreports/TR711.pdf
 */
inline double Finger::joint1() const {
  if(0 <= position && position <= 140) return (70.0/148.0 * DEG_TO_RAD) * position;
  else return 70.0 * DEG_TO_RAD;
}

/**
 * Calculate joint2 based on the Finger position. Assumes that fingers are in a non-interfering state.
 * See section 4.1 in https://www.cs.indiana.edu/ftp/techreports/TR711.pdf
 */
inline double Finger::joint2() const {
  if(0 <= position && position <= 140) return 0.0;
  else if(140 < position && position <= 240) return (90.0/100.0 * DEG_TO_RAD) * (position-140.0);
  else return 90.0 * DEG_TO_RAD;
}

/**
 * Calculate joint3 based on the Finger position. Assumes that fingers are in a non-interfering state.
 * See section 4.1 in https://www.cs.indiana.edu/ftp/techreports/TR711.pdf
 */
inline double Finger::joint3() const {
  if(0 <= position && position <= 140) return (-70.0/140.0 * DEG_TO_RAD) * position;
  else return -55.0 * DEG_TO_RAD;
}

/**
 * Robotiq 3F gripper with three Fingers.
 */
class Robotiq3 {
 public:
  /** Default constructor for Robotiq3 */
  Robotiq3() {
    scissor = 137;
    joint_positions.resize(11, 0.0);
    prefix = "";
  }
  
  /** Constructor for Robotiq3 */
  Robotiq3(std::string gripper_prefix) {
    scissor = 137;
    joint_positions.resize(11, 0.0);
    prefix = gripper_prefix;    
  }
  
  inline double scissorJoint() const;							///< Joint value for so-called scissor joint
  void callback(const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput::ConstPtr &msg);	///< Callback function for "Robotiq3FGripperRobotInput" topic
  Finger finger_left;									///< Robotiq FINGER A
  Finger finger_right;									///< Robotiq FINGER B
  Finger finger_middle;									///< Robotiq FINGER C
  int scissor;										///< Scissor position
  std::string prefix;									///< Gripper prefix
  std::vector<std::string> jointNames();						///< Joint names
  std::vector<double> joint_positions;							///< Joint values
};

/**
 * Calculates joint value of the scissor joint based on the scissor position.
 */
inline double Robotiq3::scissorJoint() const {
  // Max range for scissor mode should be 32 degrees [http://support.robotiq.com/display/IMB/6.1+Technical+dimensions].
  // That would mean that the limits of a single joint are -16 and +16.
  // By actually measuring the joint angles, the limits appear to be approximately at -11 and +11.
  if(0 <= scissor && scissor <= 15) return 11.0*DEG_TO_RAD;
  else if(15 < scissor && scissor <= 240) return (11.0-22.0*((scissor-15.0)/225.0))*DEG_TO_RAD;
  else return -11.0*DEG_TO_RAD;
}

/**
 * Callback function for "Robotiq3FGripperRobotInput" topic.
 */
void Robotiq3::callback(const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput::ConstPtr &msg) {
  finger_left = Finger(msg->gPOA);		// set left finger position
  finger_right = Finger(msg->gPOB);		// set right finger position
  finger_middle = Finger(msg->gPOC);		// set middle finger position
  scissor = msg->gPOS;				// set scissor position

  // Set all the joint values
  joint_positions.at(0)  =  scissorJoint();
  joint_positions.at(1)  =  finger_left.joint1();
  joint_positions.at(2)  =  finger_left.joint2();
  joint_positions.at(3)  =  finger_left.joint3();
  joint_positions.at(4)  = -joint_positions.at(0);
  joint_positions.at(5)  =  finger_right.joint1();
  joint_positions.at(6)  =  finger_right.joint2();
  joint_positions.at(7)  =  finger_right.joint3();
  joint_positions.at(8)  =  finger_middle.joint1();
  joint_positions.at(9)  =  finger_middle.joint2();
  joint_positions.at(10) =  finger_middle.joint3();
}

/**
 * Assigns appropriate joint names.
 */
inline std::vector<std::string>  Robotiq3::jointNames() {
  // joint names for sensor_msgs::JointState message
  // order matters!
  std::vector<std::string> joint_names(11, "");
  joint_names.at(0).assign(prefix + "palm_finger_1_joint");
  joint_names.at(1).assign(prefix + "finger_1_joint_1");
  joint_names.at(2).assign(prefix + "finger_1_joint_2");
  joint_names.at(3).assign(prefix + "finger_1_joint_3");
  joint_names.at(4).assign(prefix + "palm_finger_2_joint");
  joint_names.at(5).assign(prefix + "finger_2_joint_1");
  joint_names.at(6).assign(prefix + "finger_2_joint_2");
  joint_names.at(7).assign(prefix + "finger_2_joint_3");
  joint_names.at(8).assign(prefix + "finger_middle_joint_1");
  joint_names.at(9).assign(prefix + "finger_middle_joint_2");
  joint_names.at(10).assign(prefix + "finger_middle_joint_3");
  return joint_names;
}

/**
 * Main method.
 */
int main(int argc, char *argv[]) {
  
  // ROS init, nodehandle, and rate
  ros::init(argc, argv, "robotiq_3f_gripper_joint_states");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate loop_rate(20);  // Hz

  // set user-specified prefix
  std::string gripper_prefix;
  pnh.param<std::string>("prefix", gripper_prefix, "");

  // Create Robotiq3
  Robotiq3 robotiq(gripper_prefix);

  // joint state publisher
  ros::Publisher joint_pub;
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  // robotiq state message subscriber
  ros::Subscriber joint_sub;
  joint_sub = nh.subscribe("Robotiq3FGripperRobotInput", 10, &Robotiq3::callback, &robotiq);
  
  // Output JointState message
  sensor_msgs::JointState joint_msg;
  
  // Joint names to JointState message
  joint_msg.name = robotiq.jointNames();

  while (ros::ok()) {
    joint_msg.position = robotiq.joint_positions;
    joint_msg.header.stamp = ros::Time::now();
    joint_pub.publish(joint_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
