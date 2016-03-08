// Copyright (c) <2016>, <Matthew Horn>
// All rights reserved.
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and/or
//  other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
//  specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ros/ros.h>
#include <iostream>
#include <rs_gripper_interface.h>

/**
 * @brief   Test the robotiq gripper code
 *
 * @version 1.0
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Nov 23, 2015
 */
 
int main(int argc, char** argv) {
  ros::init(argc, argv, "rs_gripper_interface_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  RSGripperInterface gripper = RSGripperInterface(false);
  ROS_INFO("[RSGripperInterfaceTest] activating");
  gripper.activate();
  
  //temp tests/////////////////////////////
  
  
  ros::Duration(2.0).sleep();
  
  //full functionality/////////////////////
  
  
  ROS_INFO("[RSGripperInterfaceTest] resetting");
  gripper.reset();
  
  ROS_INFO("[RSGripperInterfaceTest] testing setMode");
  gripper.setMode(RSGripperInterface::MODE_BASIC);
  ros::Duration(1.0).sleep();
  gripper.setMode(RSGripperInterface::MODE_PINCH);
  ros::Duration(1.0).sleep();
  gripper.setMode(RSGripperInterface::MODE_WIDE);
  ros::Duration(1.0).sleep();
  gripper.setMode(RSGripperInterface::MODE_SCISSOR);
  ros::Duration(1.0).sleep();
  
  ROS_INFO("[RSGripperInterfaceTest] testing setSpeed, fullOpen, fullClose");
  //slow close
  gripper.home();
  gripper.setSpeed(-1);
  gripper.fullClose();
  ros::Duration(1.0).sleep();
  
  //fast open
  gripper.setSpeed(300);
  gripper.fullOpen();
  ros::Duration(1.0).sleep();
  
  //medium close in pinch mode
  gripper.setSpeed(128);
  gripper.setPosition(128);

  gripper.setSpeed(255);
  gripper.setMode(RSGripperInterface::MODE_PINCH);
  gripper.setSpeed(128);
  gripper.setPosition(255, 0, 128);
};
