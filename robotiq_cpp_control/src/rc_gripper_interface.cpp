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


#include "rc_gripper_interface.h"


RCGripperInterface::RCGripperInterface() :
  n(),
  spinner(2),
  command()
{
    spinner.start();
    // In case commands are sent via a ROS topic:
    gripperCommandSub = n.subscribe("grip_command",1, &RCGripperInterface::cb_command,this);    
    
    gripperCommandPub = n.advertise<robotiq_c_model_control::CModel_robot_output>("CModelRobotOutput",1);
    gripperStatusSub = n.subscribe("CModelRobotInput",1,&RCGripperInterface::cb_getGripperStatus,this);
    
    float printTime = 10, retryTime = 0.1;
    
    ros::Time start = ros::Time::now();
    while(
      (gripperCommandPub.getNumSubscribers() <= 0 || status.gPO == 0) && // wait for connection
      !ros::isShuttingDown())                                             // but stop if ROS shuts down
    {
      ROS_INFO_STREAM_THROTTLE(printTime, "[RCGripperInterface] Waiting for connection with gripper (" << (ros::Time::now() - start) << "s elasped)");
      ros::Duration(retryTime).sleep();
    }
    if(!ros::isShuttingDown()) {
      ROS_INFO("[RCGripperInterface] Connected to gripper");
      connected = true;
      
      //check for pre-activation
      if(status.gSTA == 3) {
	activated = true;
	command.rPR = status.gPO;
	command.rSP = 255;
  	command.rFR = 255;
	ROS_DEBUG_STREAM("[RCGripperInterface] Gripper already activated");
      }
    }
}

void RCGripperInterface::sendCommand() {
  if(!RCGripperInterface::isConnected()) {
    ROS_ERROR("[RCGripperInterface] Can't control the gripper, it's not connected or activated yet.");
    return;
  }

  if(!activated) {
    ROS_ERROR("[RCGripperInterface] Can't control the gripper, it's not activated yet. Call activate()");
    return;
  }
  
  gripperCommandPub.publish(command);
  return;
}

void RCGripperInterface::eStop() {
  ROS_FATAL("[RCGripperInterface] Beginning soft RS gripper E-stop");
  command.rATR = 1;
  sendCommand();
  if(block) {
    while(status.gFLT < 13) {
      ROS_WARN_DELAYED_THROTTLE(1, "Waiting for E-Stop to complete");
    }
  }
  ROS_FATAL("[RCGripperInterface] Finished soft RC gripper E-stop. Reactivate to start again");
}


void RCGripperInterface::reset()
{
  ROS_DEBUG_STREAM("[RCGripperInterface] Beginning reset");
  deactivate();
  command = robotiq_c_model_control::CModel_robot_output();
  activate();
  ROS_DEBUG_STREAM("[RCGripperInterface] Finished reset");
}

void RCGripperInterface::deactivate()
{
  ROS_DEBUG_STREAM("[RCGripperInterface] Deactivating");
  command.rACT = 0;
  sendCommand();
  // The following needs to be updated for a C-Model gripper
/*
  if(block) {
    while(status.gACT != 0) {
      ROS_WARN_DELAYED_THROTTLE(5, "[RCGripperInterface] Waiting for gripper to turn off...");
    }
    ROS_DEBUG_STREAM("[RCGripperInterface] Finished dectivation");
  }
*/
  activated = false;
}

void RCGripperInterface::activate()
{
  //check for pre-activation
  if(status.gSTA == 3) {
    activated = true;
    command.rPR = status.gPO;
    command.rSP = 255;
    command.rFR = 255;
    ROS_DEBUG_STREAM("[RCGripperInterface] Gripper already activated");
    return;
  }

  ROS_DEBUG_STREAM("[RCGripperInterface] Activating");

  command.rACT = 1;

  //don't call sendCommand, because sendCommand includes an activation check. Instead
  // just manually send the command
  gripperCommandPub.publish(command);
  while(status.gSTA != 3) {
    ROS_INFO_DELAYED_THROTTLE(10, "[RSGripperInterface] Waiting for activation to complete...");
  }
  ROS_DEBUG_STREAM("[RSGripperInterface] Finished activation");

  activated = true;
}

void RCGripperInterface::home()
{
  setSpeed(255);
  setForce(128);
  setPosition(0);
}

void RCGripperInterface::fullOpen()
{
  ROS_DEBUG_STREAM("[RCGripperInterface] Opening gripper");
  setPosition(0);
}

void RCGripperInterface::fullClose()
{
  ROS_DEBUG_STREAM("[RCGripperInterface] Closing gripper");
  setPosition(255);
}


void RCGripperInterface::setPosition(int position) {

  ROS_DEBUG_STREAM("[RCGripperInterface] Moving fingers to position " << position);
 
  command.rPR = position;
  command.rGTO = 1;
  sendCommand();
  
  // The following was copied from the Model S. No analog for the Model C?:

/*
  //if(block) {
    if(status.gPR != position) {
      while(status.gGTO != 0) {
	ROS_INFO_THROTTLE(5, "[RCGripperInterface] Waiting for move to begin...");
      }
    }
    while(status.gGTO) {
      ROS_INFO_THROTTLE(5, "[RCGripperInterface] Waiting for move to complete...");
    }
  //}
*/

}

void RCGripperInterface::setSpeed(int speed)
{

  ROS_DEBUG_STREAM("[RCGripperInterface] Setting fingers to speed " << speed);
  command.rSP = speed;
}

void RCGripperInterface::setForce(int force)
{

  ROS_DEBUG_STREAM("[RCGripperInterface] Setting fingers to force " << force);
  command.rFR = force;
}

void RCGripperInterface::cb_getGripperStatus(const robotiq_c_model_control::CModel_robot_input& msg)
{
  status = msg;
  
  switch(status.gFLT) {
    case 0:
      break;
    case 5:
      ROS_WARN_THROTTLE(1, "[RCGripperInterface] Activation is not complete!");
      break;
    case 6:
      ROS_WARN_THROTTLE(1, "[RCGripperInterface] Mode change is not complete!");
      break;
    case 7:
      ROS_WARN_THROTTLE(1, "[RCGripperInterface] Gripper is not activated yet!");
      break;
    case 9:
    case 10:
    case 11:
      ROS_ERROR_STREAM_THROTTLE(10, "[RCGripperInterface] Minor fault detected, #" << (int)status.gFLT);
      break;
    case 13:
    case 14:
    case 15:
      ROS_FATAL_STREAM_THROTTLE(10, "[RCGripperInterface] Major fault detected, #" << (int)status.gFLT);
      break;
    default:
      break;
  }
}

// Send commands via a ROS topic, as opposed to the other methods.
void RCGripperInterface::cb_command(const robotiq_cpp_control::RCGripperCommand& alpha)
{
  if (activated != true)
	activate();
  
  //order is important
  setForce(alpha.force);
  setPosition(alpha.position);
  setSpeed(alpha.speed);
  
  //sendCommand();
  gripperCommandPub.publish(command);

  ROS_DEBUG_STREAM("Moving");
}

bool RCGripperInterface::isConnected() {
  if(!connected) {
    ROS_WARN_THROTTLE(2.0, "[RCGripperInterface]: Not connected!");
    return false;
  }
  return true;
}
