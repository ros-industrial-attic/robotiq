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

#include <gripper_interface.h>

//ROS messages
#include <robotiq_c_model_control/CModel_robot_output.h>
#include <robotiq_c_model_control/CModel_robot_input.h>

#ifndef RC_GRIPPER_INTERFACE_H
#define RC_GRIPPER_INTERFACE_H

/**
 * @brief   Easy interface for working with robotiq gripper.
 * 
 * For details on the interface that this class implements, see these page:
 * http://support.robotiq.com/pages/viewpage.action?pageId=590045 (status)
 * Despite being called "input registers", these are the status messages returned by the gripper.
 * 
 * http://support.robotiq.com/pages/viewpage.action?pageId=590044 (command)
 * These are the bits you set to command the gripper to different positions.
 *
 * Also see the implementations of cb_getGripperStatus and sendCommand, which list the possible values
 * for the inputs and outputs.
 * 
 * 
 * WARNING: This interface does not support scissor mode, so it is disabled for now.
 * TODO To implement scissor mode:
 *   - Determine position limits in scissor mode
 *   - Add functionality to setPosition(single)
 *   - Add warning message to setPosition(all three), then call setPosition(single) with first finger position
 *   - Remove hard block/error message for scissor mode in setMode(enum)
 *   - Implement fullClose() for scissor
 *   - Add test cases to rs_gripper_interface_test.cpp
 *
 * @version 1.0
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @author  Matt Horn <mwhorn@utexas.edu, mwhorn1008@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Nov 23, 2015
 */
class RCGripperInterface : public GripperInterface {
public:
  /**
   * Possible modes for the gripper
   */
  enum Mode {
    MODE_BASIC = 0,
    MODE_PINCH,
    MODE_WIDE,
    MODE_SCISSOR
  };

  
  //Default constructor
  RCGripperInterface();
  
  // ROS callbacks
  void cb_getGripperStatus(const robotiq_c_model_control::CModel_robot_input& msg);
  void cb_command(const robotiq_cpp_control::RCGripperCommand& alpha);
  
  // Basic Commands //////////////////////////////////////////////////////////////////////////
  void reset();
  void deactivate();
  void activate();
  void eStop();
  
  // Adv Commands //////////////////////////////////////////////////////////////////////////
  ///Go to basic mode, fully open, medium force, full speed
  void home() ;
  
  /**
   * Fully open in whatever the current mode is
   * WARNING: scissor mode is not implemented
   */
  void fullOpen() ;
  
  /**
   * Fully close in whatever the current mode is
   * WARNING: scissor mode is not implemented
   */
  void fullClose() ;
  
  // Setters //////////////////////////////////////////////////////////////////////////
  
  /**
   * Set the gripper mode and move to the open position for that mode.
   * When new modes are sent to the S-Model, it automatically goes to the open position.
   * This behavior cannot be supressed.
   */
  void setMode(Mode newMode);
  
  /**
   * Set all fingers to the same position. 0 is open, 255 is closed in basic mode, 107 is closed in pinch mode.
   * WARNING: scissor mode is not implemented
   */
  void setPosition(int position);
  
  /// Set speed of all fingers.
  void setSpeed(int speed) ;
  
  
  /// Unique to the 3-finger gripper: set the positions of each finger individually.
  void setPosition(int positionA, int positionB, int positionC);
  
  /// Control the force of the grip.
  void setForce(int force);
  
  // Getters //////////////////////////////////////////////////////////////////////////
  
  /**
   * Robotiq C-Model has built-in sensors to detect whether or not the fingers closed
   * fully, or if they were stopped by an object. This method returns whether or not
   * the fingers closed as expected.
   */
  bool isObjectHeld();
  
private:
  bool isConnected();
  
  void clampByte(int& toClamp, std::string name);
  
  ///  to send specific C-Model command.
  void sendCommand() ;
  
  /// Internal mode setter (after enum has been verified).
  void setMode(int newMode);
  
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  
  ros::Subscriber gripperCommandSub;                      /// Listen for people trying to use this gripper
  
  ros::Subscriber gripperStatusSub;                       /// Listen to the gripper
  robotiq_c_model_control::CModel_robot_input status;     /// The status returned from the gripper
  
  robotiq_c_model_control::CModel_robot_output command;   /// Store the command to send to the gripper
  ros::Publisher gripperCommandPub;                       /// Used to send the command to the gripper
};

#endif
