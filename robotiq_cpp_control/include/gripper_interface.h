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


#include <math.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <robotiq_cpp_control/RSGripperCommand.h>
#include <robotiq_cpp_control/RCGripperCommand.h>

#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

/**
 * @brief   Easy interface for working with arbitrary gripper
 * 		2 methods to use it:
 *		1) C++ objects, as seen in rs_gripper_interface_test.cpp
 *		2) Publish commands on the ROS topic "grip_command".
 * @version 1.0
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Dec 4, 2015
 */
class GripperInterface {
public:
  GripperInterface();
  // Basic Commands //////////////////////////////////////////////////////////////////////
  
  /// After calling this, the gripper's internal state should be reset, including clearing any faults.
  virtual void reset() {};
  /**
   * Shut down the gripper. activate() should have to be called after this if the
   * caller wants to do anything with the gripper.
   */
  virtual void deactivate() = 0;
  /// Prepare the gripper for motion. Should require call before any motion occurs.
  virtual void activate() = 0;
  /// Stop the robot as quickly as possible.
  virtual void eStop() = 0;
  
  // Adv Commands //////////////////////////////////////////////////////////////////////
  
  /// Move the robot to a generally "decent" starting position.
  virtual void home() = 0;
  
  /// Fully close the gripper
  virtual void fullOpen() = 0;
  /// Fully open the gripper
  virtual void fullClose() = 0;
  
  // Setters //////////////////////////////////////////////////////////////////////
  
  /**
   * Set the position of the fingers. 0 represents fully open, with higher numbers being
   * more and more closed. Units and range will depend on specific gripper.
   */
  void setPosition(int position) {};
  /**
   * The speed at which the gripper moves. 0 is slowest. Units and range will depend on specific gripper.
   */
  virtual void setSpeed(int speed) = 0;
  /**
   * Whether or not calls to this interface should block until the hardware is complete.
   * Grippers that are open-loop should override this method to disable blocking functionality.
   */
  void setBlocking(bool blocking);
  
protected:
  /**
   * Convenience: clamp a number between 0 and 255.
   */
  void clampByte(int& toClamp, std::string name) {};
  
  bool block;          /// If true, commands wait for closed loop completion before returning
  bool connected;
  bool activated;     /// If true, the gripper is ready to move.
  
private:
  /**
   * Apply changes to the hardware. Implement as empty for a simulated gripper.
   */
  virtual void sendCommand() = 0;
};

#endif
