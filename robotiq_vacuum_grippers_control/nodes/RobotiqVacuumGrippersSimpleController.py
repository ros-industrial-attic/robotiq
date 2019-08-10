#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a vacuum gripper.

This serves as an example for publishing messages on the 'RobotiqVacuumGrippersRobotOutput' topic using the 'RobotiqVacuumGrippers_robot_output' msg type for sending commands to a vacuum gripper.
"""

import roslib; roslib.load_manifest('robotiq_vacuum_grippers_control')
import rospy
from robotiq_vacuum_grippers_control.msg import _RobotiqVacuumGrippers_robot_output  as outputMsg
from time import sleep


def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.RobotiqVacuumGrippers_robot_output();
        command.rACT = 1
	command.rMOD = 0
        command.rGTO = 1
	command.rPR = 0
        command.rSP  = 150
        command.rFR  = 50

    if char == 'r':
        command = outputMsg.RobotiqVacuumGrippers_robot_output();
        command.rACT = 0

    if char == 'g':
        command.rPR = 0

    if char == 'c':
        command.rPR = 255   

    if char == 'm':
        command.rMOD = 0   

    if char == 'v':
        command.rMOD = 1   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = '\n\n\nSimple vacuum Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  rACT = '  + str(command.rACT)
    currentCommand += '  rMOD = '  + str(command.rMOD)
    currentCommand += ', rGTO = '  + str(command.rGTO)
    currentCommand += ', rATR = '  + str(command.rATR)
    currentCommand += ', rPR = '   + str(command.rPR )
    currentCommand += ', rSP = '   + str(command.rSP )
    currentCommand += ', rFR = '   + str(command.rFR )


    print currentCommand

    strAskForCommand  = '\n-----Available commands:\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'g: Grip\n'
    strAskForCommand += 'c: Release\n'
    strAskForCommand += 'm: Change mode--> Automatic mode (rMOD=0x00)\n'
    strAskForCommand += 'v: Change mode--> Advanced mode (rMOD=0x01)\n'
    strAskForCommand += 'f: Increase Action timeout\n'
    strAskForCommand += 'l: Decrease Action timeout\n'
    strAskForCommand += 'i: Increase Minimum vacuum / pressure request\n'
    strAskForCommand += 'd: Decrease Minimum vacuum / pressure request\n'
    strAskForCommand += '0-255: Pressure request, ONLY VALID IN MANUAL MODE, i.e. when rMOD=0x01\n'
    strAskForCommand += '\n-->'

    return raw_input(strAskForCommand)

def publisher():
    """Main loop which requests new commands and publish them on the RobotiqVacuumGrippersRobotOutput topic."""
    rospy.init_node('RobotiqVacuumGrippersSimpleController')
    
    pub = rospy.Publisher('RobotiqVacuumGrippersRobotOutput', outputMsg.RobotiqVacuumGrippers_robot_output)

    command = outputMsg.RobotiqVacuumGrippers_robot_output();

    while not rospy.is_shutdown():

        command = genCommand(askForCommand(command), command)            
        
        pub.publish(command)

        rospy.sleep(0.1)
                        

if __name__ == '__main__':
    publisher()
