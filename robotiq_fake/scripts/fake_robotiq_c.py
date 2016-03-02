#!/usr/bin/env python

"""
Publish fake Robotiq status data to /CModelRobotInput to simulate opening and closing.
"""

# Copyright (c) <2016>, <Matthew Horn>
# All rights reserved.
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this 
# list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation and/or
#  other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors 
# may be used to endorse or promote products derived from this software without
#  specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rospy
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from math import ceil


# definitions
CLOSE_TIME = 0.68  # seconds 0.68 - 3.85
CLOSE_TIME_SLOW_DIFF = 3.85
UPDATE_RATE_HZ = 30
MOTION = True


# improve log messages
NODE_NAME = 'fake_robotiq_c'
def rosinfo(msg):
    rospy.loginfo('['+NODE_NAME+'] '+str(msg))


# helper function
def sign(value):
    if value >= 0:
        return 1
    else:
        return -1


# helper function
def clamp(value, min_value=0, max_value=255):
    return max(min(value, max_value), min_value)


def gripper_move(pos, speed):
    CLOSED_POSITION = 255
    #rosinfo(speed)
    
    # calculate distance fingers will move
    dist = clamp(abs(clamp(pos, 0, CLOSED_POSITION)-state.gPO), 0, CLOSED_POSITION)


    # status to moving
    rosinfo('moving dist: '+str(dist))
    state.gSTA = 0

    # estimate time; it takes CLOSE_TIME to move 255 counts
    time = (dist/255.0)*(CLOSE_TIME + CLOSE_TIME_SLOW_DIFF*(255-speed)/255)

    steps = int(ceil(time*UPDATE_RATE_HZ))

    dp = 0

    if steps > 0:
        dp = ceil(dist/steps)

    if pos < state.gPO:  # opening
        dp *= -1

    direction = sign(dp)

    while (dist) > 0:
        # calculate distance
        # pos is clamped between 0 and CLOSED_POSITION
        # abs(pos-state.gPO) is clamped between 0 and CLOSED_POSITION
        dist = clamp(abs(clamp(pos, 0, CLOSED_POSITION)-state.gPO), 0, CLOSED_POSITION)
        # avoid overshoot
        if abs(clamp(pos, 0, CLOSED_POSITION)-state.gPO) < abs(dp):
            dp = dist
        # get direction right
        if dp > 0 and clamp(pos, 0, CLOSED_POSITION) < state.gPO or dp < 0 and clamp(pos, 0, CLOSED_POSITION) > state.gPO:
            dp *= -1
        
        state.gPO = clamp(state.gPO+MOTION*dp, 0, CLOSED_POSITION)
        rospy.sleep(1.0/UPDATE_RATE_HZ)
    state.gSTA = 3  # move complete
    rosinfo('move complete')

# callback to handle issued commands (e.g. from Rviz)
def callback_command(command):
    # rosinfo('callback')
    global state
    global previous_mode
    if command.rACT == 0:  # reset
        state.gACT = 0
        state.gGTO = 0
        previous_mode = 0
    else:
        if state.gACT == 0 and command.rACT == 1:  # initialize
            rosinfo('initializing')
            state.gACT = 1
            state.gSTA = 0
            gripper_move(0,command.rSP)
            gripper_move(255,command.rSP)
            gripper_move(0,command.rSP)
            state.gSTA = 3
            rosinfo('initialized')
        elif command.rACT == 1 and command.rGTO == 1: # basic
            # rosinfo('basic move to '+str(command.rPRA))
            state.gGTO = 1
            state.gSTA = 0
            # move to new position
            if command.rPR != state.gPO:
                gripper_move(command.rPR,command.rSP)
            else:
                gripper_move(command.rPR,command.rSP)
            state.gGTO = 1
            state.gSTA = 3


def fake_robotiq(state):
    # initialize the node
    rospy.init_node(NODE_NAME)
    r = rospy.Rate(UPDATE_RATE_HZ)
    
    # advertise and subscribe
    pub = rospy.Publisher('CModelRobotInput', inputMsg.CModel_robot_input, queue_size=1)
    sub = rospy.Subscriber('CModelRobotOutput', outputMsg.CModel_robot_output, callback_command)

    # create message
    state.gPO = 0   # digit A in position 0 (fully open)

    # main loop
    while not rospy.is_shutdown():
        pub.publish(state)
        r.sleep()


if __name__ == '__main__':
    try:
        state = inputMsg.CModel_robot_input()
        fake_robotiq(state)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rosinfo(e.message)
