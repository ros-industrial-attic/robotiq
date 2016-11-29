#!/usr/bin/env python

"""
Publish fake Robotiq status data to /SModelRobotInput to simulate opening and closing.
"""

# The MIT License (MIT)
# 
# Copyright (c) 2014 Jack Thompson
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPOESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import rospy
from robotiq_s_model_control.msg import _SModel_robot_input as inputMsg
from robotiq_s_model_control.msg import _SModel_robot_output as outputMsg
from math import ceil


# definitions
CLOSE_TIME = 2.0  # seconds
UPDATE_RATE_HZ = 30
MOTION_A = True
MOTION_B = True
MOTION_C = True
MOTION_S = True


# improve log messages
NODE_NAME = 'fake_robotiq'
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


def basic_move(state, pos):
    gripper_move(state, pos, pos, pos, 137)


def gripper_move(state, pos_A, pos_B, pos_C, pos_S):
    if state.gACT == 1:

        CLOSED_POSITION = 255
        if state.gMOD == 1:  # pinch
            CLOSED_POSITION = 104

        # calculate distance fingers will move
        dist_A = clamp(abs(clamp(pos_A, 0, CLOSED_POSITION)-state.gPOA), 0, CLOSED_POSITION)
        dist_B = clamp(abs(clamp(pos_B, 0, CLOSED_POSITION)-state.gPOB), 0, CLOSED_POSITION)
        dist_C = clamp(abs(clamp(pos_C, 0, CLOSED_POSITION)-state.gPOC), 0, CLOSED_POSITION)
        dist_S = clamp(abs(clamp(pos_S, 0, 255)-state.gPOS), 0, 255)
        dist = max([dist_A, dist_B, dist_C, dist_S])

        if dist == 0:
            return
        # status to moving
        rosinfo('moving dist: '+str(dist))
        state.gSTA = 0  # move complete
        state.gDTA = 0
        state.gDTB = 0
        state.gDTC = 0
        state.gDTS = 0

        # estimate time; it takes CLOSE_TIME to move 255 counts
        time_A = (dist_A/255.0)*CLOSE_TIME
        time_B = (dist_B/255.0)*CLOSE_TIME
        time_C = (dist_C/255.0)*CLOSE_TIME
        time_S = (dist_S/255.0)*CLOSE_TIME

        steps_A = int(ceil(time_A*UPDATE_RATE_HZ))
        steps_B = int(ceil(time_B*UPDATE_RATE_HZ))
        steps_C = int(ceil(time_C*UPDATE_RATE_HZ))
        steps_S = int(ceil(time_S*UPDATE_RATE_HZ))

        dp_A = 0
        dp_B = 0
        dp_C = 0
        dp_S = 0

        if steps_A > 0:
            dp_A = ceil(dist_A/steps_A)
        if steps_B > 0:
            dp_B = ceil(dist_B/steps_B)
        if steps_C > 0:
            dp_C = ceil(dist_C/steps_C)
        if steps_S > 0:
            dp_S = ceil(dist_S/steps_S)

        if pos_A < state.gPOA:  # opening
            dp_A *= -1
        if pos_B < state.gPOB:  # opening
            dp_B *= -1
        if pos_C < state.gPOC:  # opening
            dp_C *= -1
        if pos_S < state.gPOS:  # opening
            dp_S *= -1
        direction_A = sign(dp_A)
        direction_B = sign(dp_B)
        direction_C = sign(dp_C)
        direction_S = sign(dp_S)

        # rosinfo('from:          '+str(state.gPOA))
        # rosinfo('to:            '+str(pos_A))
        # rosinfo('time:          '+str(time_A))
        # rosinfo('dp:            '+str(dp_A))
        # rosinfo('steps:         '+str(steps_A))
        # rosinfo('dp*steps:      '+str(dp_A*steps_A))
        # rosinfo('dp*steps-dist: '+str((dp_A*steps_A)-dist_A))

        # rosinfo('from:          '+str(state.gPOS))
        # rosinfo('to:            '+str(pos_S))
        # rosinfo('time:          '+str(time_S))
        # rosinfo('dp:            '+str(dp_S))
        # rosinfo('steps:         '+str(steps_S))
        # rosinfo('dp*steps:      '+str(dp_S*steps_S))
        # rosinfo('dp*steps-dist: '+str((dp_S*steps_S)-dist_S))

        while (dist_A + dist_B + dist_C + dist_S) > 0:
            # calculate distance
            # pos is clamped between 0 and CLOSED_POSITION
            # abs(pos-state.gPOA) is clamped between 0 and CLOSED_POSITION
            dist_A = clamp(abs(clamp(pos_A, 0, CLOSED_POSITION)-state.gPOA), 0, CLOSED_POSITION)
            dist_B = clamp(abs(clamp(pos_B, 0, CLOSED_POSITION)-state.gPOB), 0, CLOSED_POSITION)
            dist_C = clamp(abs(clamp(pos_C, 0, CLOSED_POSITION)-state.gPOC), 0, CLOSED_POSITION)
            dist_S = clamp(abs(clamp(pos_S, 0, 255)-state.gPOS), 0, 255)

            # avoid overshoot
            if abs(clamp(pos_A, 0, CLOSED_POSITION)-state.gPOA) < abs(dp_A):
                dp_A = dist_A
            if abs(clamp(pos_B, 0, CLOSED_POSITION)-state.gPOB) < abs(dp_B):
                dp_B = dist_B
            if abs(clamp(pos_C, 0, CLOSED_POSITION)-state.gPOC) < abs(dp_C):
                dp_C = dist_C
            if abs(clamp(pos_S, 0, 255)-state.gPOS) < abs(dp_S):
                dp_S = dist_S
            # get direction right
            if dp_A > 0 and clamp(pos_A, 0, CLOSED_POSITION) < state.gPOA or dp_A < 0 and clamp(pos_A, 0, CLOSED_POSITION) > state.gPOA:
                dp_A *= -1
            if dp_B > 0 and clamp(pos_B, 0, CLOSED_POSITION) < state.gPOB or dp_B < 0 and clamp(pos_B, 0, CLOSED_POSITION) > state.gPOB:
                dp_B *= -1
            if dp_C > 0 and clamp(pos_C, 0, CLOSED_POSITION) < state.gPOC or dp_C < 0 and clamp(pos_C, 0, CLOSED_POSITION) > state.gPOC:
                dp_C *= -1
            if dp_S > 0 and clamp(pos_S, 0, 255) < state.gPOS or dp_S < 0 and clamp(pos_S, 0, 255) > state.gPOS:
                dp_S *= -1
            state.gPOA = clamp(state.gPOA+MOTION_A*dp_A, 0, CLOSED_POSITION)
            state.gPOB = clamp(state.gPOB+MOTION_B*dp_B, 0, CLOSED_POSITION)
            state.gPOC = clamp(state.gPOC+MOTION_C*dp_C, 0, CLOSED_POSITION)
            state.gPOS = clamp(state.gPOS+MOTION_S*dp_S, 0, 255)
            rospy.sleep(1.0/UPDATE_RATE_HZ)
        state.gSTA = 3  # move complete
        state.gDTA = 3
        state.gDTB = 3
        state.gDTC = 3
        state.gDTS = 3
	state.gDTA = 3
	state.gDTB = 3
	state.gDTC = 3
	state.gDTS = 3
        rosinfo('move complete')

# callback to handle issued commands (e.g. from Rviz)
def callback_command(command):
    # rosinfo('callback')
    global state
    global previous_mode
    if command.rACT == 0:  # reset
        state.gACT = 0
        state.gIMC = 0
        state.gGTO = 0
        previous_mode = 0
    else:
        state.gMOD = command.rMOD
        if state.gACT == 0 and command.rACT == 1:  # initialize
            rosinfo('initializing')
            state.gACT = 1
            state.gIMC = 1
            state.gMOD = 0
            state.gSTA = 0
            state.gDTA = 0
            state.gDTB = 0
            state.gDTC = 0
            state.gDTS = 0
            gripper_move(state, 0, 0, 0, state.gPOS)
            gripper_move(state, 0, 0, 0, 0)
            gripper_move(state, 0, 0, 0, 255)
            gripper_move(state, 0, 0, 0, 137)
            gripper_move(state, 255, 255, 255, 137)
            gripper_move(state, 0, 0, 0, 137)
            state.gIMC = 3
            state.gSTA = 3
            state.gDTA = 3
            state.gDTB = 3
            state.gDTC = 3
            state.gDTS = 3
            rosinfo('initialized')
        elif command.rACT == 1 and command.rGTO == 1 and command.rICF == 1: # individual control
            rosinfo('individual control')
            state.gGTO = 1
            state.gSTA = 0
            state.gDTA = 0
            state.gDTB = 0
            state.gDTC = 0
            state.gDTS = 0
            # change mode
            if previous_mode == 0:  # basic
                gripper_move(state, 0, 0, 0, 137)
            elif previous_mode == 1:  # pinch
                gripper_move(state, 0, 0, 0, 255)
                gripper_move(state, 0, 0, 0, 137)
            elif previous_mode == 2:  # wide
                gripper_move(state, 0, 0, 0, 0)
                gripper_move(state, 0, 0, 0, 137)
            elif previous_mode == 3:  # scissor
                gripper_move(state, 0, 0, 0, 137)
            # move to new position
            if command.rPRA != state.gPOA or command.rPRB != state.gPOB or command.rPRC != state.gPOC or command.rPRS != state.gPOS:
                gripper_move(state, command.rPRA, command.rPRB, command.rPRC, command.rPRS)
            state.gGTO = 1
            state.gSTA = 3
            state.gDTA = 3
            state.gDTB = 3
            state.gDTC = 3
            state.gDTS = 3
        elif command.rACT == 1 and command.rGTO == 1 and command.rICF == 0 and state.gMOD == 0: # basic
            # rosinfo('basic move to '+str(command.rPRA))
            state.gGTO = 1
            state.gSTA = 0
            state.gDTA = 0
            state.gDTB = 0
            state.gDTC = 0
            state.gDTS = 0
            # change mode
            if previous_mode == 1:  # pinch
                rosinfo('basic')
                gripper_move(state, 0, 0, 0, 255)
                gripper_move(state, 0, 0, 0, 137)
            elif previous_mode == 2:  # wide
                rosinfo('basic')
                gripper_move(state, 0, 0, 0, 0)
                gripper_move(state, 0, 0, 0, 137)
            elif previous_mode == 3:  # scissor
                rosinfo('basic')
                gripper_move(state, 0, 0, 0, 137)
            # move to new position
            if command.rPRA != state.gPOA:
                gripper_move(state, command.rPRA, command.rPRA, command.rPRA, 137)
            previous_mode = 0
            state.gGTO = 1
            state.gSTA = 3
            state.gDTA = 3
            state.gDTB = 3
            state.gDTC = 3
            state.gDTS = 3
        elif command.rACT == 1 and command.rGTO == 1 and command.rICF == 0 and state.gMOD == 1: # pinch
            state.gGTO = 1
            state.gSTA = 0
            state.gDTA = 0
            state.gDTB = 0
            state.gDTC = 0
            state.gDTS = 0
            # change mode
            if previous_mode == 0:  # basic
                rosinfo('pinch')
                gripper_move(state, 0, 0, 0, 137)
                gripper_move(state, 0, 0, 0, 255)
            elif previous_mode == 2:  # wide
                rosinfo('pinch')
                gripper_move(state, 0, 0, 0, 0)
                gripper_move(state, 0, 0, 0, 255)
            elif previous_mode == 3:  # scissor
                rosinfo('pinch')
                gripper_move(state, 0, 0, 0, 255)
            # move to new position
            if command.rPRA != state.gPOA:
                gripper_move(state, command.rPRA, command.rPRA, command.rPRA, 255)
            previous_mode = 1
            state.gGTO = 1
            state.gSTA = 3
            state.gDTA = 3
            state.gDTB = 3
            state.gDTC = 3
            state.gDTS = 3
        elif command.rACT == 1 and command.rGTO == 1 and command.rICF == 0 and state.gMOD == 2: # wide
            state.gGTO = 1
            state.gSTA = 0
            state.gDTA = 0
            state.gDTB = 0
            state.gDTC = 0
            state.gDTS = 0
            # change mode
            if previous_mode == 0:  # basic
                rosinfo('wide')
                gripper_move(state, 0, 0, 0, 137)
                gripper_move(state, 0, 0, 0, 0)
            elif previous_mode == 1:  # pinch
                rosinfo('wide')
                gripper_move(state, 0, 0, 0, 255)
                gripper_move(state, 0, 0, 0, 0)
            elif previous_mode == 3:  # scissor
                rosinfo('wide')
                gripper_move(state, 0, 0, 0, 0)
            # move to new position
            if command.rPRA != state.gPOA:
                gripper_move(state, command.rPRA, command.rPRA, command.rPRA, 0)
            previous_mode = 2
            state.gGTO = 1
            state.gSTA = 3
            state.gDTA = 3
            state.gDTB = 3
            state.gDTC = 3
            state.gDTS = 3
        elif command.rACT == 1 and command.rGTO == 1 and command.rICS == 0 and state.gMOD == 3: # scissor
            state.gGTO = 1
            state.gSTA = 0
            state.gDTA = 0
            state.gDTB = 0
            state.gDTC = 0
            state.gDTS = 0
            # change mode
            if previous_mode == 0:  # basic
                rosinfo('scissor')
                gripper_move(state, 0, 0, 0, 137)
                gripper_move(state, 0, 0, 0, 0)
            elif previous_mode == 1:  # pinch
                rosinfo('scissor')
                gripper_move(state, 0, 0, 0, 255)
                gripper_move(state, 0, 0, 0, 0)
            elif previous_mode == 2:  # wide
                rosinfo('scissor')
                gripper_move(state, 0, 0, 0, 0)
            # move to new position
            if command.rPRA != state.gPOA:
                gripper_move(state, 0, 0, 0, command.rPRA)
            previous_mode = 3
            state.gGTO = 1
            state.gSTA = 3
            state.gDTA = 3
            state.gDTB = 3
            state.gDTC = 3
            state.gDTS = 3


def fake_robotiq(state):
    # initialize the node
    rospy.init_node(NODE_NAME)
    r = rospy.Rate(UPDATE_RATE_HZ)
    
    # advertise and subscribe
    pub = rospy.Publisher('SModelRobotInput', inputMsg.SModel_robot_input, queue_size=1)
    sub = rospy.Subscriber('SModelRobotOutput', outputMsg.SModel_robot_output, callback_command)

    # create message
    state.gPOA = 0   # digit A in position 0 (fully open)
    state.gPOB = 0   # digit B in position 0 (fully open)
    state.gPOC = 0   # digit C in position 0 (fully open)
    state.gPOS = 137   # scissor axis in position for basic mode

    # main loop
    while not rospy.is_shutdown():
        pub.publish(state)
        r.sleep()


if __name__ == '__main__':
    try:
        state = inputMsg.SModel_robot_input()
        previous_mode = state.gMOD
        fake_robotiq(state)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rosinfo(e.message)
