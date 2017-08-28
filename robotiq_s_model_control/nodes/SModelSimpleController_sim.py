#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from threading import Lock
from robotiq_s_model_control.msg import _SModel_robot_output as outputMsg
from time import sleep
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput as outputMsgSim
from robotiq_s_model_control.msg import _SModel_robot_input as inputMsg


# XXX: These commands are copied from ./src/yuripy/scripts/yuri_webcontroller.py

# XXX: The left or right side configuration of the hand is defined at
# ./src/robotiq/robotiq_s_model_visualization/cfg/robotiq_hand_macro_with_gazebo_plugin.urdf.xacro
left_hand_active = True
right_hand_active = True

class Gripper(object):
    def __init__(self):
        rospy.init_node('SModelSimpleControllerSim')
        if left_hand_active == True:
            self._pub_left_hand = rospy.Publisher\
                    ('/left_hand/command', outputMsgSim, queue_size=1)
        if right_hand_active == True:
            self._pub_right_hand = rospy.Publisher\
                    ('/right_hand/command', outputMsgSim, queue_size=1)
 
        self._sub = rospy.Subscriber("/SModelRobotInput",\
                        inputMsg.SModel_robot_input, self._status_handler)
        self._status_lock = Lock()

        # Updated from gripper topic
        self._status = {'moving':True, 'activated':False}

    def init(self):
        self.activate()
        pass
    
    def activate(self):
        command = outputMsg.SModel_robot_output()
        command.rACT = 1 # Activate gripper if not already
        command.rMOD = 1 # Start in pinch mode for Honda demo
        command.rGTO = 1 # GOTO-mode
        command.rSPA = 255 # Max Speed
        command.rSPB = 255 # Max Speed
        command.rSPC = 255 # Max Speed
        command.rPRA = 0 # Default to open
        command.rPRB = 0 # Default to open
        command.rPRC = 0 # Default to open
        command.rFRA = 255 # Force
        command.rFRB = 255 # Force
        command.rFRC = 255 # Force
        self._pub.publish(command)

    def _status_handler(self, data):
        self._status_lock.acquire(True)
        self._status = {}
        self._status['position'] = (data.gPOA, data.gPOB, data.gPOC)
        self._status['mode'] = data.gMOD
        self._status['activated'] = data.gACT is not 0
        self._status['moving'] = data.gSTA is 0
        self._status_lock.release()

    def set_position(self, val, mode=1):
        '''
        Move gripper to set position between 0-255,
        in mode={0: basic, 1: pinch, 2: wide, 3: scissor}
        '''
        command = outputMsg.SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rMOD = mode
        command.rSPA = 255
        command.rFRA = 150
        command.rFRA = 255 # Force
        command.rFRB = 255 # Force
        command.rFRC = 255 # Force
        if isinstance(val, int) is True:
            if val > 255:
                val = 255
            if val < 0:
                val = 0
            command.rPRA = val
            command.rPRB = val
            command.rPRC = val
        elif (isinstance(val, list) is True or isinstance(val, tuple) is True)\
                and len(val) == 3:
            command.rPRA, command.rPRB, command.rPRC = val
        else:
            rospy.logerr("Invalid value provided to set_gripper_position, %s",\
                    str(val))
            return
        self._pub.publish(command)

    def is_moving(self):
        '''
        True if the gripper is moving position
        '''
        return self._status['moving']


def activate():
    command = outputMsgSim
    command.rACT = 1 # Activate gripper if not already
    command.rMOD = 1 # Start in pinch mode for Honda demo
    command.rGTO = 1 # GOTO-mode
    command.rSPA = 255 # Max Speed
    command.rSPB = 255 # Max Speed
    command.rSPC = 255 # Max Speed
    command.rPRA = 0 # Default to open
    command.rPRB = 0 # Default to open
    command.rPRC = 0 # Default to open
    command.rFRA = 255 # Force
    command.rFRB = 255 # Force
    command.rFRC = 255 # Force
    return command

def _status_handler(self, data):
    self._status_lock.acquire(True)
    self._status = {}
    self._status['position'] = (data.gPOA, data.gPOB, data.gPOC)
    self._status['mode'] = data.gMOD
    self._status['activated'] = data.gACT is not 0
    self._status['moving'] = data.gSTA is 0
    self._status_lock.release()

def set_position(self, val, mode=1):
    '''
    Move gripper to set position between 0-255,
    in mode={0: basic, 1: pinch, 2: wide, 3: scissor}
    '''
    command = outputMsg.SModel_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rMOD = mode
    command.rSPA = 255
    command.rFRA = 150
    command.rFRA = 255 # Force
    command.rFRB = 255 # Force
    command.rFRC = 255 # Force
    if isinstance(val, int) is True:
        if val > 255:
            val = 255
        if val < 0:
            val = 0
        command.rPRA = val
        command.rPRB = val
        command.rPRC = val
    elif (isinstance(val, list) is True or isinstance(val, tuple) is True)\
            and len(val) == 3:
        command.rPRA, command.rPRB, command.rPRC = val
    else:
        rospy.logerr("Invalid value provided to set_gripper_position, %s",\
                str(val))
        return
    self._pub.publish(command)

def is_moving(self):
    '''
    True if the gripper is moving position
    '''
    return self._status['moving']
                   

if __name__ == '__main__':
    gripper = Gripper()
    import IPython;IPython.embed()
