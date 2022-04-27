#!/usr/bin/env python

import roslib; 
roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from sensor_msgs.msg import Joy

#GLOBAL TEMP
command = outputMsg.Robotiq2FGripper_robot_output()


def joy_listener():
    global command
    rospy.init_node("joy_controller", anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback, queue_size=1)
    rospy.spin()


def joy_callback(data):
    global command
    
    button_B = data.buttons[1]
    button_X = data.buttons[2]
    button_Y = data.buttons[3]

    if button_Y:
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1  #activates the gripper 
        command.rGTO = 1  #Go to resqueted position
        command.rFR = 255 #maximum force
        command.rSP = 255 #Maximum speed
    
    elif button_B:
        command.rPR = 255 #close
    
    elif button_X:
        command.rPR = 0 #open
   
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    pub.publish(command)



if __name__ == '__main__':
    try:
                              
        joy_listener()

    except rospy.ROSInterruptException:
        pass