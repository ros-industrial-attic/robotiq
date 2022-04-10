#!/usr/bin/env python

import roslib; 
roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from sensor_msgs.msg import Joy


def joy_listener():

    # start node
    rospy.init_node("joy_controller", anonymous=True)

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("/joy", Joy, joy_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()



#GLOBAL TEMP
command = outputMsg.Robotiq2FGripper_robot_output()
# command.rACT = 0
# command.rATR = 0
# command.rFR = 0
# command.GTO = 0
# command.rPR = 0
# command.rSP = 0


# called when joy cmd_joy message is received
def joy_callback(data):
    global command
    
    button_B = data.buttons[1]
    button_X = data.buttons[2]

    button_back = data.buttons[6]
    button_start = data.buttons[7]
    
    #activate (droite du bouton Xbox)
    if button_start:
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rFR = 150
        command.rGTO = 1
        command.rSP = 255
    #reset (gauche du bouton Xbox)
    elif button_back:
        command = outputMsg.Robotiq2FGripper_robot_output()
    #ferme
    elif button_B:
        command.rPR = 255
    #ouvre
    elif button_X:
        command.rPR = 0
   
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    pub.publish(command)



if __name__ == '__main__':
    try:                      
        joy_listener()

    except rospy.ROSInterruptException:
        pass