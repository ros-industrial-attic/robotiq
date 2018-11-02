#!/usr/bin/env python

"""@package docstring
ROS node for controling a Robotiq S-Model gripper using the Modbus RTU protocol.

The script takes as an argument the device address of the gripper (e.g. /dev/ttyUSB0). Afterwards it acts like the
Robotiq3FGripperTcpNode.
"""

import roslib; roslib.load_manifest('robotiq_3f_gripper_control')
roslib.load_manifest('robotiq_modbus_tcp')
import rospy
import robotiq_3f_gripper_control.baseRobotiq3FGripper
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from robotiq_3f_gripper_control.msg import _Robotiq3FGripper_robot_input  as inputMsg
from robotiq_3f_gripper_control.msg import _Robotiq3FGripper_robot_output as outputMsg


def mainLoop(device):
    # Gripper is a 3F gripper with a TCP connection
    gripper = robotiq_3f_gripper_control.baseRobotiq3FGripper.robotiqbaseRobotiq3FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication(retry=True)

    # We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    rospy.init_node('robotiq3FGripper')

    # The Gripper status is published on the topic named 'Robotiq3FGripperRobotInput'
    pub = rospy.Publisher('Robotiq3FGripperRobotInput', inputMsg.Robotiq3FGripper_robot_input, queue_size=1)

    # The Gripper command is received from the topic named 'Robotiq3FGripperRobotOutput'
    rospy.Subscriber('Robotiq3FGripperRobotOutput', outputMsg.Robotiq3FGripper_robot_output, gripper.refreshCommand)

    # We loop
    while not rospy.is_shutdown():
        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)

        # Wait a little
        rospy.sleep(0.05)

        # Send the most recent command
        gripper.sendCommand()

        # Wait a little
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        # TODO: Add verification that the argument is an IP address
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        # TODO: Add verification that the argument is a valid device
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException: pass
