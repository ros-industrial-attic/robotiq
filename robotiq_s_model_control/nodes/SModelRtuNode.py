#!/usr/bin/env python

"""@package docstring
ROS node for controling a Robotiq S-Model gripper using the Modbus RTU protocol.

The script takes as an argument the device address of the gripper (e.g. /dev/ttyUSB0). Afterwards it acts like the
SModelTcpNode.
"""

import roslib; roslib.load_manifest('robotiq_s_model_control')
roslib.load_manifest('robotiq_modbus_tcp')
import rospy
import robotiq_s_model_control.baseSModel
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from robotiq_s_model_control.msg import _SModel_robot_output as outputMsg


def mainLoop(device):
    # Gripper is a S-Model with a TCP connection
    gripper = robotiq_s_model_control.baseSModel.robotiqBaseSModel()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication(retry=True)

    # We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    rospy.init_node('robotiqSModel')

    # The Gripper status is published on the topic named 'SModelRobotInput'
    pub = rospy.Publisher('SModelRobotInput', inputMsg.SModel_robot_input, queue_size=1)

    # The Gripper command is received from the topic named 'SModelRobotOutput'
    rospy.Subscriber('SModelRobotOutput', outputMsg.SModel_robot_output, gripper.refreshCommand)

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
