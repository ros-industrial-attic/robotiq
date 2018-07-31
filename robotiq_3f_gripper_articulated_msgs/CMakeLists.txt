cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_gripper_articulated_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation )

add_message_files(
    FILES
    Robotiq3FGripperRobotInput.msg
    Robotiq3FGripperRobotOutput.msg
) 

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    DEPENDS std_msgs
)


