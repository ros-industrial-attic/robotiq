cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_articulated_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
    FILES
    SModelRobotInput.msg 
    SModelRobotOutput.msg
) 

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs
)

install(DIRECTORY msg
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


