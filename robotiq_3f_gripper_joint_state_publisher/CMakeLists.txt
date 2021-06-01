cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_gripper_joint_state_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  robotiq_3f_gripper_control
  roscpp
  roslib
  sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS message_runtime robotiq_3f_gripper_control roscpp roslib sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(robotiq_3f_gripper_joint_states src/robotiq_3f_gripper_joint_states.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(robotiq_3f_gripper_joint_states robotiq_3f_gripper_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotiq_3f_gripper_joint_states
    ${catkin_LIBRARIES}
)

install(TARGETS robotiq_3f_gripper_joint_states
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

