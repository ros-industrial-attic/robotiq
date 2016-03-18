cmake_minimum_required(VERSION 2.8.3)
project(robotiq_joint_state_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  roscpp
  roslib
  sensor_msgs
  robotiq_s_model_control
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
 CATKIN_DEPENDS roscpp roslib sensor_msgs robotiq_s_model_control
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(s_model_joint_states src/s_model_joint_states.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(s_model_joint_states robotiq_s_model_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(s_model_joint_states
    ${catkin_LIBRARIES}
)

