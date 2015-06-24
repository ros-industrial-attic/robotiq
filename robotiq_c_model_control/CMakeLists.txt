# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_c_model_control)
find_package(catkin REQUIRED COMPONENTS robotiq_ethercat roscpp rospy message_generation)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin message generation     ##
###################################
add_message_files(
  FILES
  CModel_robot_input.msg
  CModel_robot_output.msg
)

catkin_python_setup()

generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS rospy message_runtime roscpp robotiq_ethercat
    INCLUDE_DIRS include
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${robotiq_ethercat_INCLUDE_DIRS}
)

add_executable(c_model_ethercat_node 
  src/robotiq_c_model_control/c_model_ethercat_node.cpp 
  src/robotiq_c_model_control/c_model_ethercat_client.cpp
)

target_link_libraries(c_model_ethercat_node
   ${robotiq_ethercat_LIBRARIES}
   ${catkin_LIBRARIES}
  
)

add_dependencies(c_model_ethercat_node robotiq_c_model_control_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS nodes/CModelSimpleController.py nodes/CModelStatusListener.py 
    nodes/CModelTcpNode.py nodes/CModelRtuNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

