# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_control)
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diagnostic_updater
  dynamic_reconfigure
  hardware_interface
  message_generation
  robotiq_ethercat
  roscpp
  rospy
  socketcan_interface
  std_srvs
)

find_package(PkgConfig)

###################################
## catkin message generation     ##
###################################
add_message_files(
  FILES
  SModel_robot_input.msg
  SModel_robot_output.msg
)

catkin_python_setup()

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/SModel.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        controller_manager
        diagnostic_updater
        dynamic_reconfigure
        hardware_interface
        message_runtime
        robotiq_ethercat
        roscpp
        rospy
        socketcan_interface
        std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/s_model_api.cpp
  src/${PROJECT_NAME}/s_model_diagnostics.cpp
  src/${PROJECT_NAME}/s_model_hw_interface.cpp
  src/${PROJECT_NAME}/s_model_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(s_model_ethercat_node
  src/${PROJECT_NAME}/s_model_ethercat_client.cpp
  src/s_model_ethercat_node.cpp
)

target_link_libraries(s_model_ethercat_node
   ${PROJECT_NAME}
   ${robotiq_ethercat_LIBRARIES}
   ${catkin_LIBRARIES}
)

add_executable(s_model_can_node
  src/${PROJECT_NAME}/s_model_can_client.cpp
  src/s_model_can_node.cpp
)

target_link_libraries(s_model_can_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS nodes/SModelSimpleController.py nodes/SModelStatusListener.py 
	nodes/SModelTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} s_model_can_node s_model_ethercat_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
