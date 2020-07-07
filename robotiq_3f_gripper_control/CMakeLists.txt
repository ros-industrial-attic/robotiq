# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_gripper_control)
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diagnostic_updater
  dynamic_reconfigure
  hardware_interface
  robotiq_3f_gripper_articulated_msgs
  robotiq_ethercat
  roscpp
  rospy
  socketcan_interface
  std_srvs
)

find_package(PkgConfig)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Robotiq3FGripper.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        controller_manager
        diagnostic_updater
        dynamic_reconfigure
        hardware_interface
        robotiq_3f_gripper_articulated_msgs
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
  src/${PROJECT_NAME}/robotiq_3f_gripper_api.cpp
  src/${PROJECT_NAME}/robotiq_3f_gripper_diagnostics.cpp
  src/${PROJECT_NAME}/robotiq_3f_gripper_hw_interface.cpp
  src/${PROJECT_NAME}/robotiq_3f_gripper_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_executable(robotiq_3f_gripper_ethercat_node
  src/${PROJECT_NAME}/robotiq_3f_gripper_ethercat_client.cpp
  src/robotiq_3f_gripper_ethercat_node.cpp
)

target_link_libraries(robotiq_3f_gripper_ethercat_node
   ${PROJECT_NAME}
   ${robotiq_ethercat_LIBRARIES}
   ${catkin_LIBRARIES}
)

add_executable(robotiq_3f_gripper_can_node
  src/robotiq_3f_gripper_can_node.cpp
  src/${PROJECT_NAME}/robotiq_3f_gripper_can_client.cpp
)

target_link_libraries(robotiq_3f_gripper_can_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(PROGRAMS nodes/Robotiq3FGripperSimpleController.py nodes/Robotiq3FGripperStatusListener.py
        nodes/Robotiq3FGripperTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS robotiq_3f_gripper_can_node ${PROJECT_NAME} robotiq_3f_gripper_ethercat_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
