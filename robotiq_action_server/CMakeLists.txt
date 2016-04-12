cmake_minimum_required(VERSION 2.8.3)
project(robotiq_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  robotiq_c_model_control
  roscpp
  control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs robotiq_c_model_control roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${robotiq_action_server_INCLUDE_DIRS}
)

# The action server
add_executable(robotiq_c_model_action_server_node
    src/robotiq_c_model_action_server_node.cpp
    src/robotiq_c_model_action_server.cpp
    include/robotiq_action_server/robotiq_c_model_action_server.h)

add_dependencies(robotiq_c_model_action_server_node
  robotiq_action_server_generate_messages_cpp
  ${robotiq_action_server_EXPORTED_TARGETS}
  ${robotiq_c_model_control_EXPORTED_TARGETS})

 target_link_libraries(robotiq_c_model_action_server_node
   ${catkin_LIBRARIES}
 )

 # The test server
add_executable(robotiq_action_server_client_test
  src/robotiq_action_server_client_test.cpp
)

add_dependencies(robotiq_action_server_client_test
  robotiq_action_server_generate_messages_cpp
  ${robotiq_action_server_EXPORTED_TARGETS})

 target_link_libraries(robotiq_action_server_client_test
   ${catkin_LIBRARIES}
 )


install(TARGETS robotiq_c_model_action_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS robotiq_action_server_client_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )
