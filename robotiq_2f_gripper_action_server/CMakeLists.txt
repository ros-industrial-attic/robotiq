cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  robotiq_2f_gripper_control
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs robotiq_2f_gripper_control roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)

# The action server
add_executable(${PROJECT_NAME}_node
  include/${PROJECT_NAME}/${PROJECT_NAME}.h
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${robotiq_2f_gripper_control_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

 # The test server
add_executable(${PROJECT_NAME}_client_test
  src/${PROJECT_NAME}_client_test.cpp
)

add_dependencies(${PROJECT_NAME}_client_test
  ${PROJECT_NAME}_generate_messages_cpp
  ${robotiq_action_server_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_client_test
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_client_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
