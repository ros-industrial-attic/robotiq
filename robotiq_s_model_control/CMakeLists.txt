cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_control)
find_package(catkin REQUIRED rospy message_generation roscpp robotiq_ethercat)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
  SModel_robot_input.msg
  SModel_robot_output.msg
)

catkin_python_setup()

generate_messages()

catkin_package(
    CATKIN_DEPENDS rospy message_runtime roscpp robotiq_ethercat
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${robotiq_ethercat_INCLUDE_DIRS}
)

add_executable(s_model_ethercat_node
  src/robotiq_s_model_control/s_model_ethercat_node.cpp
  src/robotiq_s_model_control/s_model_ethercat_client.cpp
)

target_link_libraries(s_model_ethercat_node
   ${robotiq_ethercat_LIBRARIES}
   ${catkin_LIBRARIES}
)

add_dependencies(s_model_ethercat_node robotiq_s_model_control_generate_messages_cpp)

install(PROGRAMS nodes/SModelSimpleController.py nodes/SModelStatusListener.py 
	nodes/SModelTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
