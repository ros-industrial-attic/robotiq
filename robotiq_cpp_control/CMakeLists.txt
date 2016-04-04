cmake_minimum_required(VERSION 2.8.3)
project(robotiq_cpp_control)

set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )


find_package(catkin REQUIRED COMPONENTS
  robotiq_s_model_control
  robotiq_c_model_control
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RSGripperCommand.msg
  RCGripperCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_library(robotiq_cpp_control src/gripper_interface.cpp src/rs_gripper_interface.cpp src/rc_gripper_interface.cpp)
add_dependencies(robotiq_cpp_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} robotiq_cpp_control_generate_messages_cpp)
target_link_libraries(robotiq_cpp_control ${catkin_LIBRARIES} )

add_executable(rs_gripper_interface_node src/gripper_interface.cpp  src/rs_gripper_interface_node.cpp)
add_dependencies(rs_gripper_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} robotiq_cpp_control robotiq_s_model_control robotiq_cpp_control_generate_messages_cpp)
target_link_libraries(rs_gripper_interface_node ${catkin_LIBRARIES} robotiq_cpp_control ) 

add_executable(rs_gripper_interface_test src/rs_gripper_interface_test.cpp)
add_dependencies(rs_gripper_interface_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} robotiq_cpp_control robotiq_cpp_control_generate_messages_cpp)
target_link_libraries(rs_gripper_interface_test ${catkin_LIBRARIES} robotiq_cpp_control ) 

add_executable(rc_gripper_interface_node src/gripper_interface.cpp  src/rc_gripper_interface_node.cpp)
add_dependencies(rc_gripper_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} robotiq_cpp_control robotiq_c_model_control robotiq_cpp_control_generate_messages_cpp)
target_link_libraries(rc_gripper_interface_node ${catkin_LIBRARIES} robotiq_cpp_control)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
