cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_gripper_articulated_gazebo_plugins)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS std_msgs gazebo_plugins actionlib tf image_transport control_msgs trajectory_msgs geometry_msgs sensor_msgs roscpp gazebo_ros robotiq_3f_gripper_articulated_msgs)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
   DEPENDS gazebo_plugins gazebo_ros robotiq_3f_gripper_articulated_msgs roscpp
)

add_library(RobotiqHandPlugin src/RobotiqHandPlugin.cpp)
set_target_properties(RobotiqHandPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(RobotiqHandPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(RobotiqHandPlugin ${catkin_LIBRARIES})
add_dependencies(RobotiqHandPlugin robotiq_3f_gripper_articulated_msgs_gencpp)

install(TARGETS RobotiqHandPlugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)

