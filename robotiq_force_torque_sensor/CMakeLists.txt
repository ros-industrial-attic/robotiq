cmake_minimum_required(VERSION 2.8.3)
project(robotiq_force_torque_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    ft_sensor.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    sensor_accessor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

# make the executables
add_executable(rq_sensor nodes/rq_sensor.cpp src/rq_sensor_com.cpp src/rq_sensor_state.cpp)
add_dependencies(rq_sensor ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(rq_sensor ${catkin_LIBRARIES})

add_executable(rq_test_sensor nodes/rq_test_sensor.cpp)
add_dependencies(rq_test_sensor ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(rq_test_sensor ${catkin_LIBRARIES})

install(TARGETS rq_sensor rq_test_sensor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
