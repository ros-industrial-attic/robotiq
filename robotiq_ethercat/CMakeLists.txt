cmake_minimum_required(VERSION 2.8.3)
project(robotiq_ethercat)

find_package(catkin REQUIRED COMPONENTS
  ethercat_soem
  roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES robotiq_ethercat
   CATKIN_DEPENDS ethercat_soem roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(robotiq_ethercat
 include/robotiq_ethercat/ethercat_manager.h
 src/ethercat_manager.cpp
)

install(TARGETS robotiq_ethercat
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
